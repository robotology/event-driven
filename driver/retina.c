/*
 * USB retina driver
 * Martin Ebner, IGI / TU Graz (ebner at igi.tugraz.at)
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License as
 *	published by the Free Software Foundation, version 2.
 *
 * This driver is based on the 2.6.3 version of drivers/usb/usb-skeleton.c
 * (Copyright (C) 2001-2004 Greg Kroah-Hartman (greg@kroah.com))
 *
 */

#include <linux/init.h>
#include <linux/usb.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <asm/uaccess.h>

///////////////////////////////////////////////////////////
// for pre 2.6.35, activate this section by changing 0 to 1

#if 1
#define usb_buffer_alloc usb_alloc_coherent 
#define usb_buffer_free usb_free_coherent 
#endif

// all this does not work as wished..:
//#if VERSION = 2 && PATCHLEVEL = 6 && SUBLEVEL < 35
//#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 35)
///////////////////////////////////////////////////////////


/* this is the DVS 128 retina */
#define USB_RETINA_VENDOR_ID	0x152a
#define USB_RETINA_PRODUCT_ID	0x8400

/* table of devices that work with this driver */
static struct usb_device_id retina_table [] = {
	{ USB_DEVICE(USB_RETINA_VENDOR_ID, USB_RETINA_PRODUCT_ID) },
	{ }					/* Terminating entry */
};
MODULE_DEVICE_TABLE(usb, retina_table);

#define MAX_TRANSFER (PAGE_SIZE - 512)
/* MAX_TRANSFER is chosen so that the VM is not stressed by
   allocations > PAGE_SIZE and the number of packets in a page
   is an integer 512 is the largest possible packet on EHCI */
#define USB_RETINA_MINOR_BASE	192
#define WRITES_IN_FLIGHT	8

/* endpoint for getting status info: 0x81  EP 1 BULK IN */
#define USB_ENDPOINT_STATUS_IN		0x81
/* endpoint for reading data: 0x02  EP 2 BULK OUT */
#define USB_ENDPOINT_OUT		0x02
/* endpoint for reading address events: 0x86  EP 6 BULK IN */
#define USB_ENDPOINT_ADDRESS_EVENTS_IN	0x86

#define VENDOR_REQUEST_START_TRANSFER	0xb3
#define VENDOR_REQUEST_STOP_TRANSFER	0xb4
#define VENDOR_REQUEST_SET_LED		0xbf

#define AE_BUF_SIZE			512

static struct usb_driver retina_driver;

/* Structure to hold all of our device specific stuff */
struct usb_retina {
	struct usb_device	*udev;			/* the usb device for this device */
	struct usb_interface	*interface;		/* the interface for this device */
	struct semaphore	limit_sem;		/* limiting the number of writes in progress */
	struct usb_anchor	submitted;		/* in case we need to retract our submissions */
	struct urb		*urb;			/* address event urb */
	unsigned char           *bulk_in_buffer;	/* the buffer to receive data */
	size_t			bulk_in_size;		/* the size of the receive buffer */
	__u8			bulk_in_endpointAddr;	/* the address of the bulk in endpoint */
	u32			event_counter;
	int			errors;			/* the last request tanked */
	int			open_count;		/* count the number of openers */
	spinlock_t		err_lock;		/* lock for errors */
	struct kref		kref;
	struct mutex		io_mutex;		/* synchronize I/O with disconnect */
};
#define to_retina_dev(d) container_of(d, struct usb_retina, kref)

static struct usb_driver retina_driver;
static void retina_draw_down(struct usb_retina *dev);

static inline int  vendorRequest(struct usb_retina *dev, u8 request, u16 value, u16 index, u8 * data, u16 length)
{
	return usb_control_msg(
	    dev->udev,
	    usb_sndctrlpipe(dev->udev, 0),
	    request,
	    USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
	    value,
	    index,
	    data,
	    length,
	    HZ);
}

static void retina_delete(struct kref *kref)
{
	struct usb_retina *dev = to_retina_dev(kref);

	/*dev_info(&dev->interface->dev,"retina_delete");*/
	usb_put_dev(dev->udev);
	kfree(dev->bulk_in_buffer);
	kfree(dev);
}

static int retina_open(struct inode *inode, struct file *file)
{
	struct usb_retina *dev;
	struct usb_interface *interface;
	int subminor;
	int retval = 0;

	subminor = iminor(inode);

	interface = usb_find_interface(&retina_driver, subminor);
	dev_info(&interface->dev,"retina_open()");
	if (!interface) {
		dev_err(&interface->dev,"%s - error, can't find device for minor %d", __FUNCTION__, subminor);
		retval = -ENODEV;
		goto exit;
	}

	dev = usb_get_intfdata(interface);
	if (!dev) {
		retval = -ENODEV;
		goto exit;
	}

	/* increment our usage count for the device */
	kref_get(&dev->kref);

	/* lock the device to allow correctly handling errors
	 * in resumption */
	mutex_lock(&dev->io_mutex);

	if (!dev->open_count++) {
		retval = usb_autopm_get_interface(interface);
			if (retval) {
				dev->open_count--;
				mutex_unlock(&dev->io_mutex);
				kref_put(&dev->kref, retina_delete);
				goto exit;
			}
	} /* else { //uncomment this block if you want exclusive open
		retval = -EBUSY;
		dev->open_count--;
		mutex_unlock(&dev->io_mutex);
		kref_put(&dev->kref, retina_delete);
		goto exit;
	} */
	/* prevent the device from being autosuspended */

	/* save our object in the file's private structure */
	file->private_data = dev;
	mutex_unlock(&dev->io_mutex);

exit:
	return retval;
}


static int retina_release(struct inode *inode, struct file *file)
{
	struct usb_retina *dev;

	dev = (struct usb_retina *)file->private_data;
	dev_info(&dev->interface->dev,"retina_release");
	if (dev == NULL)
		return -ENODEV;

	/* allow the device to be autosuspended */
	mutex_lock(&dev->io_mutex);
	if (!--dev->open_count && dev->interface)
		usb_autopm_put_interface(dev->interface);
	mutex_unlock(&dev->io_mutex);

	/* decrement the count on our device */
	kref_put(&dev->kref, retina_delete);
	return 0;
}

static int retina_flush(struct file *file, fl_owner_t id)
{
	struct usb_retina *dev;
	int res;

	dev = (struct usb_retina *)file->private_data;
	dev_info(&dev->interface->dev,"retina_flush");
	if (dev == NULL)
		return -ENODEV;

	/* wait for io to stop */
	mutex_lock(&dev->io_mutex);
	retina_draw_down(dev);

	/* read out errors, leave subsequent opens a clean slate */
	spin_lock_irq(&dev->err_lock);
	res = dev->errors ? (dev->errors == -EPIPE ? -EPIPE : -EIO) : 0;
	dev->errors = 0;
	spin_unlock_irq(&dev->err_lock);

	mutex_unlock(&dev->io_mutex);

	return res;
}

/* read system call of device file /dev/retina0*/

static ssize_t retina_read(struct file *file, char *buffer, size_t count, loff_t *ppos)
{
	struct usb_retina *dev;
	int retval;
	int bytes_read_sum=0, bytes_read=0, bytes_to_read=count;
	u8 blocks=1, i;

	dev = (struct usb_retina *)file->private_data;
	if (count > dev->bulk_in_size)
	{
		blocks = count / dev->bulk_in_size;
		bytes_to_read = dev->bulk_in_size;
	}
	/*dev_info(&dev->interface->dev,"retina_read %d blocks of %d bytes\n",blocks,bytes_to_read);*/

	mutex_lock(&dev->io_mutex);
	if (!dev->interface) {		/* disconnect() was called */
		retval = -ENODEV;
		goto exit;
	}

	for (i=0;i<blocks;i++)
	{

		/* do a blocking bulk read to get data from the device */
        bytes_read = 0;
		retval = usb_bulk_msg(dev->udev,
				      usb_rcvbulkpipe(dev->udev, dev->bulk_in_endpointAddr),
				      dev->bulk_in_buffer,
				      bytes_to_read,
				      &bytes_read, 10/*000*/);

		/* if the read was successful, copy the data to userspace */
		if (!retval) {

			/*dev_info(&dev->interface->dev,"event = {0x%x,0x%x,0x%x,0x%x}",dev->bulk_in_buffer[0],dev->bulk_in_buffer[1],dev->bulk_in_buffer[2],dev->bulk_in_buffer[3]);*/
			//if (copy_to_user(buffer+i*dev->bulk_in_size, dev->bulk_in_buffer, bytes_read)) //BUGGED
			if (copy_to_user(buffer+bytes_read_sum, dev->bulk_in_buffer, bytes_read))
			{	
			    retval = -EFAULT;
				break;
			}
			else
			{
				bytes_read_sum += bytes_read;
			}
		}
	}
	/*dev_info(&dev->interface->dev,"%d bytes read\n",bytes_read_sum);*/

exit:
	mutex_unlock(&dev->io_mutex);
	return bytes_read_sum;
}
/*
static void retina_write_bulk_callback(struct urb *urb)
{
	struct usb_retina *dev;

	dev = (struct usb_retina *)urb->context;
	dev_info(&dev->interface->dev,"retina_write_bulk_callback");

	if (urb->status) {
		if(!(urb->status == -ENOENT ||
		    urb->status == -ECONNRESET ||
		    urb->status == -ESHUTDOWN))
			dev_err(&dev->interface->dev,"%s - nonzero write bulk status received: %d",
			    __FUNCTION__, urb->status);

		spin_lock(&dev->err_lock);
		dev->errors = urb->status;
		spin_unlock(&dev->err_lock);
	}

	usb_buffer_free(urb->dev, urb->transfer_buffer_length,
			urb->transfer_buffer, urb->transfer_dma);
	up(&dev->limit_sem);
}*/

static ssize_t retina_write(struct file *file, const char *user_buffer, size_t count, loff_t *ppos)
{
	struct usb_retina *dev;
	int retval = 0;
	struct urb *urb = NULL;
	char *buf = NULL;
	size_t writesize;
	u8 request = user_buffer[0];
	u16 value  = (user_buffer[1]&0x00ff)+((user_buffer[2]<<8)&0xff00);
	u16 index  = (user_buffer[3]&0x00ff)+((user_buffer[4]<<8)&0xff00);
	user_buffer= user_buffer+5;
	writesize = min(count-5, (size_t)MAX_TRANSFER);

	dev = (struct usb_retina *)file->private_data;
	/*dev_info(&dev->interface->dev,"Request,Value,Index,Count=0x%x,%d,%d,%d",request,value,index,writesize);*/

	/*create a urb, and a buffer for it, and copy the data to the urb */
	urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!urb) {
		retval = -ENOMEM;
		goto error;
	}

	if (writesize > 0)
	{
		buf = usb_buffer_alloc(dev->udev, writesize, GFP_KERNEL, &urb->transfer_dma);
		if (!buf) {
			retval = -ENOMEM;
			goto error;
		}

		if (copy_from_user(buf, user_buffer, writesize)) {
			retval = -EFAULT;
			usb_buffer_free(dev->udev, writesize, buf, urb->transfer_dma);
			goto error;
		}
	}
	mutex_lock(&dev->io_mutex);
	/* this lock makes sure we don't submit URBs to gone devices

	if (!dev->interface) {		 disconnect() was called
		mutex_unlock(&dev->io_mutex);
		retval = -ENODEV;
		goto error;
	}*/

	/* initialize the urb properly
	usb_fill_bulk_urb(urb, dev->udev,
			  usb_sndbulkpipe(dev->udev, dev->bulk_out_endpointAddr),
			  buf, writesize, retina_write_bulk_callback, dev);
	urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
	usb_anchor_urb(urb, &dev->submitted);

	 send the data out the bulk port
	retval = usb_submit_urb(urb, GFP_KERNEL);*/

	/*use the write call for submitting vendor requests. java has no ioctl.*/
	/*retval = vendorRequest(dev, request, value, index, buf, writesize);*/
	retval = vendorRequest(dev, request, value, index, buf, writesize);

	usb_buffer_free(dev->udev, writesize, buf, urb->transfer_dma);

	/*dev_info(&dev->interface->dev,"retina_write(ioctl): VENDOR_REQUEST 0x%x returned %d.\n(Val,Ind,Cnt=0x%x,0x%x,0x%x)",request,
	    retval,value,index,writesize);*/

	if (retval) {
		dev_err(&dev->interface->dev,"%s - failed vendor request, error %d", __FUNCTION__, retval);
		goto exit;
	}
	mutex_unlock(&dev->io_mutex);
	/* release our reference to this urb, the USB core will eventually free it entirely */
	usb_free_urb(urb);
	return writesize+5;

error:

exit:
	mutex_unlock(&dev->io_mutex);
	return retval;
}

static const struct file_operations retina_fops = {
	.owner =	THIS_MODULE,
	.open =		retina_open,
	.read =		retina_read,
	.write =	retina_write,
	.release =	retina_release,
	.flush =	retina_flush,
};

/*
 * usb class driver info in order to get a minor number from the usb core,
 * and to have the device registered with the driver core
 */
static struct usb_class_driver retina_class = {
	.name =		"retina%d",
	.fops =		&retina_fops,
	.minor_base =	USB_RETINA_MINOR_BASE,
};


static void address_event_callback(struct urb *urb)
{
	struct usb_retina *dev = urb->context;
	int length = urb->actual_length;
	int retval;

	dev->event_counter += length/4;
	if(urb->status)
	{
		dev_err(&dev->interface->dev,"address_event_callback: urb send error: 0x%x",urb->status);
		return;
	}

	if(dev->event_counter<0)
	{

		retval = usb_submit_urb (dev->urb, GFP_KERNEL);
		if (retval)
		{
		    dev_err(&dev->interface->dev,"testing: error getting ae data: %d",retval);
		}
	}
	else
	{	/*dev_info(&dev->interface->dev,"testing: %d address events read.", dev->event_counter);
		dev_info(&dev->interface->dev,"testing: event#1 = {0x%x,0x%x,0x%x,0x%x}",dev->bulk_in_buffer[0],dev->bulk_in_buffer[1],dev->bulk_in_buffer[2],dev->bulk_in_buffer[3]);*/
	}
}

static int retina_probe(struct usb_interface *interface, const struct usb_device_id *id)
{
	struct usb_retina *dev;
	struct usb_host_interface *iface_desc;
	struct usb_endpoint_descriptor *endpoint;
	size_t buffer_size;
	int i;
	int retval = -ENOMEM;
	struct urb *urb;

	/* allocate memory for our device state and initialize it */
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		dev_err(&interface->dev,"Out of memory");
		goto error;
	}
	kref_init(&dev->kref);
	sema_init(&dev->limit_sem, WRITES_IN_FLIGHT);
	mutex_init(&dev->io_mutex);
	spin_lock_init(&dev->err_lock);
	init_usb_anchor(&dev->submitted);

	dev->udev = usb_get_dev(interface_to_usbdev(interface));
	dev->interface = interface;
	/* set up the endpoint information */
	/* use only the first bulk-in endpoint */
	iface_desc = interface->cur_altsetting;
	for (i = 0; i < iface_desc->desc.bNumEndpoints; ++i) {
		endpoint = &iface_desc->endpoint[i].desc;
		/*dev_info(&interface->dev,"usb endpoint[%d] found. size=%d, addr=0x%x",i,endpoint->wMaxPacketSize,endpoint->bEndpointAddress);*/
		if (!dev->bulk_in_endpointAddr &&
			    usb_endpoint_is_bulk_in(endpoint)
			    && (endpoint->wMaxPacketSize==512)) {
			if (!dev->bulk_in_endpointAddr &&
			    usb_endpoint_is_bulk_in(endpoint)) {
				/* we found a bulk in endpoint */
				buffer_size = le16_to_cpu(endpoint->wMaxPacketSize);
				dev->bulk_in_size = buffer_size;
				dev->bulk_in_endpointAddr = endpoint->bEndpointAddress;
				dev->bulk_in_buffer = kmalloc(buffer_size, GFP_KERNEL);
				if (!dev->bulk_in_buffer) {
					dev_err(&interface->dev,"Could not allocate bulk_in_buffer");
					goto error;
				}
			}
		}
	}

	if (!(dev->bulk_in_endpointAddr)) {
		dev_err(&interface->dev,"Could not find bulk-in endpoint\n");
		goto error;
	}
	dev->event_counter = 0;
	/* save our data pointer in this interface device */
	usb_set_intfdata(interface, dev);

	/* we can register the device now, as it is ready */
	retval = usb_register_dev(interface, &retina_class);
	if (retval) {
		/* something prevented us from registering this driver */
		dev_err(&interface->dev,"Not able to get a minor for this device.");
		usb_set_intfdata(interface, NULL);
		goto error;
	}

	/* let the user know what node this device is now attached to */
	dev_info(&interface->dev,"retina now attached to /dev/retina0\n");

	vendorRequest(dev, VENDOR_REQUEST_START_TRANSFER, 0, 0, NULL, 0);
	/*dev_info(&dev->interface->dev,"VENDOR_REQUEST_START_TRANSFER returned %d",retval);*/
	urb = usb_alloc_urb (0, GFP_KERNEL);
	if (!urb)
	    goto error;
	usb_fill_bulk_urb (urb, dev->udev,
	    usb_rcvbulkpipe (dev->udev, dev->bulk_in_endpointAddr),
	    dev->bulk_in_buffer, dev->bulk_in_size,
	    address_event_callback, dev);
	dev->urb = urb;
	retval = usb_submit_urb (dev->urb, GFP_KERNEL);
	if (retval)
	{
	    dev_err(&interface->dev, "error getting ae data: %d",retval);
	    goto error;
	}
	return 0;

error:
	if (dev)
		/* this frees allocated memory */
		kref_put(&dev->kref, retina_delete);
	return retval;
}

static void retina_disconnect(struct usb_interface *interface)
{
	struct usb_retina *dev;

	dev = usb_get_intfdata(interface);
	usb_set_intfdata(interface, NULL);

	/* give back our minor */
	usb_deregister_dev(interface, &retina_class);

	/* prevent more I/O from starting */
	mutex_lock(&dev->io_mutex);
	dev->interface = NULL;
	mutex_unlock(&dev->io_mutex);

	usb_kill_anchored_urbs(&dev->submitted);

	/* decrement our usage count */
	kref_put(&dev->kref, retina_delete);

	dev_info(&interface->dev,"/dev/retina0 now disconnected\n");
}

static void retina_draw_down(struct usb_retina *dev)
{
	int time;

	time = usb_wait_anchor_empty_timeout(&dev->submitted, 1000);
	if (!time)
		usb_kill_anchored_urbs(&dev->submitted);
}

static int retina_suspend(struct usb_interface *intf, pm_message_t message)
{
	struct usb_retina *dev = usb_get_intfdata(intf);

	dev_info(&intf->dev,"preparing retina module to suspend");
	if (!dev)
		return 0;
	retina_draw_down(dev);
	return 0;
}

static int retina_resume (struct usb_interface *intf)
{
	dev_info(&intf->dev,"resuming retina module");
	return 0;
}

static int retina_pre_reset(struct usb_interface *intf)
{
	struct usb_retina *dev = usb_get_intfdata(intf);

	dev_info(&intf->dev,"pre_reset of retina module");
	mutex_lock(&dev->io_mutex);
	retina_draw_down(dev);

	return 0;
}

static int retina_post_reset(struct usb_interface *intf)
{
	struct usb_retina *dev = usb_get_intfdata(intf);

	dev_info(&intf->dev,"post_reset retina module");
	/* we are sure no URBs are active - no locking needed */
	dev->errors = -EPIPE;
	mutex_unlock(&dev->io_mutex);

	return 0;
}


static struct usb_driver retina_driver = {
	.name =		"retina",
	.probe =	retina_probe,
	.disconnect =	retina_disconnect,
	.suspend =	retina_suspend,
	.resume =	retina_resume,
	.pre_reset =	retina_pre_reset,
	.post_reset =	retina_post_reset,
	.id_table =	retina_table,
	.supports_autosuspend = 1,
};

static int __init usblab_init(void)
{
	int result;

	/*dev_info(&retina_driver.interface->dev,"retina module initialisation");*/
	/* register this driver with the USB subsystem */
	result = usb_register(&retina_driver);
	if (result)
		err("usb_register failed. Error number %d", result);

	return result;
}

static void __exit usblab_exit(void)
{
	/* deregister this driver with the USB subsystem */
	usb_deregister(&retina_driver);
}



module_init(usblab_init);
module_exit(usblab_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("USB lab module");


/* lsusb -v : dvs128 usb info
Bus 007 Device 007: ID 152a:8400
Device Descriptor:
  bLength                18
  bDescriptorType         1
  bcdUSB               2.00
  bDeviceClass            0 (Defined at Interface level)
  bDeviceSubClass         0
  bDeviceProtocol         0
  bMaxPacketSize0        64
  idVendor           0x152a
  idProduct          0x8400
  bcdDevice            0.00
  iManufacturer           1
  iProduct                2
  iSerial                 3
  bNumConfigurations      1
  Configuration Descriptor:
    bLength                 9
    bDescriptorType         2
    wTotalLength           39
    bNumInterfaces          1
    bConfigurationValue     1
    iConfiguration          0
    bmAttributes         0x80
      (Bus Powered)
    MaxPower              300mA
    Interface Descriptor:
      bLength                 9
      bDescriptorType         4
      bInterfaceNumber        0
      bAlternateSetting       0
      bNumEndpoints           3
      bInterfaceClass       255 Vendor Specific Class
      bInterfaceSubClass      0
      bInterfaceProtocol      0
      iInterface              0
      Endpoint Descriptor:
        bLength                 7
        bDescriptorType         5
        bEndpointAddress     0x81  EP 1 IN
        bmAttributes            2
          Transfer Type            Bulk
          Synch Type               None
          Usage Type               Data
        wMaxPacketSize     0x0040  1x 64 bytes
        bInterval               0
      Endpoint Descriptor:
        bLength                 7
        bDescriptorType         5
        bEndpointAddress     0x02  EP 2 OUT
        bmAttributes            2
          Transfer Type            Bulk
          Synch Type               None
          Usage Type               Data
        wMaxPacketSize     0x0200  1x 512 bytes
        bInterval               0
      Endpoint Descriptor:
        bLength                 7
        bDescriptorType         5
        bEndpointAddress     0x86  EP 6 IN
        bmAttributes            2
          Transfer Type            Bulk
          Synch Type               None
          Usage Type               Data
        wMaxPacketSize     0x0200  1x 512 bytes
        bInterval               0
can't get device qualifier: Operation not permitted
can't get debug descriptor: Operation not permitted
cannot read device status, Operation not permitted (1)
*/
