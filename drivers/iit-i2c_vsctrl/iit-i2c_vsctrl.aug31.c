#include <asm/uaccess.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/i2c.h>
//#include <linux/init.h>
//#include <linux/kdev_t.h>
#include <linux/kern_levels.h>
//#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/printk.h>
//#include <linux/slab.h>
#include <linux/stat.h>
//#include <linux/sysdev.h>

//#include <linux/jiffies.h>
//#include <linux/sched.h>

#include "iit-i2c_vsctrl.h"
#include "iit-i2c_vsctrl_ioctl.h"


#define DRV_NAME		   "iit-i2c_vsctrl"
#define MSG_PREFIX         "IIT-VSCtrl: "

#define MAX_CONV_MS  150
#define DATA_STORAGE_U32 32

struct icub_vsctrl_data {
	struct i2c_client *client;
	struct miscdevice misc_dev;
	struct mutex mutex;
    int    open_count;
    
    uint32_t data_write[DATA_STORAGE_U32];
    uint32_t data_read[DATA_STORAGE_U32];
    
    signed char chip_type;
    char roi_notbg;
    char crc_check_en;
    
    //uint8_t cfg_ack_rel_delay;
    //uint32_t prescaler_value;
    // etc.
};


static int icub_vsctrl_readwrite(struct i2c_client *client,
                                 u16 wr_len, uint8_t *wr_buf,
                                 u16 rd_len, uint8_t *rd_buf)
{
    struct i2c_msg wrmsg[2];
    int i = 0;
    int ret;

    if (wr_len) {
        wrmsg[i].addr  = client->addr;
        wrmsg[i].flags = 0;
        wrmsg[i].len = wr_len;
        wrmsg[i].buf = wr_buf;
        i++;
    }
    if (rd_len) {
        wrmsg[i].addr  = client->addr;
        wrmsg[i].flags = I2C_M_RD;
        wrmsg[i].len = rd_len;
        wrmsg[i].buf = rd_buf;
        i++;
    }

    ret = i2c_transfer(client->adapter, wrmsg, i);
    if (ret < 0)
        return ret;
    if (ret != i)
        return -EIO;

    return 0;
}

static int icub_vsctrl_write_byte(struct icub_vsctrl_data *pdata,
                                  uint8_t addr, uint8_t value)
{
    uint8_t wrbuf[2] = {0};

    wrbuf[0] = addr & 0x3F;
    wrbuf[1] = value;

    return icub_vsctrl_readwrite(pdata->client, 2, wrbuf, 0, NULL);
}

static int icub_vsctrl_fifowrite_byte(struct icub_vsctrl_data *pdata,
                                      uint8_t addr, uint8_t value[], u16 len)
{
    //uint8_t wrbuf[len+1]={0};
    uint8_t wrbuf[len+1];
    int i = 0;
    int j = 0;

    wrbuf[j++] = (addr&0x3F)|0x00;
    while (i < len)
        wrbuf[j++] = value[i++];

    return icub_vsctrl_readwrite(pdata->client, j, wrbuf, 0, NULL);
}

static int icub_vsctrl_blockwrite_byte(struct icub_vsctrl_data *pdata,
                                       uint8_t addr, uint8_t value[], u16 len)
{
    //uint8_t wrbuf[len+1]={0};
    uint8_t wrbuf[len+1];
    int i = 0;
    int j = 0;

    wrbuf[j++] = (addr&0x3F)|0x80;
    while (i < len)
        wrbuf[j++] = value[i++];

    return icub_vsctrl_readwrite(pdata->client, j, wrbuf, 0, NULL);
}

static int icub_vsctrl_write_word(struct icub_vsctrl_data *pdata,
                                  uint8_t addr, uint32_t value)
{
    uint8_t wrbuf[5] = {0};

    wrbuf[0] = (addr&0x3F)|0x40;
    wrbuf[1] = (value>> 0)&0xFF;
    wrbuf[2] = (value>> 8)&0xFF;
    wrbuf[3] = (value>>16)&0xFF;
    wrbuf[4] = (value>>24)&0xFF;

    return icub_vsctrl_readwrite(pdata->client, 5, wrbuf, 0, NULL);
}

static int icub_vsctrl_fifowrite_word(struct icub_vsctrl_data *pdata,
                                      uint8_t addr, uint32_t value[], u16 len)
{
    //uint8_t wrbuf[4*len+1]={0};
    uint8_t wrbuf[4*len+1];
    int i = 0;
    int j = 0;

    wrbuf[j++] = (addr&0x3F)|0x40;
    while (i < len) {
        wrbuf[j++] = (value[i]>> 0)&0xFF;
        wrbuf[j++] = (value[i]>> 8)&0xFF;
        wrbuf[j++] = (value[i]>>16)&0xFF;
        wrbuf[j++] = (value[i]>>24)&0xFF;
        i++;
    }

    return icub_vsctrl_readwrite(pdata->client, j, wrbuf, 0, NULL);
}

static int icub_vsctrl_blockwrite_word(struct icub_vsctrl_data *pdata,
                                       uint8_t addr, uint32_t value[], u16 len)
{
    //uint8_t wrbuf[4*len+1]={0};
    uint8_t wrbuf[4*len+1];
    int i = 0;
    int j = 0;

    wrbuf[j++] = (addr&0x3F)|0xC0;
    while (i < len) {
        wrbuf[j++] = (value[i]>> 0)&0xFF;
        wrbuf[j++] = (value[i]>> 8)&0xFF;
        wrbuf[j++] = (value[i]>>16)&0xFF;
        wrbuf[j++] = (value[i]>>24)&0xFF;
        i++;
    }

    return icub_vsctrl_readwrite(pdata->client, j, wrbuf, 0, NULL);
}




static int icub_vsctrl_read_byte(struct icub_vsctrl_data *pdata,
                                 uint8_t addr, uint8_t *value)
{
    uint8_t wrbuf[1], rdbuf[1] = {0};
    int error;

    wrbuf[0] = addr & 0x3F;

    error = icub_vsctrl_readwrite(pdata->client, 1, wrbuf, 1, rdbuf);
    if (error)
        return error;

    *value = rdbuf[0];
    return 0;
}

static int icub_vsctrl_fiforead_byte(struct icub_vsctrl_data *pdata,
                                     uint8_t addr, uint8_t value[], u16 len)
{
    uint8_t wrbuf[1] = {0};
    //uint8_t rdbuf[len]={0};
    uint8_t rdbuf[len];
    int error;
    int i = 0;
    int j = 0;

    wrbuf[0] = (addr&0x3F)|0x00;
    
    error = icub_vsctrl_readwrite(pdata->client, 1, wrbuf, len, rdbuf);
    if (error)
        return error;
        
    while (i < len)
        value[j++] = rdbuf[i++];

    return 0;
}

static int icub_vsctrl_blockread_byte(struct icub_vsctrl_data *pdata,
                                      uint8_t addr, uint8_t value[], u16 len)
{
    uint8_t wrbuf[1] = {0};
    //uint8_t rdbuf[len] = {0};
    uint8_t rdbuf[len];
    int error;
    int i = 0;
    int j = 0;

    wrbuf[0] = (addr&0x3F)|0x80;
    
    error = icub_vsctrl_readwrite(pdata->client, 1, wrbuf, len, rdbuf);
    if (error)
        return error;
        
    while (i < len)
        value[j++] = rdbuf[i++];

    return 0;
}

static int icub_vsctrl_read_word(struct icub_vsctrl_data *pdata,
                                 uint8_t addr, uint32_t *value)
{
    uint8_t wrbuf[1], rdbuf[4] = {0};
    int error;

    wrbuf[0] = (addr&0x3F)|0x40;

    error = icub_vsctrl_readwrite(pdata->client, 1, wrbuf, 4, rdbuf);
    if (error)
        return error;

    *value = (rdbuf[0]|rdbuf[1]<<8|rdbuf[2]<<16|rdbuf[3]<<24);
    return 0;
}

static int icub_vsctrl_fiforead_word(struct icub_vsctrl_data *pdata,
                                     uint8_t addr, uint32_t value[], u16 len)
{
    uint8_t wrbuf[1] = {0};
    //uint8_t rdbuf[4*len] = {0};
    uint8_t rdbuf[4*len];
    int error;
    int i = 0;
    int j = 0;

    wrbuf[0] = (addr&0x3F)|0x40;
    
    error = icub_vsctrl_readwrite(pdata->client, 1, wrbuf, 4*len, rdbuf);
    if (error)
        return error;
        
    while (i < len) {
        value[i++] = rdbuf[j]|rdbuf[j+1]<<8|rdbuf[j+2]<<16|rdbuf[j+3]<<24;
        j+=4;
    }

    return 0;
}

static int icub_vsctrl_blockread_word(struct icub_vsctrl_data *pdata,
                                      uint8_t addr, uint32_t value[], u16 len)
{
    uint8_t wrbuf[1] = {0};
    //uint8_t rdbuf[4*len] = {0};
    uint8_t rdbuf[4*len];
    int error;
    int i = 0;
    int j = 0;

    wrbuf[0] = (addr&0x3F)|0xC0;
    
    error = icub_vsctrl_readwrite(pdata->client, 1, wrbuf, 4*len, rdbuf);
    if (error)
        return error;
        
    while (i < len) {
        value[i++] = rdbuf[j]|rdbuf[j+1]<<8|rdbuf[j+2]<<16|rdbuf[j+3]<<24;
        j += 4;
    }

    return 0;
}


static uint32_t icub_vsctrl_crc_check(uint32_t *data, int len, uint32_t poly) {
    return 0;
}



static inline struct icub_vsctrl_data *get_pdata_from_file(struct file *fp)
{
	struct miscdevice *dev = fp->private_data;
	return container_of(dev, struct icub_vsctrl_data, misc_dev);
}


static int icub_vsctrl_init(struct icub_vsctrl_data *pdata, int chip_type) {
    uint8_t tmp_byte;
    
	printk(KERN_DEBUG "%s()\n", __func__);
    
	//mutex_lock(&pdata->mutex);
    if (chip_type == C_CHIP_AUTO) {
        if (icub_vsctrl_read_byte(pdata, C_FPGACFG_ADDR, &tmp_byte)) {
            printk(KERN_ERR MSG_PREFIX "Read from I2C failed\n");
            mutex_unlock(&pdata->mutex);
            return -EFAULT;
        }
        pdata->chip_type = chip_type = (tmp_byte >> 5) & 0x07;
    }
    
    //printk(KERN_DEBUG "chip_type = %d\n", chip_type);  goto lbl_init_exit;
    if (chip_type == C_CHIP_DVS) {
        icub_vsctrl_write_byte(pdata, C_BGCTRL1_ADDR, C_BG_PWRDWN_MSK);      // Put the Bias generator in PowerDown mode
        if (pdata->chip_type != chip_type) {
            printk(KERN_INFO MSG_PREFIX "Forcing chip interface to behave as DVS type\n");
            icub_vsctrl_write_byte(pdata, C_SRCCNFG_ADDR, 0x10);  // AER Req and Ack active low
            icub_vsctrl_write_byte(pdata, C_BGCNFG0_ADDR, 0x08);  // BG clock and latch active low
        } else {
            icub_vsctrl_write_byte(pdata, C_BGCNFG0_ADDR, 0x00);  // BG clock and latch active low
        }
        
        icub_vsctrl_write_byte(pdata, C_SRCTIM1_ADDR, 1);      // Set the AER Acknowledge Set Delay  TBD
        icub_vsctrl_write_byte(pdata, C_SRCTIM2_ADDR, 5);      // Set the AER Data Sample Delay  TBD
        icub_vsctrl_write_byte(pdata, C_SRCTIM3_ADDR, 6);      // Set the AER Acknowledge Release Delay TBD
        
        icub_vsctrl_write_word(pdata, C_BG_PRESC_ADDR32, 125000);  // Set the BG Prescaler Value  TBD
        icub_vsctrl_write_byte(pdata, C_BGTIM0_ADDR, 100);         // Set the BG Latch Active Time  TBD
        icub_vsctrl_write_byte(pdata, C_BGTIM1_ADDR, 100);         // Set the BG Latch Setup Time  TBD
        icub_vsctrl_write_byte(pdata, C_BGTIM2_ADDR, 1);           // Set the BG Clock Active Time  TBD
        icub_vsctrl_write_byte(pdata, C_BGTIM3_ADDR, 1);           // Set the BG Setup/Hold time  TBD
        
    } else if (chip_type == C_CHIP_ATIS) {
        icub_vsctrl_write_byte(pdata, C_BGCTRL1_ADDR, C_BG_PWRDWN_MSK);      // Put the Bias generator in PowerDown mode
        if (pdata->chip_type != chip_type) {
            printk(KERN_INFO MSG_PREFIX "Forcing chip interface to behave as ATIS type\n");
            icub_vsctrl_write_byte(pdata, C_SRCCNFG_ADDR, 0x15);  // AER Req active high, Ack active low
            icub_vsctrl_write_byte(pdata, C_BGCNFG0_ADDR, 0x39);  // BG clock and latch active high
        } else {
            icub_vsctrl_write_byte(pdata, C_BGCNFG0_ADDR, 0x30);  // BG clock and latch active high
        }

        icub_vsctrl_write_byte(pdata, C_SRCTIM1_ADDR, 0);      // Set the AER Acknowledge Set Delay  TBD
        icub_vsctrl_write_byte(pdata, C_SRCTIM2_ADDR, 0);      // Set the AER Data Sample Delay  TBD
        icub_vsctrl_write_byte(pdata, C_SRCTIM3_ADDR, 0);      // Set the AER Acknowledge Release Delay TBD
        
        icub_vsctrl_write_word(pdata, C_BG_PRESC_ADDR32, 0);   // Set the BG Prescaler Value  TBD
        icub_vsctrl_write_byte(pdata, C_BGTIM0_ADDR, 0);       // Set the BG Latch Active Time  TBD
        icub_vsctrl_write_byte(pdata, C_BGTIM1_ADDR, 0);       // Set the BG Latch Setup Time  TBD
        icub_vsctrl_write_byte(pdata, C_BGTIM2_ADDR, 0);       // Set the BG Clock Active Time  TBD
        icub_vsctrl_write_byte(pdata, C_BGTIM3_ADDR, 0);       // Set the BG Setup/Hold time  TBD

    } else {
        printk(KERN_ERR MSG_PREFIX "Chip type %d not managed\n", chip_type);
        mutex_unlock(&pdata->mutex);
        return -EINVAL;
    }
    
    pdata->chip_type = chip_type;
    pdata->roi_notbg = 0;
    //pdata->crc_check_en = 1;    // TBD: CRC check function not yet implemented
    pdata->crc_check_en = 0;
    
    icub_vsctrl_write_word(pdata, C_PAER_CFG_ADDR32, 0x00000000);   // Configure the PAER communication between VSCtrl and HPU
    
    icub_vsctrl_write_byte(pdata, C_DSTCTRL_ADDR, 0x01);            // Enable the PAER communication between VSCtrl and HPU
    icub_vsctrl_write_byte(pdata, C_SRCCTRL_ADDR, 0xC0);            // Flush the FIFOs and set the IgnoreFifoFull flag
    icub_vsctrl_write_byte(pdata, C_SRCCTRL_ADDR, 0x8A);            // Enable the AER interfaces from the Vision Sensor
    icub_vsctrl_write_word(pdata, C_STATUS_ADDR32, 0xFFFFFFFF);

lbl_init_exit:
	//mutex_unlock(&pdata->mutex);
    
	return 0;
}

static int icub_vsctrl_misc_open (struct inode *ip, struct file *fp) {
	struct icub_vsctrl_data *pdata = get_pdata_from_file(fp);
    uint8_t tmp_byte;

	printk(KERN_DEBUG "%s()\n", __func__);
    
    pdata->open_count++;
    if (pdata->open_count > 1) {
        return 0;
    }

    icub_vsctrl_read_byte(pdata, C_BGCTRL1_ADDR, &tmp_byte);
    
    if (!pdata->roi_notbg)
        tmp_byte &= ~C_BG_BIASROI_MSK;
    else
        tmp_byte |= C_BG_BIASROI_MSK;

    if (pdata->crc_check_en)
        tmp_byte |= (C_BG_CRCCLR_MSK | C_BG_CRCEN_MSK);
    else
        tmp_byte &= ~C_BG_CRCEN_MSK;
        
    icub_vsctrl_write_byte(pdata, C_BGCTRL1_ADDR, tmp_byte);

	return 0;
}

static int icub_vsctrl_misc_release (struct inode *ip, struct file *fp) {
	struct icub_vsctrl_data *pdata = get_pdata_from_file(fp);

	printk(KERN_DEBUG "%s()\n", __func__);

    pdata->open_count--;
    if (pdata->open_count > 0) {
        return 0;
    }
    
	return 0;
}


static ssize_t icub_vsctrl_misc_write (struct file *fp, const char __user *buf, size_t len, loff_t *ppos) {
	struct icub_vsctrl_data *pdata = get_pdata_from_file(fp);
    int length_uint;
    int length_trunc;
    uint32_t crc32;
    int i, j;

	printk(KERN_DEBUG "%s()\n", __func__);

    
    length_trunc = (len>(DATA_STORAGE_U32<<2)) ? DATA_STORAGE_U32<<2 : len;
    length_uint = length_trunc>>2;

    mutex_lock(&pdata->mutex);
    if (copy_from_user(pdata->data_write, (unsigned int *)buf, length_uint*sizeof(unsigned int))) {
        printk(KERN_ERR MSG_PREFIX "Copy from user space failed\n");
        mutex_unlock(&pdata->mutex);
        return -EFAULT;
    }
    crc32 = icub_vsctrl_crc_check(pdata->data_write, length_uint, C_BG_CRC_POLY);
    icub_vsctrl_write_word(pdata, C_BG_CRC_ADDR32, crc32);
    if (pdata->chip_type == C_CHIP_DVS) {
#if 0
        icub_vsctrl_fifowrite_word(pdata, C_BG_DATA_ADDR32, pdata->data_write, length_uint);
#else
        for (i=0; i<length_uint; i++) {
            j = 0;
            while (icub_vsctrl_write_word(pdata, C_BG_DATA_ADDR32, pdata->data_write[i])) {
                if (j++ > 1000) {
                    printk(KERN_ERR MSG_PREFIX "Failed to send bias data\n");
                    return i;
                }
            }
        }
#endif

#if 0            
        icub_vsctrl_write_word(pdata, C_BG_DATA_ADDR32, 0x40FFFFFF);
#else
        j = 0;
        while (icub_vsctrl_write_word(pdata, C_BG_DATA_ADDR32, 0x40FFFFFF)) {
            if (j++ > 1000) {
                printk(KERN_ERR MSG_PREFIX "Failed to send last bias data\n");
                return i;
            }
        }
#endif
    } else if (pdata->chip_type == C_CHIP_ATIS) {
        icub_vsctrl_write_byte(pdata, C_BGCTRL0_ADDR, 0x44);
        icub_vsctrl_fifowrite_word(pdata, C_BG_DATA_ADDR32, pdata->data_write, length_uint);
        icub_vsctrl_write_byte(pdata, C_BGCTRL0_ADDR, 0x80);
        icub_vsctrl_write_word(pdata, C_BG_DATA_ADDR32, 0x00000000);
    }
    
    mutex_unlock(&pdata->mutex);

	return length_uint << 2;
}


static ssize_t icub_vsctrl_misc_read (struct file *fp, char __user *buf, size_t len, loff_t *ppos) {
	struct icub_vsctrl_data *pdata = get_pdata_from_file(fp);
    int length_uint;
    int length_trunc;

	printk(KERN_DEBUG "%s()\n", __func__);

    length_trunc = (len>(DATA_STORAGE_U32<<2)) ? DATA_STORAGE_U32<<2 : len;
    length_uint = length_trunc>>2;

    mutex_lock(&pdata->mutex);
    if (icub_vsctrl_fiforead_word(pdata, C_BG_DATA_ADDR32, pdata->data_read, length_uint)) {
        printk(KERN_ERR MSG_PREFIX "Fifo write failed\n");
        mutex_unlock(&pdata->mutex);
        return -EFAULT;
    }
    if (copy_to_user((unsigned int *)buf, pdata->data_read, length_uint*sizeof(unsigned int))) {
        printk(KERN_ERR MSG_PREFIX "Copy to user space failed\n");
        mutex_unlock(&pdata->mutex);
        return -EFAULT;
    }
    mutex_unlock(&pdata->mutex);
    
	return length_uint << 2;
}

static long icub_vsctrl_misc_ioctl (struct file *fp, unsigned int cmd, unsigned long arg) {
    
    uint8_t  tmp_byte;
    uint32_t tmp_word;
    uint8_t  buf[4];
    icub_vsctrl_ioctl_arg_t ioctl_arg;
	struct icub_vsctrl_data *pdata = get_pdata_from_file(fp);
    
	printk(KERN_DEBUG "%s()\n", __func__);
    
    mutex_lock(&pdata->mutex);
    
    switch (cmd) {
        case IOC_GEN_REG_ACCESS:
            if (copy_from_user(&ioctl_arg, (icub_vsctrl_ioctl_arg_t *)arg, sizeof(icub_vsctrl_ioctl_arg_t)))
                goto cfuser_err;

            if (ioctl_arg.regs.rw == 0) {
                if (icub_vsctrl_read_word(pdata, ioctl_arg.regs.addr, &(ioctl_arg.regs.data)))
                    goto read_err;
                if (copy_to_user((icub_vsctrl_ioctl_arg_t *)arg, &ioctl_arg, sizeof(icub_vsctrl_ioctl_arg_t)))
                    goto ctuser_err;
            }
            else {
                if (icub_vsctrl_write_word(pdata, ioctl_arg.regs.addr, ioctl_arg.regs.data))
                    goto write_err;
            }
            break;

        case IOC_GET_FPGAREL:
            if (icub_vsctrl_read_byte(pdata, C_FPGAREL_ADDR, &tmp_byte))
                goto read_err;
            if (copy_to_user((void __user *)arg, &tmp_byte, sizeof(uint8_t)))
                goto ctuser_err;
            break;
            
        case IOC_GET_INFO:
            if (icub_vsctrl_read_byte(pdata, C_FPGACFG_ADDR, &tmp_byte))
                goto read_err;
            if (copy_to_user((void __user *)arg, &tmp_byte, sizeof(uint8_t)))
                goto ctuser_err;
            break;
            
        case IOC_GET_STATUS:
            if (icub_vsctrl_read_word(pdata, C_STATUS_ADDR32, &tmp_word))
                goto read_err;
            if (copy_to_user((void __user *)arg, &tmp_word, sizeof(uint32_t)))
                goto ctuser_err;
            break;

        case IOC_INIT_DEV:
            icub_vsctrl_init(pdata, (int)arg);
            break;
            
        case IOC_RESET_ARRAY:
            if (icub_vsctrl_read_byte(pdata, C_OUTSDBUS0_ADDR, &tmp_byte))
                goto read_err;
                
            if (arg)
                tmp_byte |= C_ARRAY_RST_MSK;
            else
                tmp_byte &= ~C_ARRAY_RST_MSK;

            if (icub_vsctrl_write_byte(pdata, C_OUTSDBUS0_ADDR, tmp_byte))
                goto write_err;
            break;
            
        case IOC_SET_PWRDWN:
            if (icub_vsctrl_read_byte(pdata, C_BGCTRL1_ADDR, &tmp_byte))
                goto read_err;
                
            if (arg)
                tmp_byte |= C_BG_PWRDWN_MSK;
            else
                tmp_byte &= ~C_BG_PWRDWN_MSK;

            if (icub_vsctrl_write_byte(pdata, C_BGCTRL1_ADDR, tmp_byte))
                goto write_err;
            break;
            
        case IOC_SET_BIASGEN:
            if (icub_vsctrl_read_byte(pdata, C_BGCTRL1_ADDR, &tmp_byte))
                goto read_err;
            tmp_byte &= ~C_BG_BIASROI_MSK;
            if (icub_vsctrl_write_byte(pdata, C_BGCTRL1_ADDR, tmp_byte))
                goto write_err;
            pdata->roi_notbg = 0;
            //pdata->crc_check_en = 1;    // TBD: CRC check function not yet implemented
            pdata->crc_check_en = 0;
            break;
            
        case IOC_SET_ROIGEN:
            if (icub_vsctrl_read_byte(pdata, C_BGCTRL1_ADDR, &tmp_byte))
                goto read_err;
            tmp_byte |= C_BG_BIASROI_MSK;
            if (icub_vsctrl_write_byte(pdata, C_BGCTRL1_ADDR, tmp_byte))
                goto write_err;
            pdata->roi_notbg = 1;
            pdata->crc_check_en = 0;
            break;

        case IOC_SET_GPO:
            if (icub_vsctrl_write_word(pdata, C_OUT_SDBUS_ADDR32, arg))
                goto write_err;
            break;
        
        case IOC_GET_GPI:
            if (icub_vsctrl_read_word(pdata, C_IN_SDBUS_ADDR32, &tmp_word))
                goto read_err;
            if (copy_to_user((void __user *)arg, &tmp_word, sizeof(uint32_t)))
                goto ctuser_err;
            break;
        
        case IOC_SET_AER_TIMINGS:
            if (copy_from_user(&ioctl_arg, (icub_vsctrl_ioctl_arg_t *)arg, sizeof(icub_vsctrl_ioctl_arg_t)))
                goto cfuser_err;
            buf[0] = ioctl_arg.aer_timings.cfg_ack_set_delay;
            buf[1] = ioctl_arg.aer_timings.cfg_sample_delay;
            buf[2] = ioctl_arg.aer_timings.cfg_ack_rel_delay;
            if (icub_vsctrl_blockwrite_byte(pdata, C_SRCTIM1_ADDR, buf, 3))
                goto write_err;
            break;

        case IOC_SET_BG_TIMINGS:
            if (copy_from_user(&ioctl_arg, (icub_vsctrl_ioctl_arg_t *)arg, sizeof(icub_vsctrl_ioctl_arg_t)))
                goto cfuser_err;
            if (icub_vsctrl_write_word(pdata, C_BGPRESC0_ADDR, ioctl_arg.bg_timings.prescaler_value))
                goto write_err;
            buf[0] = ioctl_arg.bg_timings.latch_active_time;
            buf[1] = ioctl_arg.bg_timings.latch_setup_time;
            buf[2] = ioctl_arg.bg_timings.clock_active_time;
            buf[3] = ioctl_arg.bg_timings.setup_hold_time;
            if (icub_vsctrl_blockwrite_byte(pdata, C_BGTIM0_ADDR, buf, 4))
                goto write_err;
            break;

        case IOC_GET_AER_TIMINGS:
            if (icub_vsctrl_blockread_byte(pdata, C_SRCTIM1_ADDR, buf, 3))
                goto read_err;
            ioctl_arg.aer_timings.cfg_ack_set_delay = buf[0];
            ioctl_arg.aer_timings.cfg_sample_delay  = buf[1];
            ioctl_arg.aer_timings.cfg_ack_rel_delay = buf[2];
            if (copy_to_user((icub_vsctrl_ioctl_arg_t *)arg, &ioctl_arg, sizeof(icub_vsctrl_ioctl_arg_t)))
                goto ctuser_err;
            break;

        case IOC_GET_BG_TIMINGS:
            if (icub_vsctrl_read_word(pdata, C_BGPRESC0_ADDR, &(ioctl_arg.bg_timings.prescaler_value)))
                goto read_err;
            if (icub_vsctrl_blockread_byte(pdata, C_BGTIM0_ADDR, buf, 4))
                goto read_err;
            ioctl_arg.bg_timings.latch_active_time = buf[0];
            ioctl_arg.bg_timings.latch_setup_time  = buf[1]; 
            ioctl_arg.bg_timings.clock_active_time = buf[2]; 
            ioctl_arg.bg_timings.setup_hold_time   = buf[3]; 
            if (copy_to_user((icub_vsctrl_ioctl_arg_t *)arg, &ioctl_arg, sizeof(icub_vsctrl_ioctl_arg_t)))
                goto ctuser_err;
            break;

        case IOC_CLR_STATUS:
            if (icub_vsctrl_write_word(pdata, C_STATUS_ADDR32, arg))
                goto write_err;
            break;

            break;

        default: 
            mutex_unlock(&pdata->mutex);
            return -EINVAL;
    }
    
    mutex_unlock(&pdata->mutex);
	return 0;
    
cfuser_err:
    mutex_unlock(&pdata->mutex);
    printk(KERN_ERR MSG_PREFIX "Copy from user space failed\n");
    return -EFAULT;

ctuser_err:
    mutex_unlock(&pdata->mutex);
    printk(KERN_ERR MSG_PREFIX "Copy to user space failed\n");
    return -EFAULT;

read_err :
    mutex_unlock(&pdata->mutex);
    printk(KERN_ERR MSG_PREFIX "Read from I2C failed\n");
    return -EFAULT;
    
write_err :
    mutex_unlock(&pdata->mutex);
    printk(KERN_ERR MSG_PREFIX "Write to I2C failed\n");
    return -EFAULT;
}


static const struct file_operations icub_vsctrl_miscdev_fops = {
    .owner          = THIS_MODULE,
    .open           = icub_vsctrl_misc_open,
    .write          = icub_vsctrl_misc_write,
    .read           = icub_vsctrl_misc_read,
    .unlocked_ioctl = icub_vsctrl_misc_ioctl,
    .release        = icub_vsctrl_misc_release,
};


static int icub_vsctrl_probe(struct i2c_client *client, const struct i2c_device_id *id) {
	int ret = 0;
	struct icub_vsctrl_data *pdata = NULL;
	struct device *dev = &client->dev;

	printk(KERN_DEBUG "%s()\n", __func__);
	
	pdata = devm_kzalloc(dev, sizeof(struct icub_vsctrl_data), GFP_KERNEL);
	if(!pdata) {
		printk(KERN_ERR "No memory!\n");
		ret = -ENOMEM;
        goto probe_exit;
	}

	pdata->client = client;
	mutex_init(&pdata->mutex);
    
	pdata->misc_dev.minor	= MISC_DYNAMIC_MINOR;
    if (client->addr == 0x2d) {
        pdata->misc_dev.name	= "vsctrl_r"; // ???
        pdata->misc_dev.nodename = "iit_vsctrl_r";
    }
    else if (client->addr == 0x2e) {
        pdata->misc_dev.name	= "vsctrl_l"; // ???
        pdata->misc_dev.nodename = "iit_vsctrl_l";
    }
    
	pdata->misc_dev.fops	= &icub_vsctrl_miscdev_fops;
    pdata->misc_dev.mode    = S_IRUGO | S_IWUGO;
    
    
	ret = misc_register(&pdata->misc_dev);
	if (ret < 0) {
        printk(KERN_ALERT "misc_register() failed for device @addr 0x%x\n", client->addr);
		goto misc_register_fail;
	}
    
	i2c_set_clientdata(client, pdata);
    
    mutex_lock(&pdata->mutex);
    icub_vsctrl_init(pdata, C_CHIP_AUTO);
    mutex_unlock(&pdata->mutex);
	
	//printk(KERN_INFO "%s driver initialized successfully @addr %x!\n", DRV_NAME, client->addr);
    printk(KERN_INFO "%s driver initialized successfully for chip %s @addr %x!\n", DRV_NAME, pdata->chip_type==C_CHIP_DVS ? "DVS" : (pdata->chip_type==C_CHIP_ATIS ? "ATIS" : "Unkn"), client->addr);
	return 0;

misc_register_fail:
	devm_kfree(dev, pdata);
probe_exit:
	return ret;
}


static int icub_vsctrl_remove(struct i2c_client *client)
{
    struct icub_vsctrl_data *pdata = i2c_get_clientdata(client);
	struct device *dev = &client->dev;
	
	printk(KERN_DEBUG "%s()\n", __func__);
	
	misc_deregister(&pdata->misc_dev);
	devm_kfree(dev, pdata);
	i2c_set_clientdata(client, NULL);
    return 0;
}


static const struct i2c_device_id icub_vsctrl_id[] = {
    { DRV_NAME, 0 },
    { }
};


static struct i2c_driver icub_vsctrl_driver = {
    .driver = {
        .owner	= THIS_MODULE,
        .name	= DRV_NAME,
    },
    .id_table	= icub_vsctrl_id,
    .probe		= icub_vsctrl_probe,
    .remove		= icub_vsctrl_remove,
};

module_i2c_driver(icub_vsctrl_driver);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Gaetano de Robertis <gaetano.derobertis@iit.it>");
MODULE_DESCRIPTION("IIT Driver for Vision Sensor Controller of iCub.");


