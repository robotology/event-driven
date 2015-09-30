#include <asm/uaccess.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/kdev_t.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
//#include <linux/sysdev.h>

#define DEV_NAME		   "iit-i2c_vsctrl"

#define MAX_CONV_MS  150

struct icub_vsctrl_data {
	struct i2c_client *client;
	struct class class;
	struct mutex mutex;
	//u32 temp_tos;
	//u32 temp_thys;
	//u32 temp_value;
};

static int icub_vsctrl_readwrite(struct i2c_client *client,
                   u16 wr_len, u8 *wr_buf,
                   u16 rd_len, u8 *rd_buf)
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
                     u8 addr, u8 value)
{
    u8 wrbuf[2]={0};

    wrbuf[0] = addr & 0x3F;
    wrbuf[1] = value;

    return icub_vsctrl_readwrite(pdata->client, 2, wrbuf, 0, NULL);
}

static int icub_vsctrl_fifowrite_byte(struct icub_vsctrl_data *pdata,
                     u8 addr, u8 value[], u16 len)
{
    //u8 wrbuf[len+1]={0};
    u8 wrbuf[len+1];
    int i=0;
    int j=0;

    wrbuf[j++] = (addr&0x3F)|0x00;
    while (i<len)
        wrbuf[j++] = value[i++];

    return icub_vsctrl_readwrite(pdata->client, j, wrbuf, 0, NULL);
}

static int icub_vsctrl_blockwrite_byte(struct icub_vsctrl_data *pdata,
                     u8 addr, u8 value[], u16 len)
{
    //u8 wrbuf[len+1]={0};
    u8 wrbuf[len+1];
    int i=0;
    int j=0;

    wrbuf[j++] = (addr&0x3F)|0x80;
    while (i<len)
        wrbuf[j++] = value[i++];

    return icub_vsctrl_readwrite(pdata->client, j, wrbuf, 0, NULL);
}

static int icub_vsctrl_write_word(struct icub_vsctrl_data *pdata,
                     u8 addr, u32 value)
{
    u8 wrbuf[5]={0};

    wrbuf[0] = (addr&0x3F)|0x40;
    wrbuf[1] = (value>> 0)&0xFF;
    wrbuf[2] = (value>> 8)&0xFF;
    wrbuf[3] = (value>>16)&0xFF;
    wrbuf[4] = (value>>24)&0xFF;

    return icub_vsctrl_readwrite(pdata->client, 5, wrbuf, 0, NULL);
}

static int icub_vsctrl_fifowrite_word(struct icub_vsctrl_data *pdata,
                     u8 addr, u32 value[], u16 len)
{
    //u8 wrbuf[4*len+1]={0};
    u8 wrbuf[4*len+1];
    int i=0;
    int j=0;

    wrbuf[j++] = (addr&0x3F)|0x40;
    for (i=0; i<len; i++) 
    while (i<len)
    {
        wrbuf[j++] = (value[i]>> 0)&0xFF;
        wrbuf[j++] = (value[i]>> 8)&0xFF;
        wrbuf[j++] = (value[i]>>16)&0xFF;
        wrbuf[j++] = (value[i]>>24)&0xFF;
        i++;
    }

    return icub_vsctrl_readwrite(pdata->client, j, wrbuf, 0, NULL);
}

static int icub_vsctrl_blockwrite_word(struct icub_vsctrl_data *pdata,
                     u8 addr, u32 value[], u16 len)
{
    //u8 wrbuf[4*len+1]={0};
    u8 wrbuf[4*len+1];
    int i=0;
    int j=0;

    wrbuf[j++] = (addr&0x3F)|0xC0;
    for (i=0; i<len; i++) 
    while (i<len)
    {
        wrbuf[j++] = (value[i]>> 0)&0xFF;
        wrbuf[j++] = (value[i]>> 8)&0xFF;
        wrbuf[j++] = (value[i]>>16)&0xFF;
        wrbuf[j++] = (value[i]>>24)&0xFF;
        i++;
    }

    return icub_vsctrl_readwrite(pdata->client, j, wrbuf, 0, NULL);
}




static int icub_vsctrl_read_byte(struct icub_vsctrl_data *pdata,
                    u8 addr, u8 *value)
{
    u8 wrbuf[1], rdbuf[1]={0};
    int error;

    wrbuf[0] = addr & 0x3F;

    error = icub_vsctrl_readwrite(pdata->client, 1, wrbuf, 1, rdbuf);
    if (error)
        return error;

    *value = rdbuf[0];
    return 0;
}

static int icub_vsctrl_fiforead_byte(struct icub_vsctrl_data *pdata,
                     u8 addr, u8 value[], u16 len)
{
    u8 wrbuf[1]={0};
    //u8 rdbuf[len]={0};
    u8 rdbuf[len];
    int error;
    int i=0;
    int j=0;

    wrbuf[0] = (addr&0x3F)|0x00;
    
    error = icub_vsctrl_readwrite(pdata->client, 1, wrbuf, len, rdbuf);
    if (error)
        return error;
        
    while (i<len)
        value[j++] = rdbuf[i++];

    return 0;
}

static int icub_vsctrl_blockread_byte(struct icub_vsctrl_data *pdata,
                     u8 addr, u8 value[], u16 len)
{
    u8 wrbuf[1]={0};
    //u8 rdbuf[len]={0};
    u8 rdbuf[len];
    int error;
    int i=0;
    int j=0;

    wrbuf[0] = (addr&0x3F)|0x80;
    
    error = icub_vsctrl_readwrite(pdata->client, 1, wrbuf, len, rdbuf);
    if (error)
        return error;
        
    while (i<len)
        value[j++] = rdbuf[i++];

    return 0;
}

static int icub_vsctrl_read_word(struct icub_vsctrl_data *pdata,
                    u8 addr, u32 *value)
{
    u8 wrbuf[1], rdbuf[4]={0};
    int error;

    wrbuf[0] = (addr&0x3F)|0x40;

    error = icub_vsctrl_readwrite(pdata->client, 1, wrbuf, 4, rdbuf);
    if (error)
        return error;

    *value = (rdbuf[0]|rdbuf[1]<<8|rdbuf[2]<<16|rdbuf[3]<<24);
    return 0;
}

static int icub_vsctrl_fiforead_word(struct icub_vsctrl_data *pdata,
                     u8 addr, u32 value[], u16 len)
{
    u8 wrbuf[1]={0};
    //u8 rdbuf[4*len]={0};
    u8 rdbuf[4*len];
    int error;
    int i=0;
    int j=0;

    wrbuf[0] = (addr&0x3F)|0x40;
    
    error = icub_vsctrl_readwrite(pdata->client, 1, wrbuf, 4*len, rdbuf);
    if (error)
        return error;
        
    while (i<len)
    {
        value[i++] = rdbuf[j]|rdbuf[j+1]<<8|rdbuf[j+2]<<16|rdbuf[j+3]<<24;
        j+=4;
    }

    return 0;
}

static int icub_vsctrl_blockread_word(struct icub_vsctrl_data *pdata,
                     u8 addr, u32 value[], u16 len)
{
    u8 wrbuf[1]={0};
    //u8 rdbuf[4*len]={0};
    u8 rdbuf[4*len];
    int error;
    int i=0;
    int j=0;

    wrbuf[0] = (addr&0x3F)|0xC0;
    
    error = icub_vsctrl_readwrite(pdata->client, 1, wrbuf, 4*len, rdbuf);
    if (error)
        return error;
        
    while (i<len)
    {
        value[i++] = rdbuf[j]|rdbuf[j+1]<<8|rdbuf[j+2]<<16|rdbuf[j+3]<<24;
        j+=4;
    }

    return 0;
}


/* class attribute store function. */
static ssize_t icub_vsctrl_store_config(struct class *cls, struct class_attribute *attr, char *buf, size_t count)
{
	struct icub_vsctrl_data *pdata = (struct icub_vsctrl_data *)container_of(cls, struct icub_vsctrl_data, class);
	int ret;
	//int value;
	unsigned long start_time;
	
	mutex_lock(&pdata->mutex);
	
	start_time = jiffies;
//	pdata->temp_value = myir_stlm75x_read_word(pdata, REG_TEMP);
////	printk(KERN_ERR "pdata->temp_value: %#X", pdata->temp_value);
//	value = to_readable_value(pdata->temp_value);
//	ret = sprintf(buf, "%d.%s\n", value/2, value%2?"5":"0");

	while (time_before(jiffies, start_time + msecs_to_jiffies(MAX_CONV_MS))) schedule();
	
	mutex_unlock(&pdata->mutex);
	
	return ret;
}

/* class attribute show function. */
static ssize_t icub_vsctrl_show_config(struct class *cls, struct class_attribute *attr, char *buf)
{
	struct icub_vsctrl_data *pdata = (struct icub_vsctrl_data *)container_of(cls, struct icub_vsctrl_data, class);
	int ret;
	//int value;
	unsigned long start_time;
	
	mutex_lock(&pdata->mutex);
	
	start_time = jiffies;
//	pdata->temp_value = myir_stlm75x_read_word(pdata, REG_TEMP);
////	printk(KERN_ERR "pdata->temp_value: %#X", pdata->temp_value);
//	value = to_readable_value(pdata->temp_value);
//	ret = sprintf(buf, "%d.%s\n", value/2, value%2?"5":"0");

	while (time_before(jiffies, start_time + msecs_to_jiffies(MAX_CONV_MS))) schedule();
	
	mutex_unlock(&pdata->mutex);
	
	return ret;
}


/* class attribute store function. */
static ssize_t icub_vsctrl_store_bias(struct class *cls, struct class_attribute *attr, char *buf, size_t count)
{
	struct icub_vsctrl_data *pdata = (struct icub_vsctrl_data *)container_of(cls, struct icub_vsctrl_data, class);
	int ret;
	//int value;
	unsigned long start_time;
	
	mutex_lock(&pdata->mutex);
	
	start_time = jiffies;
//	pdata->temp_value = myir_stlm75x_read_word(pdata, REG_TEMP);
////	printk(KERN_ERR "pdata->temp_value: %#X", pdata->temp_value);
//	value = to_readable_value(pdata->temp_value);
//	ret = sprintf(buf, "%d.%s\n", value/2, value%2?"5":"0");

	while (time_before(jiffies, start_time + msecs_to_jiffies(MAX_CONV_MS))) schedule();
	
	mutex_unlock(&pdata->mutex);
	
	return ret;
}

/* class attribute show function. */
static ssize_t icub_vsctrl_show_bias(struct class *cls, struct class_attribute *attr, char *buf)
{
	struct icub_vsctrl_data *pdata = (struct icub_vsctrl_data *)container_of(cls, struct icub_vsctrl_data, class);
	int ret;
	//int value;
	unsigned long start_time;
	
	mutex_lock(&pdata->mutex);
	
	start_time = jiffies;
//	pdata->temp_value = myir_stlm75x_read_word(pdata, REG_TEMP);
////	printk(KERN_ERR "pdata->temp_value: %#X", pdata->temp_value);
//	value = to_readable_value(pdata->temp_value);
//	ret = sprintf(buf, "%d.%s\n", value/2, value%2?"5":"0");

	while (time_before(jiffies, start_time + msecs_to_jiffies(MAX_CONV_MS))) schedule();
	
	mutex_unlock(&pdata->mutex);
	
	return ret;
}



/* Attributes declaration: Here I have declared two attributes */
static struct class_attribute icub_vsctrl_class_attrs[] = {
	__ATTR(bias_gen, S_IRUGO | S_IWUSR , icub_vsctrl_show_bias, icub_vsctrl_store_bias), //use macro for permission
	__ATTR(config, S_IRUGO | S_IWUSR , icub_vsctrl_show_config, icub_vsctrl_store_config),     //use macro for permission
	__ATTR_NULL
};


static int icub_vsctrl_probe(struct i2c_client *client,
                     const struct i2c_device_id *id)
{
	int ret = 0;
	struct icub_vsctrl_data *pdata = NULL;
	
	printk(KERN_ALERT "%s()\n", __func__);
	
	pdata = kmalloc(sizeof(struct icub_vsctrl_data), GFP_KERNEL);
	if(!pdata) {
		printk(KERN_ERR "No memory!\n");
		return -ENOMEM;
	}
	memset(pdata, 0, sizeof(struct icub_vsctrl_data));

	pdata->client = client;
	
	/* Init class */
	mutex_init(&pdata->mutex);
	pdata->class.name = DEV_NAME;
	pdata->class.owner = THIS_MODULE;
	pdata->class.class_attrs = icub_vsctrl_class_attrs;
	ret = class_register(&pdata->class);
	if(ret) {
		printk(KERN_ERR "class_register failed @addr %x, err = %d!\n", client->addr, ret);
		goto class_register_fail;
	}
	i2c_set_clientdata(client, pdata);
	
	printk(KERN_ALERT "%s driver initialized successfully @addr %x!\n", DEV_NAME, client->addr);
	return 0;

class_register_fail:
	
	return ret;
}


static int icub_vsctrl_remove(struct i2c_client *client)
{
    struct icub_vsctrl_data *pdata = i2c_get_clientdata(client);
	
	class_unregister(&pdata->class);
	kfree(pdata);
	i2c_set_clientdata(client, NULL);
    return 0;
}


static const struct i2c_device_id icub_vsctrl_id[] = {
    { DEV_NAME, 0 },
    { }
};


static struct i2c_driver icub_vsctrl_driver = {
    .driver = {
        .owner	= THIS_MODULE,
        .name	= DEV_NAME,
    },
    .id_table	= icub_vsctrl_id,
    .probe		= icub_vsctrl_probe,
    .remove		= icub_vsctrl_remove,
};

module_i2c_driver(icub_vsctrl_driver);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Gaetano de Robertis <gaetano.derobertis@iit.it>");
MODULE_DESCRIPTION("IIT Driver for Vision Sensor Controller of iCub.");
