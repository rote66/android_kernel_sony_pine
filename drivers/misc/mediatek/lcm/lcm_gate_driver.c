/*****************************************************************************
** LCM Gate Driver
******************************************************************************/
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>

#define I2C_ID_NAME "ktd2151"

static int ktd2151_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int ktd2151_remove(struct i2c_client *client);

static struct i2c_client *ktd2151_i2c_client;

static const struct of_device_id lcm_of_match[] = {
		{ .compatible = "mediatek,i2c_lcd_bias" },
		{},
};


/**********************************************************
**
***********************************************************/
static const struct i2c_device_id ktd2151_id[] = {
	{ I2C_ID_NAME, 0 },
	{ }
};

static struct i2c_driver ktd2151_iic_driver = {
	.id_table	= ktd2151_id,
	.probe		= ktd2151_probe,
	.remove		= ktd2151_remove,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "ktd2151",
		.of_match_table = lcm_of_match,
	},
};

static int ktd2151_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    printk("%s...\n", __func__ );
    printk("TPS: info==>name=%s addr=0x%x\n", client->name, client->addr);

    ktd2151_i2c_client = client;

    return 0;
}

static int ktd2151_remove(struct i2c_client *client)
{
    ktd2151_i2c_client = NULL;
    i2c_unregister_device(client );

    return 0;
}

/**********************************************************
**
***********************************************************/
int lcm_gate_write_bytes(unsigned char addr, unsigned char value)
{
    int ret = 0;
    char write_data[2]={0};
    struct i2c_client *client = ktd2151_i2c_client;

    write_data[0] = addr;
    write_data[1] = value;

    ret = i2c_master_send( client, write_data, 2 );
    if( ret < 0 )
      printk("lcm_gate write data fail !!\n");

    return ret ;
}

EXPORT_SYMBOL(lcm_gate_write_bytes);

/**************************************************************************
** module load/unload record keeping
***************************************************************************/
static int __init lcm_gate_i2c_init(void)
{
    printk("%s...\n", __func__ );

    if( i2c_add_driver( &ktd2151_iic_driver ))
    {
        printk("lcm_gate add I2C driver error\n");
        return -1;
    }
    else
        printk("lcm_gate add I2C driver success\n");

    return 0;
}

static void __exit lcm_gate_i2c_exit(void)
{
    printk("%s...\n", __func__ );

    i2c_del_driver( &ktd2151_iic_driver );
}


module_init(lcm_gate_i2c_init);
module_exit(lcm_gate_i2c_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("LCM Gate I2C driver");
