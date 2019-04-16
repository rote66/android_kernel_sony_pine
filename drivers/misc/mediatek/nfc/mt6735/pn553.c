/*
 * Copyright (C) 2010 Trusted Logic S.A.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

//#define DEBUG
//#define NFC_DEBUG

#ifdef DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif

/*****************************************************************************
** Include
******************************************************************************/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/device.h>
#include "pn553.h"
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
//[sm31][NFC][16100401][Kent][begin] To enhance the message handling in suspend mode
#include <linux/wakelock.h>
//[sm31][NFC][16100401][Kent][end] To enhance the message handling in suspend mode

/*****************************************************************************
** Macro-Define
******************************************************************************/
#define NFC_CLIENT_TIMING 400
#define I2C_RETRY_TIMES 0
/*========================================================
** For information print out
**========================================================*/
#define   I2C_DMA_USAGE
#if defined( I2C_DMA_USAGE )
    #include  <linux/dma-mapping.h>
#endif


#define   PN553_DEV_ADDR            (0x28)  //(0x2B)  /* PN547C2: 0x2B, PN547: 0x28*/
#define   PN553_SLAVE_ADDR          (PN553_DEV_ADDR<<1)
#define   NFC_DEV_NAME              "pn54x"
#define   I2C_ID_NAME               "pn553"
#define   MAX_BUFFER_SIZE           512
#define   ENABLE_DELAY              50
#define   DISABLE_DELAY             70
#define   VEN_ENABALE               1
#define   VEN_DISABALE              0

typedef struct st_pn553_dev
{
    wait_queue_head_t   read_wq;
    struct mutex        read_mutex;
    struct i2c_client * client;
    struct miscdevice   pn553_device;
   //[sm31][NFC][16100401][Kent][begin] To enhance the message handling in suspend mode
    struct wake_lock wake_lock;
    //[sm31][NFC][16100401][Kent][end] To enhance the message handling in suspend mode
    unsigned int        ven_gpio;
    unsigned int        firm_gpio;
    unsigned int        irq_gpio;
    bool                irq_enabled;
    spinlock_t          irq_enabled_lock;
} PN553_DEV;

/* For DMA */
#if defined( I2C_DMA_USAGE )
static char   * I2CDMAWriteBuf = NULL;
static uintptr_t   I2CDMAWriteBuf_pa;  // = NULL;
static char   * I2CDMAReadBuf = NULL;
static uintptr_t  I2CDMAReadBuf_pa;   // = NULL;
#endif /* End.. (I2C_DMA_USAGE) */

struct pn553_i2c_platform_data  pn553_pdata;
static PN553_DEV  * g_pn553_dev = NULL;
static u32 nfc_irq;
struct platform_device *nfc_plt_dev = NULL;
struct pinctrl *gpctrl = NULL;
struct pinctrl_state *st_ven_h = NULL;
struct pinctrl_state *st_ven_l = NULL;
struct pinctrl_state *st_rst_h = NULL;
struct pinctrl_state *st_rst_l = NULL;
struct pinctrl_state *st_eint_h = NULL;
struct pinctrl_state *st_eint_l = NULL;
struct pinctrl_state *st_irq_init = NULL;

static int mt_nfc_pinctrl_select(struct pinctrl *p, struct pinctrl_state *s);
/*****************************************************************************
**
******************************************************************************/
enum {
	MTK_NFC_GPIO_DIR_IN = 0x0,
	MTK_NFC_GPIO_DIR_OUT,
	MTK_NFC_GPIO_DIR_INVALID,
};

static int mt_nfc_get_gpio_dir(int gpio_num)
{
	if (gpio_num == pn553_pdata.irq_gpio) {
		return MTK_NFC_GPIO_DIR_IN;	/* input */

	} else if ((gpio_num == pn553_pdata.ven_gpio) ||
		   (gpio_num == pn553_pdata.firm_gpio) ) {
		return MTK_NFC_GPIO_DIR_OUT;	/* output */

	} else {
		return MTK_NFC_GPIO_DIR_INVALID;

	}
}
static int mt_nfc_get_gpio_value(int gpio_num)
{
	int value = 0;
	if (mt_nfc_get_gpio_dir(gpio_num) != MTK_NFC_GPIO_DIR_INVALID) {
#if !defined(CONFIG_MTK_LEGACY)
		value = __gpio_get_value(gpio_num);
#else
		value = mt_get_gpio_in(gpio_num);
#endif
	}
	return value;
}

static void pn553_disable_irq(PN553_DEV *pn553_dev)
{
    unsigned long   flags;
    CDBG("%s +++\n", __func__);
    spin_lock_irqsave( &pn553_dev->irq_enabled_lock, flags );
    if( pn553_dev->irq_enabled )
    {
    	CDBG("%s: pn553_dev->irq_enabled\n", __func__);
	    disable_irq_nosync(nfc_irq);
        pn553_dev->irq_enabled = false;
    }
    spin_unlock_irqrestore( &pn553_dev->irq_enabled_lock, flags );
    CDBG("%s ---\n", __func__);
}

/*****************************************************************************
**
******************************************************************************/
static void pn553_enable_irq(PN553_DEV *pn553_dev)
{
    unsigned long   flags;
    CDBG("%s +++\n", __func__);
    spin_lock_irqsave( &pn553_dev->irq_enabled_lock, flags );
    if( !( pn553_dev->irq_enabled ))
    {
    	CDBG("%s: !( pn553_dev->irq_enabled )\n", __func__);
	    enable_irq(nfc_irq);
        pn553_dev->irq_enabled = true;
    }
    spin_unlock_irqrestore( &pn553_dev->irq_enabled_lock, flags );
    CDBG("%s ---\n", __func__);
}

/*****************************************************************************
**
******************************************************************************/

static irqreturn_t pn553_dev_irq_handler(int irq, void *dev_id)
{
    PN553_DEV *pn553_dev = g_pn553_dev;
    CDBG( "%s, enable irq \n", __func__);
    if(mt_nfc_get_gpio_value(pn553_pdata.irq_gpio))
    {
      pn553_disable_irq( pn553_dev );
      wake_up_interruptible( &pn553_dev->read_wq );
    }
    else
    {
    	CDBG( "%s, no irq \n", __func__);
    }
    return  IRQ_HANDLED;
}

/*****************************************************************************
**
******************************************************************************/
static ssize_t pn553_dev_read(struct file *filp, char __user *buf,
    size_t count, loff_t *offset)
{
    PN553_DEV *pn553_dev = filp->private_data;
	#if I2C_RETRY_TIMES
    int retry_i;
	#endif
    #if !defined( I2C_DMA_USAGE )
    char  tmp[MAX_BUFFER_SIZE];
    #endif
    int   ret = 0;
    #if defined( NFC_DEBUG )
    int   i;
    #endif /* NFC_DEBUG */
    unsigned short  addr;
    __u32           ext_flag;

    CDBG("[NFC]%s: Enter for reading %zu bytes.\n", __func__, count );
    if( count > MAX_BUFFER_SIZE )
      count = MAX_BUFFER_SIZE;
    //[sm31][NFC][16100401][Kent][begin] To enhance the message handling in suspend mode
    current->flags |= PF_NOFREEZE;
    current->flags &= ~PF_FROZEN;
    //[sm31][NFC][16100401][Kent][begin] To enhance the message handling in suspend mode
    mutex_lock( &pn553_dev->read_mutex );
    CDBG("%s: Mutex Lock\n", __func__);
    if( !mt_nfc_get_gpio_value(pn553_pdata.irq_gpio) )
    {
    	CDBG("%s: no irq, irq_gpio: %d, irq_enabled: %d\n", __func__,
        mt_nfc_get_gpio_value(pn553_pdata.irq_gpio), pn553_dev->irq_enabled );

        if( filp->f_flags & O_NONBLOCK )
        {
            ret = -EAGAIN;
            goto fail;
        }
        if( !( pn553_dev->irq_enabled ))
        {
        	CDBG("%s: enable_irq\n", __func__ );
            pn553_enable_irq(pn553_dev);
        }

        CDBG("%s: start wait!\n", __func__ );
        //ret = wait_event_interruptible_timeout( pn553_dev->read_wq, pn553_dev->irq_enabled == false, 5*HZ);
        ret = wait_event_interruptible(pn553_dev->read_wq,(mt_nfc_get_gpio_value(pn553_pdata.irq_gpio)) );
        CDBG("%s, ret: 0x%X\n", __func__, ret );
        pn553_disable_irq( pn553_dev );
        if(!mt_nfc_get_gpio_value(pn553_pdata.irq_gpio))
        {
            ret = -EIO;
            goto fail;
        }
    }
    addr      = pn553_dev->client->addr;
    ext_flag  = pn553_dev->client->ext_flag;

	#if defined( I2C_DMA_USAGE )
	//pn553_dev->client->addr     &= I2C_MASK_FLAG;
	pn553_dev->client->addr     |= I2C_DMA_FLAG;
	//pn553_dev->client->addr     |= I2C_ENEXT_FLAG;
	pn553_dev->client->ext_flag |= I2C_DMA_FLAG;
	//pn553_dev->client->ext_flag |= I2C_DIRECTION_FLAG;
	//pn553_dev->client->ext_flag |= I2C_A_FILTER_MSG;
	pn553_dev->client->timing = NFC_CLIENT_TIMING;

	/* Read data */
	#if I2C_RETRY_TIMES
	for(retry_i=0; retry_i < I2C_RETRY_TIMES ;retry_i++) //pn553 i2c must be retry
	{
	#endif
		ret = i2c_master_recv( pn553_dev->client, (unsigned char *)I2CDMAReadBuf_pa, count );
	#if I2C_RETRY_TIMES
		if( ret < 0 )
		{
		pr_err("%s: i2c_master_recv retry %d\n", __func__, retry_i );
		continue;
		}else
		{
		break;
		}
	}//for(retry_i=0;
	#endif

//	udelay(1000);

    pn553_dev->client->addr     = addr;
    pn553_dev->client->ext_flag = ext_flag;
    mutex_unlock( &pn553_dev->read_mutex );
    CDBG("%s: Mutex unLock\n", __func__);

	#if I2C_RETRY_TIMES
	if(retry_i >= I2C_RETRY_TIMES)
	{
		pr_err("%s: i2c_master_recv returned %d\n", __func__, ret );
		return ret;
	}
	#else
	if(ret < 0)
	{
		pr_err("%s: i2c_master_recv returned %d\n", __func__, ret );
		return ret;
	}
	#endif //#if I2C_RETRY_TIMES

	if( ret > count )
	{
		pr_err("%s: received too many bytes from i2c (%d)\n", __func__, ret );
		return -EIO;
	}
	#else //#if defined( I2C_DMA_USAGE )
		/* Read data */
		pn553_dev->client->timing = NFC_CLIENT_TIMING;
		if(count <=8 )
		{
			pn553_dev->client->addr&=I2C_MASK_FLAG;
			ret = i2c_master_recv( pn553_dev->client, (unsigned char *)tmp, count );
		break;
		}else
		{
			int read_i,read_times,read_reduncy;
			unsigned char read_buf[8];
			unsigned char *buf_ptr = tmp;

			pr_err("%s: i2c_master_recv over length 8!\n", __func__ );
			pn553_dev->client->addr&=I2C_MASK_FLAG;
			read_times = count /8;
			for(read_i=0;read_i <read_times;read_i++)
			{
				ret = i2c_master_recv( pn553_dev->client, (unsigned char *)read_buf, 8 );
				if( ret != 8 )
				{
				  pr_err("%s : i2c_master_recv failed %d retry %d\n", __func__, ret, 0 );
				  ret = -EIO;
				  //continue;
				}
				memcpy(buf_ptr,read_buf,8);
				buf_ptr +=8;
			}
			read_reduncy = count%8;
			if(read_reduncy)
			{
				ret = i2c_master_recv( pn553_dev->client, (unsigned char *)read_buf, read_reduncy );
				memcpy(buf_ptr,read_buf,read_reduncy);
			}
		}

//		udelay(1000);

	    pn553_dev->client->addr     = addr;
	    pn553_dev->client->ext_flag = ext_flag;
	    mutex_unlock( &pn553_dev->read_mutex );
	    pr_err("%s: Mutex unLock\n", __func__);

	    if( ret < 0 )
	    {
	        CDBG("%s: i2c_master_recv returned %d\n", __func__, ret );
	        return ret;
	    }
	    if( ret > count )
	    {
	        CDBG("%s: received too many bytes from i2c (%d)\n", __func__, ret );
	        return -EIO;
	    }
	#endif //#if defined( I2C_DMA_USAGE )



#if defined( I2C_DMA_USAGE )
    if( copy_to_user( buf, I2CDMAReadBuf, ret ))
#else
    if( copy_to_user( buf, tmp, ret ))
#endif
    {
        pr_warning("%s : failed to copy to user space.\n", __func__);
        return -EFAULT;
    }

#if defined( NFC_DEBUG )
    CDBG( "%s: bytes[%d] ", __func__, (int)count );
       #if defined( I2C_DMA_USAGE )
	if(I2CDMAReadBuf!=NULL)
        #else
	if(tmp!=NULL)
       #endif
	{
		for( i = 0; i < count; i++ )
		{
		#if defined( I2C_DMA_USAGE )
			CDBG("%02X ", *(I2CDMAReadBuf + i));
		#else
			CDBG("%02X ", tmp[i] );
		#endif
		}
	}
	else
		CDBG(" tmp==Null \n");
	CDBG(" \n");
#endif /* NFC_DEBUG */
	//[sm31][NFC][16100401][Kent][begin] To enhance the message handling in suspend mode
    wake_lock_timeout(&pn553_dev->wake_lock, msecs_to_jiffies(1500));
    //[sm31][NFC][16100401][Kent][end] To enhance the message handling in suspend mode
    CDBG("%s complete, irq: %d\n", __func__, mt_nfc_get_gpio_value(pn553_pdata.irq_gpio));//jack
    return ret;

fail:
    mutex_unlock( &pn553_dev->read_mutex );
    pr_err("%s error ---, ret: 0x%X\n", __func__, ret );
    return ret;
}

/*****************************************************************************
**
******************************************************************************/
static ssize_t pn553_dev_write(struct file *filp, const char __user *buf,
    size_t count, loff_t *offset)
{
#if 1
    PN553_DEV * pn553_dev;
	#if I2C_RETRY_TIMES
    int retry_i;
	#endif
    #if !defined( I2C_DMA_USAGE )
    char  tmp[MAX_BUFFER_SIZE];
    #endif
    #if defined( NFC_DEBUG )
    int   i;
    #endif /* NFC_DEBUG */
    int   ret ;
    unsigned short  addr;
    __u32           ext_flag;

    CDBG("[NFC]%s: Enter for writing %zu bytes.\n", __func__, count );

    pn553_dev = filp->private_data;

    if( count > MAX_BUFFER_SIZE )
      count = MAX_BUFFER_SIZE;

#if defined( I2C_DMA_USAGE )
    if( copy_from_user( I2CDMAWriteBuf, buf, count ))
#else
    if( copy_from_user( tmp, buf, count ))
#endif
    {
      pr_err( "%s : failed to copy from user space\n", __func__ );
      return -EFAULT;
    }

#if defined( NFC_DEBUG )
    CDBG("%s: bytes[%d] ", __func__, (int)count );
    #if defined( I2C_DMA_USAGE )
	if(I2CDMAReadBuf!=NULL)
    #else
	if(tmp!=NULL)
    #endif
	{
		for( i = 0; i < count; i++ )
		{
		#if defined( I2C_DMA_USAGE )
			CDBG("%02X ", *(I2CDMAWriteBuf + i));
		#else
			CDBG("%02X ", tmp[i] );
		#endif
		}
	}else
		CDBG(" temp==NULL\n");
	CDBG(" \n");
#endif /* NFC_DEBUG */

    addr      = pn553_dev->client->addr;
    ext_flag  = pn553_dev->client->ext_flag;
    CDBG("%02X ",addr);
#if defined( I2C_DMA_USAGE )
   // pn553_dev->client->addr     &= I2C_MASK_FLAG;
    pn553_dev->client->addr     |= I2C_DMA_FLAG;
  //pn553_dev->client->addr     |= I2C_ENEXT_FLAG;
    pn553_dev->client->ext_flag |= I2C_DMA_FLAG;
  //pn553_dev->client->ext_flag |= I2C_DIRECTION_FLAG;
  //pn553_dev->client->ext_flag |= I2C_A_FILTER_MSG;
#endif
    pn553_dev->client->timing = NFC_CLIENT_TIMING;
    /* Write data */
	#if I2C_RETRY_TIMES
    for(retry_i=0; retry_i < I2C_RETRY_TIMES ;retry_i++)//pn553 i2c must be retry
    {
	#endif //I2C_RETRY_TIMES

	#if defined( I2C_DMA_USAGE )
		  ret = i2c_master_send( pn553_dev->client, (unsigned char *)I2CDMAWriteBuf_pa, count );
		//ret = i2c_master_send( pn553_dev->client, (unsigned char *)I2CDMAWriteBuf, count );
	#else
		ret = i2c_master_send( pn553_dev->client, (unsigned char *)tmp, count );
	#endif

	#if I2C_RETRY_TIMES
		if( ret != count )
		{
		  pr_err("%s : i2c_master_send failed %d retry %d\n", __func__, ret, retry_i );
		  ret = -EIO;

		  continue;
		}else
		{
			break;
		}
    }

    if(retry_i >= I2C_RETRY_TIMES)
    {
    	pr_err("%s :  failed \n", __func__ );
    	ret = -EIO;
    }
	#endif //#if I2C_RETRY_TIMES

    //udelay(1000);

    pn553_dev->client->addr     = addr;
    pn553_dev->client->ext_flag = ext_flag;

    CDBG("%s : complete, result = %d\n", __func__, ret );
    return ret;
#else	//#if 1
	PN553_DEV * pn553_dev;
	int ret = 0, ret_tmp = 0, count_ori = 0,count_remain = 0, idx = 0;
	pn553_dev = filp->private_data;
    count_ori = count;
    count_remain = count_ori;
    CDBG("[NFC]%s: Enter for writing %zu bytes.\n", __func__, count );
	if (count > MAX_BUFFER_SIZE)
	{
		count = MAX_BUFFER_SIZE;
		count_remain -= count;
	}
    do
    {
        if (copy_from_user(I2CDMAWriteBuf, &buf[(idx*512)], count)) 
        {
            pr_err(KERN_DEBUG "%s : failed to copy from user space\n", __func__);
            return -EFAULT;
    	}
    	//pr_err(KERN_DEBUG "%s : writing %zu bytes, remain bytes %zu.\n", __func__, count, count_remain);
    	pr_err(KERN_DEBUG "%s : writing %zu bytes, remain bytes %d.\n", __func__, count, count_remain);
    	
    	/* Write data */
        pn553_dev->client->addr = (pn553_dev->client->addr & I2C_MASK_FLAG);// | I2C_DMA_FLAG;
        pn553_dev->client->ext_flag |= I2C_DMA_FLAG;
        pn553_dev->client->timing = NFC_CLIENT_TIMING;
        ret_tmp = i2c_master_send(pn553_dev->client, (unsigned char *)(uintptr_t)I2CDMAWriteBuf_pa, count);
        if (ret_tmp != count) 
        {
            pr_err(KERN_DEBUG "%s : i2c_master_send returned %d\n", __func__, ret);
            ret = -EIO;
            return ret;
        }
    	
        ret += ret_tmp;
        pr_err(KERN_DEBUG "%s : %d,%d,%d\n", __func__, ret_tmp,ret,count_ori);

        if( ret ==  count_ori)
    	{
            pr_err(KERN_DEBUG "%s : ret== count_ori \n", __func__);
            break;		
    	}
    	else
    	{
            if(count_remain > MAX_BUFFER_SIZE)
            {
                count = MAX_BUFFER_SIZE;
    		    count_remain -= MAX_BUFFER_SIZE;
            }
            else
            {
                count = count_remain;
                count_remain = 0;
            }
            idx++;		
            pr_err(KERN_DEBUG "%s :remain_bytes, %d,%d,%d,%d,%d\n", __func__, ret_tmp,ret,(int)count,count_ori,idx);
    	}
	}
	while(1);

	pr_err(KERN_DEBUG "%s : writing %d bytes. Status %d \n", __func__, count_ori,ret);
	return ret;
#endif
}

/*****************************************************************************
**
******************************************************************************/
static int pn553_dev_open(struct inode *inode, struct file *filp)
{
PN553_DEV *pn553_dev = container_of( filp->private_data,
                          PN553_DEV,
                          pn553_device );

    CDBG("[NFC]%s: Enter...\n", __func__ );
    filp->private_data = pn553_dev;
    CDBG("%s : %d, %d\n", __func__, imajor( inode ), iminor( inode ));
    return 0;
}

/*****************************************************************************
**
******************************************************************************/
static long pn553_dev_ioctl(struct file *filp,
          unsigned int cmd, unsigned long arg)
{
	CDBG("[NFC]%s: Enter...\n", __func__ );
    switch( cmd )
    {
      case PN553_SET_PWR:
      {
        if( arg == 2 )
        {
        	pr_err("[NFC]%s power on with firmware\n", __func__ );
		    mt_nfc_pinctrl_select(gpctrl,st_eint_h);
		    usleep_range(ENABLE_DELAY*900, ENABLE_DELAY*1000);
		    mt_nfc_pinctrl_select(gpctrl,st_ven_l);
		    usleep_range(DISABLE_DELAY*900, DISABLE_DELAY*1000);
		    mt_nfc_pinctrl_select(gpctrl,st_ven_h);
		    usleep_range(ENABLE_DELAY*900, ENABLE_DELAY*1000);
        }
        else if( arg == 1 )
        { /* power on */
        	pr_err("[NFC]%s power on\n", __func__ );
        	//[sm31][NFC][16100401][Kent][begin] To enhance the message handling in suspend mode
            pn553_enable_irq(filp->private_data);
            //[sm31][NFC][16100401][Kent][end] To enhance the message handling in suspend mode
		    mt_nfc_pinctrl_select(gpctrl,st_ven_h);
		    mt_nfc_pinctrl_select(gpctrl,st_eint_l);
            usleep_range(ENABLE_DELAY*900, ENABLE_DELAY*1000);
        }
        else if( arg == 0 )
        { /* power off */
        	pr_err("[NFC]%s power off\n", __func__ );
        	//[sm31][NFC][16100401][Kent][begin] To enhance the message handling in suspend mode
            pn553_enable_irq(filp->private_data);
            //[sm31][NFC][16100401][Kent][end] To enhance the message handling in suspend mode
		    mt_nfc_pinctrl_select(gpctrl,st_ven_l);
		    mt_nfc_pinctrl_select(gpctrl,st_eint_l);
            usleep_range(DISABLE_DELAY*900, DISABLE_DELAY*1000);
        }
        else
        {
            pr_err("%s bad arg %lu\n", __func__, arg );
            return -EINVAL;
        }
      } 
	  break;
      default:
      {
        pr_err("%s bad ioctl %u\n", __func__, cmd );
        return -EINVAL;
      }
    }
    return 0;
}

static const struct file_operations pn553_dev_fops = {
    .owner  = THIS_MODULE,
    .llseek = no_llseek,
    .read   = pn553_dev_read,
    .write  = pn553_dev_write,
    .open   = pn553_dev_open,
    .unlocked_ioctl = pn553_dev_ioctl,
};

static int mt_nfc_pinctrl_select(struct pinctrl *p, struct pinctrl_state *s)
{
	int ret = 0;
	if (p != NULL && s != NULL) {
		ret = pinctrl_select_state(p, s);
	} else {
		pr_debug("%s : pinctrl_select err\n", __func__);
		ret = -1;
	}
	return ret;
}
static int mt_nfc_pinctrl_init(struct platform_device *pdev,int probe_F)
{
	int ret = 0;
	gpctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(gpctrl)) {
		dev_err(&pdev->dev, "NFC Cannot find pinctrl!");
		ret = PTR_ERR(gpctrl);
		goto end;
	}
	
	st_ven_h = pinctrl_lookup_state(gpctrl, "ven_high");
	if (IS_ERR(st_ven_h)) {
		ret = PTR_ERR(st_ven_h);
		pr_debug("%s : pinctrl err, ven_high NFC\n", __func__);
	}
	st_ven_l = pinctrl_lookup_state(gpctrl, "ven_low");
	if (IS_ERR(st_ven_l)) {
		ret = PTR_ERR(st_ven_l);
		pr_debug("%s : pinctrl err, ven_low NFC\n", __func__);
	}
/*	st_rst_h = pinctrl_lookup_state(gpctrl, "rst_high");
	if (IS_ERR(st_rst_h)) {
		ret = PTR_ERR(st_rst_h);
		pr_debug("%s : pinctrl err, rst_high\n", __func__);
	}

	st_rst_l = pinctrl_lookup_state(gpctrl, "rst_low");
	if (IS_ERR(st_rst_l)) {
		ret = PTR_ERR(st_rst_l);
		pr_debug("%s : pinctrl err, rst_low\n", __func__);
	}
*/
	st_eint_h = pinctrl_lookup_state(gpctrl, "eint_high");
	if (IS_ERR(st_eint_h)) {
		ret = PTR_ERR(st_eint_h);
		pr_debug("%s : pinctrl err, eint_high NFC\n", __func__);
	}
	st_eint_l = pinctrl_lookup_state(gpctrl, "eint_low");
	if (IS_ERR(st_eint_l)) {
		ret = PTR_ERR(st_eint_l);
		pr_debug("%s : pinctrl err, eint_low NFC\n", __func__);
	}
	st_irq_init = pinctrl_lookup_state(gpctrl, "irq_init");
	if (IS_ERR(st_irq_init)) {
		ret = PTR_ERR(st_irq_init);
		pr_debug("%s : pinctrl err, irq_init NFC\n", __func__);
	}

	/* select state */
	if(probe_F==1)
	{
	    ret = mt_nfc_pinctrl_select(gpctrl, st_irq_init);
	    usleep_range(900, 1000);
	    ret = mt_nfc_pinctrl_select(gpctrl, st_ven_l);
	    usleep_range(900, 1000);
        //ret = mt_nfc_pinctrl_select(gpctrl, st_rst_h);
        //usleep_range(900, 1000);
	    ret = mt_nfc_pinctrl_select(gpctrl, st_eint_l);
	}
end:
	return ret;
}
static int mt_nfc_gpio_init(void)
{
	struct device_node *node;
	CDBG("[NFC] %s : 1\n", __func__);
	node = of_find_compatible_node(NULL, NULL, "mediatek,nfc-gpio-v2");
	CDBG("[NFC] %s : 2\n", __func__);
	if (node) {
		CDBG("[NFC] %s : 3\n", __func__);
		of_property_read_u32_array(node, "ven-gpio",
					   &(pn553_pdata.ven_gpio), 1);
	/*	of_property_read_u32_array(node, "gpio-rst",
					   &(mt6605_platform_data.sysrstb_gpio),
					   1);*/
		of_property_read_u32_array(node, "firm-gpio",
					   &(pn553_pdata.firm_gpio),
					   1);
		of_property_read_u32_array(node, "irq-gpio",
					   &(pn553_pdata.irq_gpio), 1);
	} else {
		pr_debug("%s : get gpio num err.\n", __func__);
		return -1;
	}
	CDBG("[NFC] %s : 4\n", __func__);
	pr_err("%s, ven_gpio:%d, firm_gpio:%d, irq_gpio:%d\n", __func__,pn553_pdata.ven_gpio,pn553_pdata.firm_gpio,pn553_pdata.irq_gpio);//jack
	return 0;
}
static int nxp_nfc_probe(struct platform_device *pdev)
{
	int ret = 0;
	nfc_plt_dev = pdev;
	CDBG("[NFC] %s : &nfc_plt_dev=%p\n", __func__, nfc_plt_dev);
	/* pinctrl init */
	ret = mt_nfc_pinctrl_init(pdev,1);
	/* gpio init */
	CDBG("[NFC] %s :1\n", __func__);
	if (mt_nfc_gpio_init() != 0)
		pr_debug("%s : mt_nfc_gpio_init err.\n", __func__);
	pr_err("[NFC] %s :2\n", __func__);
	return 0;
}
static int nxp_nfc_remove(struct platform_device *pdev)
{
	pr_err("[NFC] %s : &pdev=%p\n", __func__, pdev);
	return 0;
}

static const struct of_device_id nfc_dev_of_match[] = {
	{.compatible = "mediatek,nfc-gpio-v2",},
	{},
};
static struct platform_driver nxp_nfc_platform_driver = {
	.probe = nxp_nfc_probe,
	.remove = nxp_nfc_remove,
	.driver = {
		   .name = "pn553",
		   .owner = THIS_MODULE,
		   .of_match_table = nfc_dev_of_match,
		   },
};
/*****************************************************************************
**
******************************************************************************/
static int pn553_probe(struct i2c_client *client,
    const struct i2c_device_id *id)
{
    struct pn553_i2c_platform_data *platform_data;
    PN553_DEV *pn553_dev;
    int ret;
    struct device_node *node;
    pr_err("[NFC]%s: Enter...\n", __func__ );
    client->timing    = 400; /* 400 KHz */
	platform_data=&pn553_pdata;
	CDBG("[NFC]%s step 1...\n", __func__ );
    if( !platform_data )
    {
        pr_err( "%s, Can not get platform_data\n", __func__ );
        return -EINVAL;
    }

    dev_dbg( &client->dev, "pn553 probe: %s, inside pn553 flags = %x\n",
          __func__, client->flags );

    if( platform_data == NULL )
    {
        pr_err("%s : nfc probe fail\n", __func__ );
        return  -ENODEV;
    }
    CDBG("[NFC]%s step 2...\n", __func__ );

    if( !i2c_check_functionality( client->adapter, I2C_FUNC_I2C ))
    {
        pr_err( "%s : need I2C_FUNC_I2C\n", __func__ );
        return  -ENODEV;
    }
    CDBG("[NFC]%s step 3...\n", __func__ );
	
    pn553_dev = kzalloc( sizeof( *pn553_dev ), GFP_KERNEL );
    if( pn553_dev == NULL )
    {
        dev_err( &client->dev, "failed to allocate memory for module data\n");
        ret = -ENOMEM;
        goto err_exit;
    }
    CDBG("[NFC]%s step 4...\n", __func__ );
	
    pn553_dev->irq_gpio   = platform_data->irq_gpio;
    pn553_dev->ven_gpio   = platform_data->ven_gpio;
    pn553_dev->firm_gpio  = platform_data->firm_gpio;
    pn553_dev->client     = client;

  /* init mutex and queues */
    init_waitqueue_head( &pn553_dev->read_wq );
    mutex_init( &pn553_dev->read_mutex );
    spin_lock_init( &pn553_dev->irq_enabled_lock );

    pn553_dev->pn553_device.minor = MISC_DYNAMIC_MINOR;
    pn553_dev->pn553_device.name  = NFC_DEV_NAME;
    pn553_dev->pn553_device.fops  = &pn553_dev_fops;
    ret = misc_register( &pn553_dev->pn553_device );
    if( ret )
    {
        pr_err("%s : misc_register failed\n", __FILE__);
        goto err_misc_register;
    }
    CDBG("[NFC]%s step 5...\n", __func__ );

#if defined( I2C_DMA_USAGE )
    if( I2CDMAWriteBuf == NULL )
    {
	    #ifdef CONFIG_64BIT    
	    I2CDMAWriteBuf = (char *)dma_alloc_coherent(&client->dev, MAX_BUFFER_SIZE, (dma_addr_t *)&I2CDMAWriteBuf_pa, GFP_KERNEL);
        #else
        I2CDMAWriteBuf = (char *)dma_alloc_coherent( NULL, MAX_BUFFER_SIZE, &I2CDMAWriteBuf_pa, GFP_KERNEL );
	    #endif
        if( I2CDMAWriteBuf == NULL )
        {
            pr_err("%s : failed to allocate dma write buffer\n", __func__ );
            goto err_request_irq_failed;
        }
    }
    CDBG("[NFC]%s step 5-1...\n", __func__ );

    if( I2CDMAReadBuf == NULL )
    {
	    #ifdef CONFIG_64BIT 	
	    I2CDMAReadBuf = (char *)dma_alloc_coherent(&client->dev, MAX_BUFFER_SIZE, (dma_addr_t *)&I2CDMAReadBuf_pa, GFP_KERNEL);
	    #else
        I2CDMAReadBuf = (char *)dma_alloc_coherent( NULL, MAX_BUFFER_SIZE, (dma_addr_t *)&I2CDMAReadBuf_pa, GFP_KERNEL );
	    #endif
        if( I2CDMAReadBuf == NULL )
        {
            pr_err("%s : failed to allocate dma read buffer\n", __func__ );
            goto err_request_irq_failed;
        }
    }
    CDBG("[NFC]%s step 5-2...\n", __func__ );
#endif /* End.. (I2C_DMA_USAGE) */

	node = of_find_compatible_node(NULL, NULL, "mediatek,nfc-gpio-v2");
	if (node) {
		nfc_irq = irq_of_parse_and_map(node, 0);
		client->irq = nfc_irq;
		ret =request_irq(nfc_irq, pn553_dev_irq_handler,IRQF_TRIGGER_HIGH, "irq_nfc-eint", NULL);
		if (ret) {
			pr_err("%s : EINT IRQ LINE NOT AVAILABLE\n", __func__);
			goto err_request_irq_failed;
		} else {
			pr_err("%s : set EINT finished, nfc_irq=%d\n", __func__,nfc_irq);
			pn553_dev->irq_enabled = true;
            pn553_disable_irq( pn553_dev );
		}
	} else {
		pr_err("%s : can not find NFC eint compatible node\n",__func__);
	}
	
	CDBG("[NFC]%s step 6...\n", __func__ );
    i2c_set_clientdata( client, pn553_dev );
	//>2016/02/04-JackHu--B[All][Main][NFC][DMS07217345] To enhance the message handling in suspend mode
    wake_lock_init(&pn553_dev->wake_lock, WAKE_LOCK_SUSPEND, "pn553_nfc_read");
    //>2016/02/04-JackHu--B[All][Main][NFC][DMS07217345] To enhance the message handling in suspend mode
    CDBG("[NFC]%s Success...\n", __func__ );
	g_pn553_dev = pn553_dev;
	if(g_pn553_dev->client==NULL)
	    pr_err("%s : g_pn553_dev->client==NULL  NFC\n", __func__);
	CDBG("[NFC]%s check\n", __func__ );
	g_pn553_dev->irq_gpio=pn553_pdata.irq_gpio;
	g_pn553_dev->ven_gpio=pn553_pdata.ven_gpio;
	g_pn553_dev->firm_gpio=pn553_pdata.firm_gpio;
	pr_err("%s : g_pn553_dev.ven:%d, firm:%d, irq:%d  NFC\n", __func__,g_pn553_dev->ven_gpio,g_pn553_dev->firm_gpio,g_pn553_dev->irq_gpio);
	pr_err("%s : g_pn553_dev.irq enabled:%d  NFC\n", __func__,g_pn553_dev->irq_enabled);
    return 0;
err_request_irq_failed:
    misc_deregister( &pn553_dev->pn553_device );
err_misc_register:
    mutex_destroy( &pn553_dev->read_mutex );
//    mutex_destroy( &pn553_dev->rw_mutex );
err_exit:
    kfree( pn553_dev );
    //>2016/02/04-JackHu--B[All][Main][NFC][DMS07217345] To enhance the message handling in suspend mode
    wake_lock_destroy(&pn553_dev->wake_lock);
    //>2016/02/04-JackHu--B[All][Main][NFC][DMS07217345] To enhance the message handling in suspend mode

    pr_err("[NFC]%s some error...\n", __func__ );
    return  ret;
}

/*****************************************************************************
**
******************************************************************************/
static int pn553_remove(struct i2c_client *client)
{
    PN553_DEV *pn553_dev;
    pr_err("[NFC]%s: Enter...\n", __func__ );
    pn553_dev = i2c_get_clientdata( client );

#if defined( I2C_DMA_USAGE )
    if( I2CDMAWriteBuf )
    {
	    #ifdef CONFIG_64BIT 	
	    dma_free_coherent(&client->dev, MAX_BUFFER_SIZE, I2CDMAWriteBuf, I2CDMAWriteBuf_pa);
	    #else
        dma_free_coherent( NULL, MAX_BUFFER_SIZE, I2CDMAWriteBuf, I2CDMAWriteBuf_pa );
	    #endif
        I2CDMAWriteBuf    = NULL;
        I2CDMAWriteBuf_pa = 0;
    }

    if( I2CDMAReadBuf )
    {
	    #ifdef CONFIG_64BIT
	    dma_free_coherent(&client->dev, MAX_BUFFER_SIZE, I2CDMAReadBuf, I2CDMAReadBuf_pa);
	    #else
        dma_free_coherent( NULL, MAX_BUFFER_SIZE, I2CDMAReadBuf, I2CDMAReadBuf_pa );
	    #endif
        I2CDMAReadBuf     = NULL;
        I2CDMAReadBuf_pa  = 0;
    }
#endif /* End.. (I2C_DMA_USAGE) */
    //>2016/02/04-JackHu--B[All][Main][NFC][DMS07217345] To enhance the message handling in suspend mode
    wake_lock_destroy(&pn553_dev->wake_lock);
    //free_irq( client->irq, pn553_dev );
	//>2016/02/04-JackHu--B[All][Main][NFC][DMS07217345] To enhance the message handling in suspend mode
    misc_deregister( &pn553_dev->pn553_device );
    mutex_destroy( &pn553_dev->read_mutex );
    kfree( pn553_dev );
    return 0;
}

/*****************************************************************************
**
******************************************************************************/
/* i2c_register_board_info will be not to do over here */


static struct of_device_id pn553_match_table[] = {
    { .compatible = "mediatek,NFC" },
    {}
};

static const struct i2c_device_id pn553_id[] = {
    { "pn553", 0},
    {}
};

static struct i2c_driver pn553_driver = {
    .id_table = pn553_id,
    .probe    = pn553_probe,
    .remove   = pn553_remove,
    .driver   =
    {
      .owner  = THIS_MODULE,
      .name   = "pn553",
      .of_match_table = pn553_match_table,
    },
};

/*
 * module load/unload record keeping
 */

static int __init pn553_dev_init(void)
{
    int vRet = 0;
    pr_err("[NFC] %s: Loading pn553 driver...Jacack\n", __func__ );
#if 0
    i2c_register_board_info( 1, &pn553_i2c_device, ARRAY_SIZE(pn553_i2c_device));
#endif
    platform_driver_register(&nxp_nfc_platform_driver);
    if( i2c_add_driver( &pn553_driver ))
    {
      pr_err("[NFC] PN553 add I2C driver error\n");
      vRet = -1;
    }
    else
    {
      pr_err("[NFC] PN553 add I2C driver success\n");
    }
    return vRet;
}
module_init( pn553_dev_init );

static void __exit pn553_dev_exit(void)
{
    pr_err("[NFC] Unloading pn553 driver\n");
    i2c_del_driver( &pn553_driver );
}
module_exit( pn553_dev_exit );

MODULE_AUTHOR("Sylvain Fonteneau");
MODULE_DESCRIPTION("NFC PN553 driver");
MODULE_LICENSE("GPL");
