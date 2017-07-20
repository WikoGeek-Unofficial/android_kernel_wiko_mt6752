/* 
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>

#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#include <mach/irqs.h>
#include <mach/eint.h>

#include <asm/io.h>
#include <cust_eint.h>
#include "bu52031nvx.h"
/******************************************************************************
 * configuration
*******************************************************************************/
/*----------------------------------------------------------------------------*/
#define APS_TAG                  "[BU52031NVX] "
#define APS_FUN(f)               printk(KERN_INFO APS_TAG"%s\n", __FUNCTION__)
#define APS_ERR(fmt, args...)    printk(KERN_ERR  APS_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define APS_LOG(fmt, args...)    printk(KERN_ERR APS_TAG fmt, ##args)
#define APS_DBG(fmt, args...)    printk(KERN_INFO APS_TAG fmt, ##args)         

struct platform_device sensor_hall = {
	.name	       = "hall",
	.id            = -1,
};
static	struct work_struct	eint_work;
static struct input_dev *bu52031nvx_key_dev;
extern int gBackLightLevel;
//#define KEY_SENSOR 239
#define KEY_SENSOR 116    
#define KEY_Power_SENSOR 116  //power key
static int enable_input_key = 1;
static s32 value_hall1_rev = 1;
static s32 value_hall2_rev = 1;
static s32 hall_state = 1; //open status
struct mutex mutex_power_key;
static int bu52031nvx_probe(struct platform_device *pdev);
static int bu52031nvx_remove(struct platform_device *pdev);
/******************************************************************************
 * extern functions
*******************************************************************************/
extern void mt_eint_mask(unsigned int eint_num);
extern void mt_eint_unmask(unsigned int eint_num);
extern void mt_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
extern void mt_eint_set_polarity(unsigned int eint_num, unsigned int pol);
extern unsigned int mt_eint_set_sens(unsigned int eint_num, unsigned int sens);
extern void mt_eint_registration(unsigned int eint_num, unsigned int flow, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);
extern void mt_eint_print_status(void);


/*-------------------------------attribute file for debugging----------------------------------*/

/******************************************************************************
 * Sysfs attributes
*******************************************************************************/
/*----------------------------------------------------------------------------*/
static ssize_t bu52031nvx_show_state(struct device_driver *ddri, char *buf)
{
	APS_LOG("hall_state = %d\n", hall_state);
	return snprintf(buf, PAGE_SIZE, "%d\n", hall_state);
}
/*----------------------------------------------------------------------------*/
static ssize_t bu52031nvx_show_key(struct device_driver *ddri, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", enable_input_key);
}
/*----------------------------------------------------------------------------*/
static ssize_t bu52031nvx_store_key(struct device_driver *ddri, const char *buf, size_t count)
{
    int enable, res;
    u8 databuf[1];
	
	if(1 == sscanf(buf, "%d", &enable))
	{
		enable_input_key = enable;
	}
	else 
	{
		APS_ERR("invalid enable content: '%s', length = %d\n", buf, count);
	}
	return count;    
}
/*---------------------------------------------------------------------------------------*/
static DRIVER_ATTR(hall_state,     S_IWUSR | S_IRUGO, bu52031nvx_show_state, NULL);
static DRIVER_ATTR(input_key_enable,    0666, bu52031nvx_show_key, bu52031nvx_store_key);
/*----------------------------------------------------------------------------*/
static struct driver_attribute *bu52031nvx_attr_list[] = {
    &driver_attr_hall_state,
    &driver_attr_input_key_enable,
};

/*----------------------------------------------------------------------------*/
static int bu52031nvx_create_attr(struct device_driver *driver) 
{
	int idx, err = 0;
	int num = (int)(sizeof(bu52031nvx_attr_list)/sizeof(bu52031nvx_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if((err = driver_create_file(driver, bu52031nvx_attr_list[idx])))
		{            
			APS_ERR("driver_create_file (%s) = %d\n", bu52031nvx_attr_list[idx]->attr.name, err);
			break;
		}
	}    
	return err;
}
/*----------------------------------------------------------------------------*/
	static int bu52031nvx_delete_attr(struct device_driver *driver)
	{
	int idx ,err = 0;
	int num = (int)(sizeof(bu52031nvx_attr_list)/sizeof(bu52031nvx_attr_list[0]));

	if (!driver)
	return -EINVAL;

	for (idx = 0; idx < num; idx++) 
	{
		driver_remove_file(driver, bu52031nvx_attr_list[idx]);
	}
	
	return err;
}
/*----------------------------------------------------------------------------*/
#ifdef CONFIG_OF
static const struct of_device_id hall_of_match[] = {
	{ .compatible = "mediatek,hall", },
	{},
};
#endif
static struct platform_driver bu52031nvx_hall_driver = {
	.probe      = bu52031nvx_probe,
	.remove     = bu52031nvx_remove,    
	.driver     = {
		.name  = "hall",
        #ifdef CONFIG_OF
		.of_match_table = hall_of_match,
	#endif
	}
};

/*----------------------------------------------------------------------------*/
static void bu52031nvx_eint1_func(void)
{

	APS_ERR("debug bu52031nvx_eint1_func!");
	schedule_work(&eint_work);
}

static void bu52031nvx_eint2_func(void)
{

	APS_ERR("debug bu52031nvx_eint2_func!");
	schedule_work(&eint_work);
}


/*----------------------------------interrupt functions--------------------------------*/
static void bu52031nvx_eint_work(struct work_struct *work)
{
	int res = 0;
	s32 value_hall1 = -1;
	s32 value_hall2 = -1;
	
	value_hall1 = mt_get_gpio_in(GPIO_HALL_1_PIN);
	APS_LOG("BU52031NVX_eint_work GPIO_HALL_1_PIN=%d\n",value_hall1);	
	if(!value_hall1)
		mt_eint_registration(CUST_EINT_HALL_1_NUM, EINTF_TRIGGER_HIGH, bu52031nvx_eint1_func, 0);
	else
		mt_eint_registration(CUST_EINT_HALL_1_NUM, EINTF_TRIGGER_LOW, bu52031nvx_eint1_func, 0);
/*
	value_hall2 = mt_get_gpio_in(GPIO_HALL_2_PIN);
	APS_LOG("BU52031NVX_eint_work GPIO_HALL_2_PIN=%d\n",value_hall2);	
	if(!value_hall2)
		mt_eint_registration(CUST_EINT_HALL_2_NUM, EINTF_TRIGGER_HIGH, bu52031nvx_eint2_func, 0);
	else
		mt_eint_registration(CUST_EINT_HALL_2_NUM, EINTF_TRIGGER_LOW, bu52031nvx_eint2_func, 0);
	*/
	if(0 == value_hall1)//||(0 == value_hall2))
		hall_state = 0;
	else
		hall_state = 1;		
		
	APS_LOG("===========BU52031NVX_eint_work gBackLightLevel=%d enable_input_key==========%d\n",gBackLightLevel, enable_input_key);	
	APS_LOG("============BU52031NVX_eint_work value_hall1_rev=%d value_hall1===============%d\n",value_hall1_rev, value_hall1);	
	APS_LOG("============BU52031NVX_eint_work value_hall2_rev=%d value_hall2===============%d\n",value_hall2_rev, value_hall2);	

	if(enable_input_key){
		if(value_hall1_rev != value_hall1)
                {
		APS_LOG("===========BU52031NVX_eint_work gBackLightLevel=%d enable_input_key=====1=====%d\n",gBackLightLevel, enable_input_key);	
		mutex_lock(&mutex_power_key);
		input_report_key(bu52031nvx_key_dev, KEY_SENSOR, 1);
                input_sync(bu52031nvx_key_dev);
		input_report_key(bu52031nvx_key_dev, KEY_SENSOR, 0);
		input_sync(bu52031nvx_key_dev);
		input_sync(bu52031nvx_key_dev);
		mutex_unlock(&mutex_power_key);
		}
    /*            else if(value_hall1)//||(value_hall2&&(0==gBackLightLevel)))
               {
APS_LOG("===========BU52031NVX_eint_work gBackLightLevel=%d enable_input_key=====2=====%d\n",gBackLightLevel, enable_input_key);	
	        input_report_key(bu52031nvx_key_dev, KEY_SENSOR, 1);
                 input_sync(bu52031nvx_key_dev);
		input_report_key(bu52031nvx_key_dev, KEY_SENSOR, 0);
		input_sync(bu52031nvx_key_dev);
	       }
*/
		value_hall1_rev = value_hall1;
		value_hall2_rev = value_hall2;
	}
	
	mt_eint_unmask(CUST_EINT_HALL_1_NUM); 
	//mt_eint_unmask(CUST_EINT_HALL_2_NUM); 
	return;
}

int bu52031nvx_setup_eint()
{
	
	mt_set_gpio_dir(GPIO_HALL_1_PIN, GPIO_DIR_IN);
	mt_set_gpio_mode(GPIO_HALL_1_PIN, GPIO_HALL_1_PIN_M_EINT);
	mt_set_gpio_pull_enable(GPIO_HALL_1_PIN, TRUE);
	mt_set_gpio_pull_select(GPIO_HALL_1_PIN, GPIO_PULL_UP);


/*	mt_set_gpio_dir(GPIO_HALL_2_PIN, GPIO_DIR_IN);
	mt_set_gpio_mode(GPIO_HALL_2_PIN, GPIO_HALL_2_PIN_M_EINT);
	mt_set_gpio_pull_enable(GPIO_HALL_2_PIN, TRUE);
	mt_set_gpio_pull_select(GPIO_HALL_2_PIN, GPIO_PULL_UP);
*/
	mt_eint_set_sens(CUST_EINT_HALL_1_NUM, MT_LEVEL_SENSITIVE);
	mt_eint_set_hw_debounce(CUST_EINT_HALL_1_NUM, CUST_EINT_HALL_1_DEBOUNCE_CN);
	mt_eint_registration(CUST_EINT_HALL_1_NUM, EINTF_TRIGGER_LOW, bu52031nvx_eint1_func, 0);
	mt_eint_unmask(CUST_EINT_HALL_1_NUM); 
/*	
	mt_eint_set_sens(CUST_EINT_HALL_2_NUM, MT_LEVEL_SENSITIVE);
	mt_eint_set_hw_debounce(CUST_EINT_HALL_2_NUM, CUST_EINT_HALL_2_DEBOUNCE_CN);
	mt_eint_registration(CUST_EINT_HALL_2_NUM, EINTF_TRIGGER_LOW, bu52031nvx_eint2_func, 0);
	mt_eint_unmask(CUST_EINT_HALL_2_NUM); 
   */
  	return 0;
}

/*----------------------------------------------------------------------------*/

static int bu52031nvx_probe(struct platform_device *pdev) 
{
	APS_FUN();  
	int err;

	mutex_init(&mutex_power_key);  
	INIT_WORK(&eint_work, bu52031nvx_eint_work);
	bu52031nvx_setup_eint();
	
	if((err = bu52031nvx_create_attr(&bu52031nvx_hall_driver.driver)))
	{
		APS_ERR("create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}

	//------------------------------------------------------------------
	// 							Create input device 
	//------------------------------------------------------------------
	bu52031nvx_key_dev = input_allocate_device();
	if (!bu52031nvx_key_dev) 
	{
		APS_LOG("[APS]bu52031nvx_key_dev : fail!\n");
		return -ENOMEM;
	}
        
	//define multi-key keycode
	__set_bit(EV_KEY, bu52031nvx_key_dev->evbit);

        input_set_capability(bu52031nvx_key_dev, EV_KEY, KEY_POWER);
        input_set_capability(bu52031nvx_key_dev, EV_KEY, KEY_SENSOR);
	
	bu52031nvx_key_dev->id.bustype = BUS_HOST;
	bu52031nvx_key_dev->name = "BU52031NVX";
	if(input_register_device(bu52031nvx_key_dev))
	{
		APS_LOG("[APS]bu52031nvx_key_dev register : fail!\n");
	}else
	{
		APS_LOG("[APS]bu52031nvx_key_dev register : success!!\n");
	} 
	
	return 0;
	
	exit_create_attr_failed:
	APS_ERR("%s: err = %d\n", __func__, err);
	return err;
}
/*----------------------------------------------------------------------------*/
static int bu52031nvx_remove(struct platform_device *pdev)
{
	
	APS_FUN(); 
	int err;	
	
	input_unregister_device(bu52031nvx_key_dev);
	if((err = bu52031nvx_delete_attr(&bu52031nvx_hall_driver.driver)))
	{
		APS_ERR("bu52031nvx_delete_attr fail: %d\n", err);
	} 
		
	return 0;
}



/*----------------------------------------------------------------------------*/
static int __init bu52031nvx_init(void)
{
	int retval = 0;
	printk("============bu52031nvx_init========================");
	APS_FUN();
#if defined(CUSTOM_KERNEL_HALL)
	retval = platform_device_register(&sensor_hall);
		printk("sensor_hall device!");
	if (retval != 0)
		return retval;
#endif
  if(platform_driver_register(&bu52031nvx_hall_driver))
	{
		APS_ERR("failed to register driver");
		return -ENODEV;
	}
	return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit bu52031nvx_exit(void)
{
	APS_FUN();
	platform_driver_unregister(&bu52031nvx_hall_driver);
}
/*----------------------------------------------------------------------------*/
module_init(bu52031nvx_init);
module_exit(bu52031nvx_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("aijun.zhu");
MODULE_DESCRIPTION("abu52031nvx driver");
MODULE_LICENSE("GPL");

