
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/time.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include "kd_camera_hw.h"
#include <cust_gpio_usage.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/xlog.h>
#include <linux/version.h>
#ifdef CONFIG_COMPAT
#include <linux/fs.h>
#include <linux/compat.h>
#endif
#include "kd_flashlight.h"
/******************************************************************************
 * Debug configuration
******************************************************************************/
// availible parameter
// ANDROID_LOG_ASSERT
// ANDROID_LOG_ERROR
// ANDROID_LOG_WARNING
// ANDROID_LOG_INFO
// ANDROID_LOG_DEBUG
// ANDROID_LOG_VERBOSE
#define TAG_NAME "sub_strobe.c"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    xlog_printk(ANDROID_LOG_DEBUG  , TAG_NAME, KERN_INFO  "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_WARN(fmt, arg...)        xlog_printk(ANDROID_LOG_WARNING, TAG_NAME, KERN_WARNING  "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_NOTICE(fmt, arg...)      xlog_printk(ANDROID_LOG_DEBUG  , TAG_NAME, KERN_NOTICE  "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_INFO(fmt, arg...)        xlog_printk(ANDROID_LOG_INFO   , TAG_NAME, KERN_INFO  "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_TRC_FUNC(f)              xlog_printk(ANDROID_LOG_DEBUG  , TAG_NAME,  "<%s>\n", __FUNCTION__);
#define PK_TRC_VERBOSE(fmt, arg...) xlog_printk(ANDROID_LOG_VERBOSE, TAG_NAME,  fmt, ##arg)
#define PK_ERROR(fmt, arg...)       xlog_printk(ANDROID_LOG_ERROR  , TAG_NAME, KERN_ERR "%s: " fmt, __FUNCTION__ ,##arg)


#define DEBUG_LEDS_STROBE
#ifdef  DEBUG_LEDS_STROBE
	#define PK_DBG PK_DBG_FUNC
	#define PK_VER PK_TRC_VERBOSE
	#define PK_ERR PK_ERROR
#else
	#define PK_DBG(a,...)
	#define PK_VER(a,...)
	#define PK_ERR(a,...)
#endif

static DEFINE_SPINLOCK(g_strobeSMPLock_sub); /* cotta-- SMP proection */

static u32 strobe_Res = 0;
static u32 strobe_Timeus = 0;
static BOOL g_strobe_On = 0;

static int gDuty=0;
static int g_timeOutTimeMs=0;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37))
static DEFINE_MUTEX(g_strobeSem_sub);
#else
static DECLARE_MUTEX(g_strobeSem_sub);
#endif

#define STROBE_DEVICE_ID 0xCE


static struct work_struct workTimeOut_sub;

#define FLASH_GPIO_ENE   (GPIO118 | 0x80000000)
#define FLASH_GPIO_ENS	 (GPIO122 | 0x80000000)
#define FLASH_GPIO_ENT  (GPIO119  | 0x80000000)

/*****************************************************************************
Functions
*****************************************************************************/
extern int iWriteReg_led(u8 a_u2Addr , u8 a_u4Data , u8 a_u4Bytes , u16 i2cId);
extern int iReadRegI2C_led(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
static void work_timeOutFunc(struct work_struct *data);
extern int kdSetI2CBusNum(u32 i2cBusNum);


static int FL_Enable(void)
{
	char buf[2];
	char bufR[2];
	int i = 0;

	PK_DBG("FL_enable sub :g_duty=%d \n",gDuty);
	PK_DBG("FL_enable sub :strobe_device_id =%d \n",i);
	mt_set_gpio_out(FLASH_GPIO_ENE,GPIO_OUT_ONE);
	mdelay(20);
	//kdSetI2CBusNum(1);	
	buf[0]=0;
	iReadRegI2C_led(buf, 1, bufR, 1, STROBE_DEVICE_ID);
	PK_DBG(" FL_Enable  LED124  line=%d reg0 %x\n",__LINE__, (int)bufR[0]);


	if(gDuty == 0)
	{
		iWriteReg_led(0x05, 0x30, 1, STROBE_DEVICE_ID);	
		iWriteReg_led(0x06, 0x7F, 1, STROBE_DEVICE_ID);
		iWriteReg_led(0x07, 0x7F, 1, STROBE_DEVICE_ID);		
		iWriteReg_led(0x01, 0x02, 1, STROBE_DEVICE_ID);            //torch	
	}else if (gDuty == 1){
		iWriteReg_led(0x05, 0x0, 1, STROBE_DEVICE_ID);	
		iWriteReg_led(0x06, 0x7F, 1, STROBE_DEVICE_ID);
		iWriteReg_led(0x07, 0x7F, 1, STROBE_DEVICE_ID);		
		iWriteReg_led(0x01, 0x02, 1, STROBE_DEVICE_ID);            //torch		
	}else {
		iWriteReg_led(0x04, 0x47, 1, STROBE_DEVICE_ID);//20141208
		iWriteReg_led(0x05, gDuty, 1, STROBE_DEVICE_ID);	
		iWriteReg_led(0x06, 0x7F, 1, STROBE_DEVICE_ID);
		iWriteReg_led(0x07, 0x7F, 1, STROBE_DEVICE_ID);		
		iWriteReg_led(0x01, 0x03, 1, STROBE_DEVICE_ID);            //flash			
	}

	//kdSetI2CBusNum(3);
    return 0;
}



static int FL_Disable(void)
{
	char buf[2];
	char bufR[2];

	mt_set_gpio_out(FLASH_GPIO_ENE,GPIO_OUT_ONE);
	mdelay(20);
	iWriteReg_led(0x01, 0x00, 1, STROBE_DEVICE_ID);


	mt_set_gpio_out(FLASH_GPIO_ENE,GPIO_OUT_ZERO);
	mt_set_gpio_out(FLASH_GPIO_ENT,GPIO_OUT_ZERO);
	PK_DBG(" FL_Disable line=%d\n",__LINE__);
    return 0;
}

static int FL_dim_duty(kal_uint32 duty)
{
	PK_DBG(" FL_dim_duty line=%d\n",__LINE__);
	gDuty =  duty;
    return 0;
}


static int FL_Init(void)
{
    if(mt_set_gpio_mode(FLASH_GPIO_ENT,GPIO_MODE_00)){PK_DBG("[constant_flashlight] set gpio mode failed!! \n");}
    if(mt_set_gpio_dir(FLASH_GPIO_ENT,GPIO_DIR_OUT)){PK_DBG("[constant_flashlight] set gpio dir failed!! \n");}
    if(mt_set_gpio_out(FLASH_GPIO_ENT,GPIO_OUT_ZERO)){PK_DBG("[constant_flashlight] set gpio failed!! \n");}

    if(mt_set_gpio_mode(FLASH_GPIO_ENE,GPIO_MODE_00)){PK_DBG("[constant_flashlight] set gpio mode failed!! \n");}
    if(mt_set_gpio_dir(FLASH_GPIO_ENE,GPIO_DIR_OUT)){PK_DBG("[constant_flashlight] set gpio dir failed!! \n");}
    if(mt_set_gpio_out(FLASH_GPIO_ENE,GPIO_OUT_ZERO)){PK_DBG("[constant_flashlight] set gpio failed!! \n");}


    if(mt_set_gpio_mode(FLASH_GPIO_ENS,GPIO_MODE_00)){PK_DBG("[constant_flashlight] set gpio mode failed!! \n");}
    if(mt_set_gpio_dir(FLASH_GPIO_ENS,GPIO_DIR_OUT)){PK_DBG("[constant_flashlight] set gpio dir failed!! \n");}
    if(mt_set_gpio_out(FLASH_GPIO_ENS,GPIO_OUT_ZERO)){PK_DBG("[constant_flashlight] set gpio failed!! \n");}


    PK_DBG(" FL_Init line=%d\n",__LINE__);
    return 0;
}


static int FL_Uninit(void)
{
	FL_Disable();
    return 0;
}

/*****************************************************************************
User interface
*****************************************************************************/

static void work_timeOutFunc(struct work_struct *data)
{
    FL_Disable();
    PK_DBG("ledTimeOut_callback\n");
    //printk(KERN_ALERT "work handler function./n");
}



static enum hrtimer_restart ledTimeOutCallback(struct hrtimer *timer)
{
    schedule_work(&workTimeOut_sub);
    return HRTIMER_NORESTART;
}
static struct hrtimer g_timeOutTimer;
static void timerInit(void)
{
  	INIT_WORK(&workTimeOut_sub, work_timeOutFunc);
	g_timeOutTimeMs=1000; //1s
	hrtimer_init( &g_timeOutTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL );
	g_timeOutTimer.function=ledTimeOutCallback;

}



static int sub_strobe_ioctl(unsigned int  cmd, unsigned long  arg)
{
	PK_DBG("sub dummy ioctl");


	int i4RetValue = 0;
	int ior_shift;
	int iow_shift;
	int iowr_shift;
	ior_shift = cmd - (_IOR(FLASHLIGHT_MAGIC,0, int));
	iow_shift = cmd - (_IOW(FLASHLIGHT_MAGIC,0, int));
	iowr_shift = cmd - (_IOWR(FLASHLIGHT_MAGIC,0, int));
	PK_DBG("RT4505 constant_flashlight_ioctl() line=%d ior_shift=%d, iow_shift=%d iowr_shift=%d arg=%d\n",__LINE__, ior_shift, iow_shift, iowr_shift, arg);
	switch(cmd)
	{

	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		PK_DBG("FLASH_IOC_SET_TIME_OUT_TIME_MS: %d\n",arg);
		g_timeOutTimeMs=arg;
	break;


	case FLASH_IOC_SET_DUTY :
		PK_DBG("FLASHLIGHT_DUTY: %d\n",arg);
		FL_dim_duty(arg);
		break;


	case FLASH_IOC_SET_STEP:
		PK_DBG("FLASH_IOC_SET_STEP: %d\n",arg);

		break;

	case FLASH_IOC_SET_ONOFF :
		PK_DBG("FLASHLIGHT_ONOFF: %d\n",arg);
		if(arg==1)
		{
			if(g_timeOutTimeMs!=0)
	        {
	        	ktime_t ktime;
				ktime = ktime_set( 0, g_timeOutTimeMs*1000000 );
				hrtimer_start( &g_timeOutTimer, ktime, HRTIMER_MODE_REL );
	        }
			FL_Enable();
		}
		else
		{
			FL_Disable();
			hrtimer_cancel( &g_timeOutTimer );
		}
		break;
	default :
		PK_DBG(" No such command \n");
		i4RetValue = -EPERM;
		break;
	}
	return i4RetValue;

}

static int sub_strobe_open(void *pArg)
{
    PK_DBG("sub dummy open");

	int i4RetValue = 0;
	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

	if (0 == strobe_Res)
	{
	FL_Init();
	timerInit();
	}
	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);
	spin_lock_irq(&g_strobeSMPLock_sub);


	if(strobe_Res)
	{
	PK_ERR(" busy!\n");
	i4RetValue = -EBUSY;
	}
	else
	{
	strobe_Res += 1;
	}


	spin_unlock_irq(&g_strobeSMPLock_sub);

	return i4RetValue;

}

static int sub_strobe_release(void *pArg)
{
    PK_DBG("sub dummy release");
	
    if (strobe_Res)
    {
        spin_lock_irq(&g_strobeSMPLock_sub);

        strobe_Res = 0;
        strobe_Timeus = 0;

        /* LED On Status */
        g_strobe_On = FALSE;

        spin_unlock_irq(&g_strobeSMPLock_sub);

    	FL_Uninit();
    }
	
    return 0;

}

FLASHLIGHT_FUNCTION_STRUCT	subStrobeFunc=
{
	sub_strobe_open,
	sub_strobe_release,
	sub_strobe_ioctl
};


MUINT32 subStrobeInit(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
    if (pfFunc != NULL)
    {
        *pfFunc = &subStrobeFunc;
    }
    return 0;
}





