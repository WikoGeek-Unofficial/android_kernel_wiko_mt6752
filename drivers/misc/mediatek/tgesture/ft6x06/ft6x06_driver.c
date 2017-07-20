 #include "tpd.h"
#include <linux/interrupt.h>
#include <cust_eint.h>
#include <linux/i2c.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/rtpm_prio.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>

#include "tpd_custom_ft6x06.h"

#include <mach/mt_pm_ldo.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_boot.h>

#include <linux/jiffies.h>

#include <linux/slab.h>

#if defined(MT6577)
#include <mach/mt_pm_ldo.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_boot.h>
#elif defined(MT6575)
#include <mach/mt6575_pm_ldo.h>
#include <mach/mt6575_typedefs.h>
#include <mach/mt6575_boot.h>
#elif defined(CONFIG_ARCH_MT6573)
#include <mach/mt6573_boot.h>
#endif

#include "cust_gpio_usage.h"
#include "focaltech_ctl.h"


//LINE<touch panel><date20131021><tp auto update>yinhuiyong
#define  FTS_AUTO_TP_UPGRADE
#ifdef  FTS_AUTO_TP_UPGRADE
#include "ftbin_HRC.h"
#include "ftbin_yeji_ft6x06.h"//LINE <ft6x06> <DATE20140910> <modify for ft6336> yolo
#include "ftbin_yeji_ft6336.h"
#include "ftbin_jiemian.h"
static uint8_t file_FT6406_fw_data[] = {
	#include "FT6406_V01_20141222_app.i"
};
#endif

#define FTS_SUPPORT_TRACK_ID

struct tpd_device  *back_ctp = 0;

static DECLARE_WAIT_QUEUE_HEAD(waiter);
static void tpd_eint_interrupt_handler(void);
extern void mt_eint_unmask(unsigned int line);
extern void mt_eint_mask(unsigned int line);
extern void mt_eint_registration(unsigned int eint_num, unsigned int flag, 
              void (EINT_FUNC_PTR) (void), unsigned int is_auto_umask);

#if defined (TINNO_FULL_TOUCH_SUPPORT)
extern void tpded_register_sub(struct input_dev *dev, int xMax, int yMax, int (*dev_enable)(int ));
extern void tpded_set_event_sub(/*struct input_dev *dev, */int x, int y, int pressure, int trackID, int isPress, int isSync);
#endif  /* TINNO_FULL_TOUCH_SUPPORT */

#define APS_ERR(fmt, args...)    printk(KERN_ERR  "%d : "fmt, __FUNCTION__, __LINE__, ##args)

static int tpd_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tpd_detect(struct i2c_client *client, struct i2c_board_info *info);
static int tpd_remove(struct i2c_client *client);
static int touch_event_handler(void *unused);

#ifdef TPD_HAVE_BUTTON 

static int tpd_keys_local[TPD_KEY_COUNT] = TPD_KEYS;
static int tpd_keys_dim_local_BYD[TPD_KEY_COUNT][4] = TPD_KEYS_DIM_BYD;
static int tpd_keys_dim_local_NB[TPD_KEY_COUNT][4] = TPD_KEYS_DIM_NB;

#if defined (TINNO_FULL_TOUCH_SUPPORT)
static kal_uint8 g_tp_enable = 0;
static int ft6X06_enable(int value);
#endif  /* TINNO_FULL_TOUCH_SUPPORT */
//BEGIN <add changing flag> <DATE20130330> <add changing flag> zhangxiaofei
kal_uint8 g_pre_tp_charger_flag = 0;
kal_uint8 g_tp_charger_flag = 0;
//END <add changing flag> <DATE20130330> <add changing flag> zhangxiaofei

static s32 tp_state = -1; 

static void tinno_update_tp_button_dim(int panel_vendor)
{
	if ( FTS_CTP_VENDOR_NANBO == panel_vendor ){
		tpd_button_setting(TPD_KEY_COUNT, tpd_keys_local, tpd_keys_dim_local_NB);
	}else{
		tpd_button_setting(TPD_KEY_COUNT, tpd_keys_local, tpd_keys_dim_local_BYD);
	}
}
 
#endif

extern char tpd_desc[50];

static tinno_ts_data *g_pts = NULL;
static volatile	int tpd_flag;

#define DRIVER_NAME "ft6x06"	

#define GESTURE_X_MAX 240
#define GESTURE_Y_MAX 320


#define BACKCTP_DRIVER_NAME "tgesture-back"   

static const struct i2c_device_id ft6x06_backctp_id[] = {{DRIVER_NAME,0},{}};

static struct i2c_board_info __initdata ft6x06_i2c_backctp[]={{I2C_BOARD_INFO(DRIVER_NAME, TPD_I2C_SLAVE_ADDR1)}};		

static struct i2c_driver backctp_i2c_driver = {
	.driver = {
		 .name = DRIVER_NAME,
	},
	.probe = tpd_probe,
	.remove = tpd_remove,
	.id_table = ft6x06_backctp_id,
	.detect = tpd_detect,
};

 
#ifdef TINNO_ANDROID_L8420AE
#define GPIO_BACK_CTP_EN         (GPIO146 | 0x80000000)    
#endif

static  void tpd_down(tinno_ts_data *ts, int x, int y, int pressure, int trackID) 
{
    CTP_DBG("x=%03d, y=%03d, pressure=%03d, ID=%03d", x, y, pressure, trackID);
    #if defined (TINNO_FULL_TOUCH_SUPPORT)
    tpded_set_event_sub(GESTURE_X_MAX-x-1, y, pressure, trackID, 1, 1);
    #else
    input_report_abs(back_ctp->dev, ABS_PRESSURE, pressure);
    input_report_abs(back_ctp->dev, ABS_MT_PRESSURE, pressure);
    input_report_key(back_ctp->dev, BTN_TOUCH, 1);
    input_report_abs(back_ctp->dev, ABS_MT_POSITION_X, x);
    input_report_abs(back_ctp->dev, ABS_MT_POSITION_Y, y);
    #ifdef FTS_SUPPORT_TRACK_ID
    input_report_abs(back_ctp->dev, ABS_MT_TRACKING_ID, trackID);
    #endif
    input_report_abs(back_ctp->dev, ABS_MT_WIDTH_MAJOR, pressure*pressure/112);
    input_report_abs(back_ctp->dev, ABS_MT_TOUCH_MAJOR, pressure*pressure/112);
    input_mt_sync(back_ctp->dev);
    #endif  /* TINNO_FULL_TOUCH_SUPPORT */
    __set_bit(trackID, &ts->fingers_flag);
    ts->touch_point_pre[trackID].x=x;
    ts->touch_point_pre[trackID].y=y;
    if (FACTORY_BOOT == get_boot_mode() || RECOVERY_BOOT == get_boot_mode()) {   
        tpd_button(x, y, 1);  
    }	 
    TPD_DOWN_DEBUG_TRACK(x,y);
 }
 
static  int tpd_up(tinno_ts_data *ts, int x, int y, int pressure, int trackID) 
{
    CTP_DBG("x=%03d, y=%03d, ID=%03d", x, y, trackID);     
    if (FACTORY_BOOT == get_boot_mode() || RECOVERY_BOOT == get_boot_mode()) {   
        CTP_DBG("x=%03d, y=%03d, ID=%03d", x, y, trackID);
        input_report_abs(back_ctp->dev, ABS_PRESSURE, 0);
        input_report_abs(back_ctp->dev, ABS_MT_PRESSURE, 0);
        input_report_key(back_ctp->dev, BTN_TOUCH, 0);
        input_report_abs(back_ctp->dev, ABS_MT_POSITION_X, x);
        input_report_abs(back_ctp->dev, ABS_MT_POSITION_Y, y);
        #ifdef FTS_SUPPORT_TRACK_ID
        input_report_abs(back_ctp->dev, ABS_MT_TRACKING_ID, trackID);
        #endif
        input_report_abs(back_ctp->dev, ABS_MT_WIDTH_MAJOR, 0);
        input_report_abs(back_ctp->dev, ABS_MT_TOUCH_MAJOR, 0);// This must be placed at the last one.
        input_mt_sync(back_ctp->dev);
    }else{//Android 4.0 don't need to report these up events.
        int i, have_down_cnt = 0;
        for ( i=0; i < TINNO_TOUCH_TRACK_IDS; i++ ){
            if ( test_bit(i, &ts->fingers_flag) ){
                ++have_down_cnt;
            }
        }
        if ( have_down_cnt < 2 ){
            #if defined (TINNO_FULL_TOUCH_SUPPORT)
            tpded_set_event_sub(x, y, pressure, trackID, 0, 1);
            #else
            input_mt_sync(back_ctp->dev);  
            #endif  /* TINNO_FULL_TOUCH_SUPPORT */
        }
        CTP_DBG("x=%03d, y=%03d, ID=%03d, have_down=%d", x, y, trackID, have_down_cnt);
    }

    __clear_bit(trackID, &ts->fingers_flag);
    TPD_UP_DEBUG_TRACK(x,y);
    if (FACTORY_BOOT == get_boot_mode() || RECOVERY_BOOT == get_boot_mode()) {   
        tpd_button(x, y, 0); 
    }   		 
    return 0;
} 
 
 static int tpd_touchinfo(tinno_ts_data *ts, tinno_ts_point *touch_point)
 {
	int i = 0;
	int iInvalidTrackIDs = 0;
	int iTouchID, iSearchDeep;
	fts_report_data_t *pReportData = (fts_report_data_t *)ts->buffer;

	if ( tpd_read_touchinfo(ts) ){
		CTP_DBG("Read touch information error. \n");
		return -EAGAIN; 
	}
	
	if ( 0 != pReportData->device_mode ){
		CTP_DBG("device mode is %d\n", pReportData->device_mode);
		return -EPERM; 
	}
	
	//We need only valid points...
	if ( pReportData->fingers > TINNO_TOUCH_TRACK_IDS ){
		CTP_DBG("fingers is %d\n", pReportData->fingers);
		return -EAGAIN; 
	}

	// For processing gestures.
	if (pReportData->gesture >= 0xF0 && pReportData->gesture <= 0xF3) {
		//fts_6x06_parase_keys(ts, pReportData);
	}	
	iSearchDeep = 0;
#ifdef FTS_SUPPORT_TRACK_ID
	for ( i = 0; i < TINNO_TOUCH_TRACK_IDS; i++ ){
		iSearchDeep += ((pReportData->xy_data[i].event_flag != FTS_EF_RESERVED)?1:0);
	}
#else
	if (pReportData->fingers >= ts->last_fingers ){
		iSearchDeep = pReportData->fingers;
	}else{
		iSearchDeep = ts->last_fingers;
	}
	ts->last_fingers = pReportData->fingers;
#endif

	if ( iSearchDeep ) {
#ifdef FTS_SUPPORT_TRACK_ID
		for ( i=0; i < TINNO_TOUCH_TRACK_IDS; i++ ){
#else
		for ( i=0; i < iSearchDeep; i++ ){
#endif
			if (pReportData->xy_data[i].event_flag != FTS_EF_RESERVED) {
#ifdef FTS_SUPPORT_TRACK_ID
				iTouchID = pReportData->xy_data[i].touch_id;
				if ( iTouchID >= TINNO_TOUCH_TRACK_IDS )
				{
					CTP_DBG("i: Invalied Track ID(%d)\n!", i, iTouchID);
					iInvalidTrackIDs++;
					continue;
				}
#else
				iTouchID = i;
#endif
				touch_point[iTouchID].flag = pReportData->xy_data[i].event_flag;
				touch_point[iTouchID].x = pReportData->xy_data[i].x_h << 8 | pReportData->xy_data[i].x_l;
				touch_point[iTouchID].y = pReportData->xy_data[i].y_h << 8 | pReportData->xy_data[i].y_l;
				touch_point[iTouchID].pressure = pReportData->xy_data[i].pressure;
			}else{
				//CTP_DBG("We got a invalied point, we take it the same as a up event!");
				//CTP_DBG("As it has no valid track ID, we assume it's order is the same as it's layout in the memory!");
				//touch_point[i].flag = FTS_EF_RESERVED;
			}
		}
		if ( TINNO_TOUCH_TRACK_IDS == iInvalidTrackIDs ){
			CTP_DBG("All points are Invalied, Ignore the interrupt!\n");
			return -EAGAIN; 
		}
	}
	
	CTP_DBG("p0_flag=0x%x x0=0x%03x y0=0x%03x pressure0=0x%03x "
	              "p1_flag=0x%x x1=0x%03x y1=0x%03x pressure1=0x%03x "
	              "gesture = 0x%x fingers=0x%x", 
	       touch_point[0].flag, touch_point[0].x, touch_point[0].y, touch_point[0].pressure,
	       touch_point[1].flag, touch_point[1].x, touch_point[1].y, touch_point[1].pressure,
	       pReportData->gesture, pReportData->fingers); 
		  
	 return 0;

 };
 
 static int touch_event_handler(void *para)
 {	 
 	int i;
	tinno_ts_point touch_point[TINNO_TOUCH_TRACK_IDS];
	struct sched_param param = { .sched_priority = RTPM_PRIO_TPD };
	tinno_ts_data *ts = (tinno_ts_data *)para;
	sched_setscheduler(current, SCHED_RR, &param);

	printk("aijun.zhu:touch_event_handler--------->\n");
	
    //END <touch panel> <DATE20130831> <tp proximity> zhangxiaofei
	do {
		set_current_state(TASK_INTERRUPTIBLE); 
		wait_event_interruptible(waiter, tpd_flag!=0);
		tpd_flag = 0;
		memset(touch_point, FTS_INVALID_DATA, sizeof(touch_point));
		set_current_state(TASK_RUNNING); 
		 
        //BEGIN <add changing flag> <DATE20130330> <add changing flag> zhangxiaofei
        if(g_tp_charger_flag != g_pre_tp_charger_flag){
			g_pre_tp_charger_flag = g_tp_charger_flag;
			fts_ft6x06_switch_charger_status(g_tp_charger_flag);
		}
        //END <add changing flag> <DATE20130330> <add changing flag> zhangxiaofei
        
		if (!tpd_touchinfo(ts, &touch_point)) {
			//report muti point then
			for ( i=0; i < TINNO_TOUCH_TRACK_IDS; i++ ){
				if ( FTS_INVALID_DATA != touch_point[i].x ){
					if ( FTS_EF_UP == touch_point[i].flag ){
						if( test_bit(i, &ts->fingers_flag) ){
							tpd_up(ts, ts->touch_point_pre[i].x, ts->touch_point_pre[i].y, 
								touch_point[i].pressure, i);
					}else{
							CTP_DBG("This is a invalid up event.(%d)", i);
						}
					}else{//FTS_EF_CONTACT or FTS_EF_DOWN
						if ( test_bit(i, &ts->fingers_flag) 
							&& (FTS_EF_DOWN == touch_point[i].flag) ){
							CTP_DBG("Ignore a invalid down event.(%d)", i);
							continue;
						}
						tpd_down(ts, touch_point[i].x, touch_point[i].y, 
							touch_point[i].pressure, i);
					}
				}else if (  test_bit(i, &ts->fingers_flag) ){
					CTP_DBG("Complete a invalid down or move event.(%d)", i);
					tpd_up(ts, ts->touch_point_pre[i].x, ts->touch_point_pre[i].y, 
						touch_point[i].pressure, i);
				}
			}
			//input_sync(tpd->dev);
			input_sync(back_ctp->dev);
		}	
		//mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM); 
		mt_eint_unmask(CUST_EINT_GESTURE_PANEL_NUM); 
	}while(!kthread_should_stop());
	//mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM); 
	mt_eint_unmask(CUST_EINT_GESTURE_PANEL_NUM); 
	return 0;
 }
 
static int tpd_detect (struct i2c_client *client, struct i2c_board_info *info) 
{
	strcpy(info->type, BACKCTP_DRIVER_NAME);	
	return 0;
}
 
static void tpd_eint_interrupt_handler(void)
{
	if ( 0 == tpd_load_status ){
		return;
	}
	tpd_flag = 1;
	wake_up_interruptible(&waiter);
}

void fts_6x06_hw_reset(void)
{
	mt_set_gpio_out(GPIO_GESTURE_RST_PIN, GPIO_OUT_ONE);
	msleep(3);
	mt_set_gpio_out(GPIO_GESTURE_RST_PIN, GPIO_OUT_ZERO);
	msleep(3);
	mt_set_gpio_out(GPIO_GESTURE_RST_PIN, GPIO_OUT_ONE);
	msleep(40);
}
//LINE <ft6x06> <DATE20140910> <modify for ft6336> yolo
void fts_6x06_hw_reset_isp(int para)
{
    CTP_DBG("fts_6x06_hw_reset_isp (%d) \n", para);
    mt_set_gpio_out(GPIO_GESTURE_RST_PIN, GPIO_OUT_ONE);
    msleep(3);//mdelay(1);
    mt_set_gpio_out(GPIO_GESTURE_RST_PIN, GPIO_OUT_ZERO);
    msleep(3);//mdelay(15);
    mt_set_gpio_out(GPIO_GESTURE_RST_PIN, GPIO_OUT_ONE);
    msleep(para);//mdelay(1);
}

static void fts_6x06_hw_init(void)
{  
	printk("fts_6x06_hw_init()\n");
#ifdef TINNO_ANDROID_L8420AE
	mt_set_gpio_mode(GPIO_BACK_CTP_EN,GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_BACK_CTP_EN,GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_BACK_CTP_EN,GPIO_OUT_ONE);   
#endif

	msleep(2);  
	
	mt_set_gpio_mode(GPIO_GESTURE_RST_PIN, GPIO_GESTURE_RST_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_GESTURE_RST_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_GESTURE_RST_PIN, GPIO_OUT_ONE);
	msleep(30);
}

static char *fts_get_vendor_name(int vendor_id)
{
	switch(vendor_id){
		case FTS_CTP_VENDOR_BYD:		  return "BYD";		    break;
		case FTS_CTP_VENDOR_TRULY:		  return "TRULY";		break;
		case FTS_CTP_VENDOR_NANBO:		  return "NANBO";		break;
		case FTS_CTP_VENDOR_BAOMING:	  return "BAOMING";	    break;
		case FTS_CTP_VENDOR_JIEMIAN:	  return "JIEMIAN";	    break;
		case FTS_CTP_VENDOR_YEJI:		  return "YEJI";		break;
		case FTS_CTP_VENDOR_HUARUICHUANG: return "HUARUICHUANG";break;
		case FTS_CTP_VENDOR_DEFAULT:	  return "DEFAULT";	    break;
		default:						  return "UNKNOWN";	    break;
	}
	return "UNKNOWN";
}
//BEGIN<touch panel><date20131021><tp auto update>yinhuiyong
#ifdef  FTS_AUTO_TP_UPGRADE
static struct task_struct * focaltech_update_thread;
extern int ic_type_flag;	//LINE <ft6x06> <DATE20140910> <modify for ft6336> yolo
static char *chip_type[50];

static  int update_firmware_thread(void *priv)
{
//int i = 0;
	CTP_DBG("[Focaltech] enter update_firmware_thread\n");
	//fts_ctpm_fw_upgrade_with_i_file();
	if( 0 == memcmp(tpd_desc, "HUARUICHUANG", 12))
	{
		CTP_DBG("aijun.zhu:focaltech_auto_upgrade HUARUICHUANG------->\n");
		ft6x06_tp_upgrade(ftbin_HRC, sizeof(ftbin_HRC));
	}else if(0 == memcmp(tpd_desc, "YEJI",4))
	{
		CTP_DBG("aijun.zhu:focaltech_auto_upgrade YEJI------->\n");
	//LINE <ft6x06> <DATE20140910> <modify for ft6336> yolo
	      if(ic_type_flag){
			ft6x06_tp_upgrade(ftbin_YEJI_ft6336, sizeof(ftbin_YEJI_ft6336));
		   }else{
			ft6x06_tp_upgrade(ftbin_YEJI_ft6x06, sizeof(ftbin_YEJI_ft6x06));
	      }
	}else if(0 == memcmp(tpd_desc, "JIEMIAN",7))
	{
		CTP_DBG("aijun.zhu:focaltech_auto_upgrade JIEMIAN------->\n");
		ft6x06_tp_upgrade(ftbin_JIEMIAN, sizeof(ftbin_JIEMIAN));
	}else if (0 == memcmp(tpd_desc, "UNKNOWN",7))     
	{
		CTP_DBG("aijun.zhu:focaltech_auto_upgrade UNKNOWN------->\n");
		/*for(i = 0;i < sizeof(file_FT6406_fw_data);i++)
		{
			CTP_DBG("aijun.zhu: i = %d,file_FT6406_fw_data[i] = %d\n",i,file_FT6406_fw_data[i]);       
		}*/  
		ft6x06_tp_upgrade(file_FT6406_fw_data, sizeof(file_FT6406_fw_data));    
	}else{
		CTP_DBG("aijun.zhu:focaltech_auto_upgrade xxxxxx\n");  
	}
	
	kthread_should_stop();
	return NULL;
}

int focaltech_auto_upgrade(void)
{
	int err;
	CTP_DBG("aijun.zhu:focaltech_auto_upgrade------->\n");
	
	focaltech_update_thread = kthread_run(update_firmware_thread, 0, BACKCTP_DRIVER_NAME);
	if (IS_ERR(focaltech_update_thread)) {
		err = PTR_ERR(focaltech_update_thread);
		CTP_DBG(" failed to create update_firmware_thread thread: %d\n", err);  
	}
	return err;
}
#endif

 static int tpd_probe(struct i2c_client *client, const struct i2c_device_id *id)
 {	 
	int retval = 0;
	int panel_version = 0;
	int panel_vendor = 0;
	int iRetry = 3;
	tinno_ts_data *ts;
	int ret = 0;
	int int_value = -1;

	if ( tpd_load_status ){
		CTP_DBG("Already probed a TP, needn't to probe any more!");
		return -1;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev,"need I2C_FUNC_I2C");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}
	
	CTP_DBG("TPD enter tpd_probe ts=0x%p, TPD_RES_X=%d, TPD_RES_Y=%d, addr=0x%x\n", ts, TPD_RES_X, TPD_RES_Y, client->addr);
	memset(ts, 0, sizeof(*ts));
	g_pts = ts;

	client->timing = I2C_MASTER_CLOCK;
	ts->client = client;
	ts->start_reg = 0x00;
	atomic_set( &ts->ts_sleepState, 0 );
	mutex_init(&ts->mutex);

	i2c_set_clientdata(client, ts);

	fts_6x06_hw_init();
	msleep(120);
	
	fts_iic_init(ts);
	if ( fts_6x06_isp_init(ts) ){
		goto err_isp_register;
	}
	while (iRetry) {
		ret = ft6x06_get_vendor_version(ts, &panel_vendor, &panel_version);
		if ( panel_version < 0 || panel_vendor<0 || ret<0 ){
			CTP_DBG("Product version is %d\n", panel_version);
			fts_6x06_hw_reset();
		}else{
			break;
		}
		iRetry--;
		msleep(15);  
	} 
	if ( panel_version < 0 || panel_vendor<0 || ret<0 ){
		goto err_get_version;
	}
#ifdef TPD_HAVE_BUTTON 
	tinno_update_tp_button_dim(panel_vendor);
#endif

	if((back_ctp=(struct tpd_device*)kmalloc(sizeof(struct tpd_device), GFP_KERNEL))==NULL) 
	{
		printk("back_ctp kmalloc fail\n");
		return -ENOMEM;
	}
    	memset(back_ctp, 0, sizeof(struct tpd_device));
      /* allocate input device */
      if((back_ctp->dev=input_allocate_device())==NULL) 
      {
      		printk("back_ctp->dev : fail!\n");
    		kfree(back_ctp); 
		return -ENOMEM;
      }
	  
      back_ctp->dev->name = "tgesture-back"; 
      back_ctp->dev->phys = "tgesture-back/input0";
	  
	input_set_drvdata(back_ctp->dev, back_ctp);

	set_bit(EV_SYN, back_ctp->dev->evbit);
	set_bit(EV_KEY, back_ctp->dev->evbit);
	set_bit(EV_ABS, back_ctp->dev->evbit);
	set_bit(BTN_TOUCH,back_ctp->dev->keybit);

	 input_set_abs_params(back_ctp->dev,
        ABS_MT_POSITION_X, 0/*GESTURE_UI_X_START*/, GESTURE_UI_X_END, 0, 0);
    input_set_abs_params(back_ctp->dev,
        ABS_MT_POSITION_Y, 0/*GESTURE_UI_Y_START*/, GESTURE_UI_Y_END, 0, 0);

	if(input_register_device(back_ctp->dev))
	{
		printk("back_ctp input register fail");   
	}

    #if defined (TINNO_FULL_TOUCH_SUPPORT)
    tpded_register_sub(back_ctp->dev, GESTURE_X_MAX, GESTURE_Y_MAX, ft6X06_enable);
    #endif  /* TINNO_FULL_TOUCH_SUPPORT */
    
	mt_set_gpio_mode(GPIO_GESTURE_EINT_PIN, GPIO_GESTURE_EINT_PIN_M_EINT);
	mt_set_gpio_dir(GPIO_GESTURE_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_GESTURE_EINT_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_GESTURE_EINT_PIN, GPIO_PULL_UP);
       mt_eint_registration(CUST_EINT_GESTURE_PANEL_NUM, CUST_EINT_GESTURE_PANEL_TYPE, tpd_eint_interrupt_handler, 1);                 
	
	ts->thread = kthread_run(touch_event_handler, ts, BACKCTP_DRIVER_NAME);
	 if (IS_ERR(ts->thread)){ 
		  retval = PTR_ERR(ts->thread);
		  CTP_DBG(" failed to create kernel thread: %d\n", retval);
			goto err_start_touch_kthread;
	}

	tpd_load_status = 1;
	mt_eint_unmask(CUST_EINT_GESTURE_PANEL_NUM); 

	int_value = mt_get_gpio_in(GPIO_GESTURE_EINT_PIN);

	tp_state = int_value;     

	printk("aijun.zhu:int_value = %d\n",int_value);         
	
	CTP_DBG("Touch Panel Device(%s) Probe PASS\n", fts_get_vendor_name(panel_vendor));
	//BEGIN <tp> <DATE20130507> <tp version> zhangxiaofei
	{
		extern char tpd_desc[50];
		extern int tpd_fw_version;
		sprintf(tpd_desc, "%s", fts_get_vendor_name(panel_vendor));
		tpd_fw_version = panel_version;
	}

	if(!ic_type_flag){
		//chip_type = "FT6X06";
		sprintf(chip_type, "%s", "FT6X06");
	}else{
		//chip_type = "FT6336";
		sprintf(chip_type, "%s", "FT6336");
	}
	CTP_DBG("ftx06_tpd_get_device_chip_type chip_type=%s\n", chip_type);

#ifdef FTS_CTL_IIC
        if (ft_rw_iic_drv_init(client) < 0)
            dev_err(&client->dev, "%s:[FTS] create fts control iic driver failed\n",
                    __func__);
#endif
	printk("aijun.zhu:------------@@@@@@\n");  

//BEGIN<touch panel><date20131028><tp auto update>yinhuiyong
#ifdef  FTS_AUTO_TP_UPGRADE
		focaltech_auto_upgrade();
#endif
//END<touch panel><date20131028><tp auto update>yinhuiyong  

	printk("aijun.zhu:------------OK\n");
	return 0;
   
err_start_touch_kthread:
	//mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM); 
	mt_eint_mask(CUST_EINT_GESTURE_PANEL_NUM);
err_get_version:
err_isp_register:
	fts_6x06_isp_exit();
	mutex_destroy(&ts->mutex);
	g_pts = NULL;
	kfree(ts);
err_alloc_data_failed:
err_check_functionality_failed:
	CTP_DBG("Touch Panel Device Probe FAIL\n");
	return -1;
 }

static int tpd_remove(struct i2c_client *client)
{
	CTP_DBG("TPD removed\n");

	 input_unregister_device(back_ctp->dev);
	
#ifdef FTS_CTL_IIC
	ft_rw_iic_drv_exit();
#endif
	return 0;
}

static int ft6X06_probe(struct platform_device *pdev)
{
    return i2c_add_driver(&backctp_i2c_driver);
}

static int ft6X06_remove(struct platform_device *pdev)
{
    i2c_del_driver(&backctp_i2c_driver);      
    return 0;
}

#if defined (TINNO_FULL_TOUCH_SUPPORT)
static int ft6X06_enable(int value)
{
    CTP_DBG("ft6X06_enable %d\n", value);
    
    if (value)  /* enable device. */
    {
        fts_6x06_hw_init();
        mt_eint_unmask(CUST_EINT_GESTURE_PANEL_NUM); 
        g_tp_enable = 1;
    }
    else
    {	
        g_tp_enable = 0;
        mt_eint_mask(CUST_EINT_GESTURE_PANEL_NUM);

        mt_set_gpio_mode(GPIO_GESTURE_RST_PIN, GPIO_GESTURE_RST_PIN_M_GPIO);
        mt_set_gpio_dir(GPIO_GESTURE_RST_PIN, GPIO_DIR_OUT);
        mt_set_gpio_out(GPIO_GESTURE_RST_PIN, GPIO_OUT_ZERO);
        msleep(2);

        mt_set_gpio_mode(GPIO_BACK_CTP_EN, GPIO_BACK_CTP_EN);
        mt_set_gpio_dir(GPIO_BACK_CTP_EN, GPIO_DIR_OUT);
        mt_set_gpio_out(GPIO_BACK_CTP_EN, GPIO_OUT_ZERO);

        msleep(5);
    }
}
#endif  /* TINNO_FULL_TOUCH_SUPPORT */

static int ft6X06_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	CTP_DBG("TPD sleep done\n");
	mt_eint_mask(CUST_EINT_GESTURE_PANEL_NUM);

	mt_set_gpio_mode(GPIO_GESTURE_RST_PIN, GPIO_GESTURE_RST_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_GESTURE_RST_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_GESTURE_RST_PIN, GPIO_OUT_ZERO);
	msleep(2);

       mt_set_gpio_mode(GPIO_BACK_CTP_EN, GPIO_BACK_CTP_EN);
	mt_set_gpio_dir(GPIO_BACK_CTP_EN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_BACK_CTP_EN, GPIO_OUT_ZERO);

	msleep(5);

	return 0;
 }

static int ft6X06_resume(struct platform_device *pdev)
{
    	// reset ctp 
	fts_6x06_hw_init();
	
    	CTP_DBG("TPD wake up done\n");
	
    	mt_eint_unmask(CUST_EINT_GESTURE_PANEL_NUM);        

    	return 0;
 }

// platform structure
static struct platform_driver backctp_ft6x06_driver = {
    .probe        = ft6X06_probe,
    .remove    = ft6X06_remove,
    .suspend    = ft6X06_suspend,
    .resume    = ft6X06_resume,
    .driver        = {
        .name    = BACKCTP_DRIVER_NAME,
        .owner    = THIS_MODULE,
    }
};
static struct platform_device backctp_ft6x06_device = {
    .name = BACKCTP_DRIVER_NAME,
    .id = 0,
    .dev = {}
};

 /* called when loaded into kernel */
 static int __init tpd_driver_init(void) 
 {
	printk("aijun.zhu:ft6x06 touch panel dricer init\n");
	i2c_register_board_info(TPD_I2C_GROUP_ID, &ft6x06_i2c_backctp, sizeof(ft6x06_i2c_backctp)/sizeof(ft6x06_i2c_backctp[0]));
	if(platform_device_register(&backctp_ft6x06_device)){
        printk("aijun.zhu:failed to register ft6x06 driver\n");
        return -ENODEV;
    }

    if(platform_driver_register(&backctp_ft6x06_driver)){
        printk("aijun.zhu:failed to register ft6x06s driver\n");
        return -ENODEV;
    }

	printk("aijun.zhu:ft6x06 touch panel dricer init OK\n");
    return 0;   
 }
 
 /* should never be called */
static void __exit tpd_driver_exit(void) 
{
	platform_driver_unregister(&backctp_ft6x06_driver);   
	 
}
 
 module_init(tpd_driver_init);
 module_exit(tpd_driver_exit);

MODULE_DESCRIPTION("FT6X06 touch panel module driver");
MODULE_AUTHOR("aijun.zhu <aijun.zhu@tinno.com>");
MODULE_LICENSE("GPL");
