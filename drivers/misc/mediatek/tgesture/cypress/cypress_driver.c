//Gesture driver
#include <linux/interrupt.h>
#include <cust_eint.h>
#include <linux/i2c.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/rtpm_prio.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/delay.h>

#include <linux/module.h>
#include <linux/input.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/miscdevice.h>
#include <linux/hrtimer.h>
#include "tpd.h"


#include <mach/mt_pm_ldo.h> 
#include <mach/mt_typedefs.h>
#include <mach/mt_boot.h>

#include "cust_gpio_usage.h"

// for linux 2.6.36.3
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <asm/ioctl.h>

//dma
#include <linux/dma-mapping.h>
#include <linux/wakelock.h>
#include <linux/timer.h>

#include "cypress_driver.h"


/////////////////////////////////////////////////////////
// 0: Independance input device
// 1: Used TPD input device
#define GESTURE_REPORT_MODE 0

// 0: Used CTP event
// 1: Used RTP event
#define GESTURE_EVENT_MODE 1



/////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define GESTURE_SINGLE_CLICK_TIMER		(400*1000*1000) // 200 ms
#define GESTURE_DOUBLE_CLICK_TIMER		(300*1000*1000) // 400 ms
#define GESTURE_LONG_TAP_TIMER		(1200*1000*1000) // 1200 ms
#define GESTURE_SCAN_TIMER		(20*1000*1000) // 20 ms

#define GESTURE_POINT_PRESSED   0xbb
#define GESTURE_POINT_RELEASED  0xcc
#define GESTURE_POINT_INVALID   0xff

typedef enum
{
    GESTURE_STATE_IDLE,
    GESTURE_STATE_PRESSED,
    GESTURE_STATE_RELEASED,
    GESTURE_STATE_LONG_PRESSED,
    GESTURE_STATE_MAX
} cypress_gesture_state;

typedef enum
{
    GESTURE_TIMER_SCAN,
    GESTURE_TIMER_SINGLE_CLICK,
    GESTURE_TIMER_LONG_PRESSED,
    GESTURE_TIMER_MAX
} gesture_timer_type;

typedef struct 
{
    unsigned char flag;
    unsigned char x;
    unsigned char y;
    unsigned char reserve;
    struct timespec	time;
} gesture_point;


struct cypress_gesture_ts_data {
    struct i2c_client *i2c_client;
    struct input_dev *input_dev;
    struct work_struct work;
    struct early_suspend early_suspend;
    int intr_gpio;

    cypress_gesture_state   state;
    int x_resolution;
    int y_resolution;
    gesture_point point[2];
    gesture_point last_point;

    struct hrtimer timer;
};

static struct cypress_gesture_ts_data gesture_ts;
static struct cypress_gesture_ts_data *private_ts;


static struct task_struct *thread = NULL;
static struct task_struct *gesture_timer_thread = NULL;
static struct hrtimer long_pressed_timer;
static struct hrtimer single_click_timer;
static struct hrtimer pressed_scan_timer;
static gesture_timer_type gesture_timer_index = GESTURE_TIMER_MAX;
static int gesture_flag = 0;
static int gesture_timer_flag = 0;
static DECLARE_WAIT_QUEUE_HEAD(waiter);
static DECLARE_WAIT_QUEUE_HEAD(gesture_timer_waiter);
static DEFINE_MUTEX(gesture_i2c_access);



/////////////////////////////////////////////////////////////////////////////////////////////////////////////

extern struct tpd_device *tpd;
extern void mt_eint_unmask(unsigned int line);
extern void mt_eint_mask(unsigned int line);
extern void mt_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
extern unsigned int mt_eint_set_sens(unsigned int eint_num, unsigned int sens);

extern int gesture_x_max;
extern int gesture_y_max;


static int gesture_chip_eint_config(int enable);

/////////////////////////////////////////////////////////////////////////////////////////////////////////////

static int gesture_is_long_pressed(gesture_point *pt1, gesture_point *pt2)
{
    unsigned char delta_x, delta_y;
    GESTURE_DBG("point1(%d, %d, %x), point2(%d, %d, %x)\n", 
        pt1->x, pt1->y, pt1->flag, pt2->x, pt2->y, pt2->flag);

    if ((GESTURE_POINT_PRESSED != pt1->flag) || (GESTURE_POINT_PRESSED != pt2->flag))
        return 0;
    
    delta_x = abs(pt1->x - pt2->x);
    delta_y = abs(pt1->y - pt2->y);
    if ((delta_x < 10) && (delta_y < 10))
    {
        long delta_time = abs(pt1->time.tv_nsec - pt1->time.tv_nsec);

        if ((delta_time > GESTURE_LONG_TAP_TIMER))
        {
            return 1;   // long pressed.
        }
    }

    return 0;
}

static int gesture_is_single_click(gesture_point *pt1, gesture_point *pt2)
{
    unsigned char delta_x, delta_y;
    
    GESTURE_DBG("gesture_is_single_click point1(%d, %d, %x), point2(%d, %d, %x)\n", 
        pt1->x, pt1->y, pt1->flag, pt2->x, pt2->y, pt2->flag);

    if ((GESTURE_POINT_PRESSED != pt1->flag) || (GESTURE_POINT_RELEASED != pt2->flag))
        return 0;

    delta_x = abs(pt1->x - pt2->x);
    delta_y = abs(pt1->y - pt2->y);

    if ((10 > delta_x) && (10 > delta_y))
    {
        return 1;   // single click
    }

    return 0;
}


static int gesture_is_double_click(gesture_point *pt1, gesture_point *pt2)
{
    unsigned char delta_x, delta_y;
    
    GESTURE_DBG("point1(%d, %d, %x), point2(%d, %d, %x)\n", 
        pt1->x, pt1->y, pt1->flag, pt2->x, pt2->y, pt2->flag);

    if ((GESTURE_POINT_PRESSED != pt1->flag) || (GESTURE_POINT_PRESSED != pt2->flag))
        return 0;

    // check first pressed and second pressed
    delta_x = abs(pt1->x - pt2->x);
    delta_y = abs(pt1->y - pt2->y);

    if ((10 > delta_x) || (10 > delta_y))
        return 0;

    // check time
    if (GESTURE_DOUBLE_CLICK_TIMER > abs(pt2->time.tv_nsec - pt1->time.tv_nsec))
    {
        return 1;   // double click
    }
    
    return 0;
}

static int gesture_is_slide_up(gesture_point *pt1, gesture_point *pt2)
{
    unsigned char delta_x, delta_y;
    
    GESTURE_DBG("point1(%d, %d, %x), point2(%d, %d, %x)\n", 
        pt1->x, pt1->y, pt1->flag, pt2->x, pt2->y, pt2->flag);

    if ((GESTURE_POINT_PRESSED != pt1->flag) || (GESTURE_POINT_RELEASED != pt2->flag))
        return 0;

    if (pt1->y <= pt2->y)
        return 0;

    delta_x = abs(pt1->x - pt2->x);
    delta_y = abs(pt1->y - pt2->y);

    if ((delta_x < delta_y) && (50 < delta_y))
    {
        return 1;   // slide up
    }

    return 0;
}

static int gesture_is_slide_down(gesture_point *pt1, gesture_point *pt2)
{
    unsigned char delta_x, delta_y;
    
    GESTURE_DBG("point1(%d, %d, %x), point2(%d, %d, %x)\n", 
        pt1->x, pt1->y, pt1->flag, pt2->x, pt2->y, pt2->flag);

    if ((GESTURE_POINT_PRESSED != pt1->flag) || (GESTURE_POINT_RELEASED != pt2->flag))
        return 0;

    if (pt1->y >= pt2->y)
        return 0;

    delta_x = abs(pt1->x - pt2->x);
    delta_y = abs(pt1->y - pt2->y);

    if ((delta_x < delta_y) && (50 < delta_y))
    {
        return 1;   // slide down
    }

    return 0;
}


static int gesture_is_slide_left(gesture_point *pt1, gesture_point *pt2)
{
    unsigned char delta_x, delta_y;
    
    GESTURE_DBG("point1(%d, %d, %x), point2(%d, %d, %x)\n", 
        pt1->x, pt1->y, pt1->flag, pt2->x, pt2->y, pt2->flag);

    if ((GESTURE_POINT_PRESSED != pt1->flag) || (GESTURE_POINT_RELEASED != pt2->flag))
        return 0;

    if (pt1->x <= pt2->x)
        return 0;

    delta_x = abs(pt1->x - pt2->x);
    delta_y = abs(pt1->y - pt2->y);

    if ((delta_x > delta_y) && (50 < delta_x))
    {
        return 1;   // slide left
    }

    return 0;
}


static int gesture_is_slide_right(gesture_point *pt1, gesture_point *pt2)
{
    unsigned char delta_x, delta_y;
    
    GESTURE_DBG("point1(%d, %d, %x), point2(%d, %d, %x)\n", 
        pt1->x, pt1->y, pt1->flag, pt2->x, pt2->y, pt2->flag);

    if ((GESTURE_POINT_PRESSED != pt1->flag) || (GESTURE_POINT_RELEASED != pt2->flag))
        return 0;

    if (pt1->x >= pt2->x)
        return 0;

    delta_x = abs(pt1->x - pt2->x);
    delta_y = abs(pt1->y - pt2->y);

    if ((delta_x > delta_y) && (50 < delta_x))
    {
        return 1;   // slide right
    }

    return 0;
}

static void gesture_long_pressed_report(void)
{
    GESTURE_DBG("\n");
}

static void gesture_single_click_report(void)
{
    GESTURE_DBG("\n");
}

static void gesture_double_click_report(void)
{
    GESTURE_DBG("\n");
}

static void gesture_slide_up_report(void)
{
    GESTURE_DBG("\n");
}

static void gesture_slide_down_report(void)
{
    GESTURE_DBG("\n");
}

static void gesture_slide_left_report(void)
{
    GESTURE_DBG("\n");
}

static void gesture_slide_right_report(void)
{
    GESTURE_DBG("\n");
}

static void gesture_point_report(gesture_point *pt)
{
    struct input_dev *idev = NULL;
    int x, y;
    int tmp_x = pt->x, tmp_y = pt->y;

    GESTURE_DBG("\n");

    #if (GESTURE_REPORT_MODE == 0)
    idev = private_ts->input_dev;
    #else
    if (!tpd) return ;
    idev = tpd->dev;
    #endif  /* GESTURE_REPORT_MODE */
    if (!idev) return ;
    
    #if 0
    if ((pt->x >= GESTURE_X_MAX) || (pt->y >= GESTURE_Y_MAX)) 
    {   
        return ;
    }
    #else
    // convert co-ordinate direction
    {
        tmp_x = GESTURE_X_MAX - pt->y;
        tmp_y = GESTURE_Y_MAX - pt->x;
    }
    #endif  /* #if 0 */
    
    // Scaling co-ordinate to UI
    #if 0
    x = tmp_x;
    y = tmp_y;
    #else
    x = GESTURE_UI_X_START + tmp_x * (GESTURE_UI_X_END - GESTURE_UI_X_START)/GESTURE_X_MAX;
    y = GESTURE_UI_Y_START + tmp_y * (GESTURE_UI_Y_END - GESTURE_UI_Y_START)/GESTURE_Y_MAX;
    #endif  /* #if 0 */

    if (GESTURE_POINT_PRESSED == pt->flag)
    {
        GESTURE_DBG("Gesture down\n");
        #if(GESTURE_EVENT_MODE == 0)
        input_report_key(idev, BTN_TOUCH, 1);
        input_report_abs(idev, ABS_MT_POSITION_X, x);
        input_report_abs(idev, ABS_MT_POSITION_Y, y);
        input_mt_sync(idev);
        input_sync(idev);
        #else
        input_report_abs(idev, ABS_X, x);
        input_report_abs(idev, ABS_Y, y);
        input_report_key(idev, BTN_TOUCH, 1);
        input_sync(idev);
        #endif /* GESTURE_EVENT_MODE */
    }
    else
    {
        GESTURE_DBG("Gesture up\n");
        #if(GESTURE_EVENT_MODE == 0)
        input_report_key(idev, BTN_TOUCH, 0);
        input_mt_sync(idev);
        input_sync(idev);
        #else
        input_report_key(idev, BTN_TOUCH, 0);
        input_sync(idev);
        #endif /* GESTURE_EVENT_MODE */
    }

    GESTURE_DBG("report finish\n");
}

static int gesture_read_point(gesture_point *pt)
{
    struct i2c_client *client = private_ts->i2c_client;

    mutex_lock(&gesture_i2c_access);

    if (i2c_master_recv(client, (char *)pt, GESTURE_PACKAGE_SIZE) != GESTURE_PACKAGE_SIZE)
    {
        GESTURE_DBG("gesture_read_point flag=%x x=%d y=%d\n", 
            pt->flag, pt->x, pt->y);
        
        mutex_unlock(&gesture_i2c_access);
        return -EINVAL;
    }

    GESTURE_DBG("point, flag=%x x=%d y=%d\n", 
        pt->flag, pt->x, pt->y);

    if (GESTURE_POINT_RELEASED == pt->flag) // set last pressed co-ordinate when finger up.
    {
        pt->x = private_ts->last_point.x;
        pt->y = private_ts->last_point.y;
    }

    pt->time = CURRENT_TIME;

    mutex_unlock(&gesture_i2c_access);
    return 0;
}


static void gesture_eint_interrupt_handler(void)
{
    GESTURE_DBG("gesture interrupt has been triggered\n");

    gesture_chip_eint_config(1);

    gesture_flag = 1;
    wake_up_interruptible(&waiter);
}


static void gesture_chip_reset(void )
{
#ifdef GESTURE_POWER_SOURCE_CUSTOM
    hwPowerOn(GESTURE_POWER_SOURCE_CUSTOM, VOL_3300, "GESTURE");
#else
    mt_set_gpio_mode(GPIO_GESTURE_POWER_PIN, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_GESTURE_POWER_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_GESTURE_POWER_PIN, GPIO_OUT_ONE);
#endif

    msleep(1);
    mt_set_gpio_mode(GPIO_GESTURE_RST_PIN, GPIO_GESTURE_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_GESTURE_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_GESTURE_RST_PIN, GPIO_OUT_ZERO);
    msleep(10);
    mt_set_gpio_out(GPIO_GESTURE_RST_PIN, GPIO_OUT_ONE);
}

static uint8_t gesture_chip_id(void )
{
    struct i2c_client *client = private_ts->i2c_client;
    uint8_t buf_recv[GESTURE_PACKAGE_SIZE] = {0};

    GESTURE_DBG("%s\n",__func__);

    if (!client)
        return 0xff;

    mutex_lock(&gesture_i2c_access);

    if (i2c_master_recv(client, buf_recv, GESTURE_PACKAGE_SIZE) != GESTURE_PACKAGE_SIZE)
    {
        GESTURE_DBG("gesture_chip_id buf[0]=%x buf[1]=%x buf[2]=%x\n", 
            buf_recv[0], buf_recv[1], buf_recv[2]);
        mutex_unlock(&gesture_i2c_access);
        return -EINVAL;
    }
    
    GESTURE_DBG("gesture_chip_id buf[0]=%x buf[1]=%x buf[2]=%x\n", 
        buf_recv[0], buf_recv[1], buf_recv[2]);
    
    mutex_unlock(&gesture_i2c_access);
    return buf_recv[0];
}

static int gesture_chip_eint_config(int enable)
{
    int level = 0;

    mt_set_gpio_mode(GPIO_GESTURE_EINT_PIN, GPIO_GESTURE_EINT_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_GESTURE_EINT_PIN, GPIO_DIR_IN);
    mt_set_gpio_pull_enable(GPIO_GESTURE_EINT_PIN, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO_GESTURE_EINT_PIN, GPIO_PULL_UP);
    level = mt_get_gpio_in(GPIO_GESTURE_EINT_PIN);
    
    if (enable)
    {
        mt_set_gpio_mode(GPIO_GESTURE_EINT_PIN, GPIO_GESTURE_EINT_PIN_M_EINT);
        mt_set_gpio_dir(GPIO_GESTURE_EINT_PIN, GPIO_DIR_IN);
        mt_set_gpio_pull_enable(GPIO_GESTURE_EINT_PIN, GPIO_PULL_ENABLE);
        mt_set_gpio_pull_select(GPIO_GESTURE_EINT_PIN, GPIO_PULL_UP);

        mt_eint_set_hw_debounce(CUST_EINT_GESTURE_NUM, CUST_EINT_GESTURE_DEBOUNCE_CN);
        if (level)
            mt_eint_registration(CUST_EINT_GESTURE_NUM, EINTF_TRIGGER_LOW, gesture_eint_interrupt_handler, 1); 
        else
            mt_eint_registration(CUST_EINT_GESTURE_NUM, EINTF_TRIGGER_HIGH, gesture_eint_interrupt_handler, 1); 
    }
    else
    {}
}

static enum hrtimer_restart gesture_long_pressed_timer_func()
{
    GESTURE_DBG("\n");
    gesture_timer_index = GESTURE_TIMER_LONG_PRESSED;
    gesture_timer_flag = 1;
    wake_up_interruptible(&gesture_timer_waiter);
    return HRTIMER_NORESTART;
}

static enum hrtimer_restart gesture_single_click_timer_func()
{
    GESTURE_DBG("\n");
    gesture_timer_index = GESTURE_TIMER_SINGLE_CLICK;
    gesture_timer_flag = 1;
    wake_up_interruptible(&gesture_timer_waiter);
   
    return HRTIMER_NORESTART;
}

static enum hrtimer_restart gesture_scan_timer_func()
{
    GESTURE_DBG("\n");
    gesture_timer_index = GESTURE_TIMER_SCAN;
    gesture_timer_flag = 1;
    wake_up_interruptible(&gesture_timer_waiter);

    return HRTIMER_NORESTART;
}


static void gesture_report_data(struct i2c_client *client, gesture_point *pt)
{
    if (gesture_read_point(pt))
    {
        // Read package error
        return ;
    }

    gesture_point_report(pt);
    
    if (GESTURE_POINT_PRESSED == pt->flag)  // Touch down
    {
        switch (private_ts->state)
        {
            case GESTURE_STATE_IDLE:    // first point pressed
            {
                private_ts->point[0] = *pt;
                private_ts->last_point = *pt;
                private_ts->state = GESTURE_STATE_PRESSED;

                // start time for long pressed
                hrtimer_start(&long_pressed_timer, ktime_set(0, GESTURE_LONG_TAP_TIMER), HRTIMER_MODE_REL);
                hrtimer_start(&pressed_scan_timer, ktime_set(0, GESTURE_SCAN_TIMER), HRTIMER_MODE_REL);
            }
            break;

            case GESTURE_STATE_PRESSED: // ignore
            {}
            break;

            case GESTURE_STATE_RELEASED:
            {
                if (gesture_is_double_click(&private_ts->point[0], pt))    // double click ?
                {
                    private_ts->state = GESTURE_STATE_IDLE;
                    hrtimer_cancel(&single_click_timer);
                    gesture_double_click_report();
                }
            }
            break;

            default:
                break;
        }
    
    }
    else    // Touch up
    {
        switch (private_ts->state)
        {
            case GESTURE_STATE_IDLE:    // ignore
            {
            }
            break;

            case GESTURE_STATE_PRESSED:
            {
                // stop long pressed and pressed scan timer
                hrtimer_cancel(&long_pressed_timer);
                hrtimer_cancel(&pressed_scan_timer);
                
                if (gesture_is_slide_down(&private_ts->point[0], pt))
                {
                    private_ts->state = GESTURE_STATE_IDLE;
                    gesture_slide_down_report();
                }
                else if (gesture_is_slide_up(&private_ts->point[0], pt))
                {
                    private_ts->state = GESTURE_STATE_IDLE;
                    gesture_slide_up_report();
                }
                else if (gesture_is_slide_left(&private_ts->point[0], pt))
                {
                    private_ts->state = GESTURE_STATE_IDLE;
                    gesture_slide_left_report();
                }
                else if (gesture_is_slide_right(&private_ts->point[0], pt))
                {
                    private_ts->state = GESTURE_STATE_IDLE;
                    gesture_slide_right_report();
                }
                else
                {
                    private_ts->state = GESTURE_STATE_RELEASED;
                    hrtimer_start(&single_click_timer, ktime_set(0, GESTURE_SINGLE_CLICK_TIMER), HRTIMER_MODE_REL);
                }
                
            }
            break;
            
            default:
                break;
        }
        
    }
}

static int gesture_event_handler(void *unused)
{
    gesture_point point = {0};
    struct sched_param param = { .sched_priority = RTPM_PRIO_TPD };
    sched_setscheduler(current, SCHED_RR, &param);
    GESTURE_DBG("gesture_event_handler\n");

    do
    {
        mt_eint_unmask(GPIO_GESTURE_EINT_PIN);
        GESTURE_DBG("gesture_event_handler mt_eint_unmask\n");
        set_current_state(TASK_INTERRUPTIBLE);
        wait_event_interruptible(waiter, gesture_flag != 0);
        //msleep(2000);
        gesture_flag = 0;
        set_current_state(TASK_RUNNING);
        mt_eint_mask(GPIO_GESTURE_EINT_PIN);
        GESTURE_DBG("gesture_event_handler mt_eint_mask\n");
        
        gesture_report_data(private_ts->i2c_client, &point);

    }while(!kthread_should_stop());

    return 0;
}

static int gesture_timer_handler(void *unused)
{
    gesture_point point;
    struct sched_param param = { .sched_priority = RTPM_PRIO_TPD };
    sched_setscheduler(current, SCHED_RR, &param);
    GESTURE_DBG("gesture_timer_handler\n");

    do
    {
        set_current_state(TASK_INTERRUPTIBLE);
        wait_event_interruptible(gesture_timer_waiter, gesture_timer_flag != 0);
        //msleep(2000);
        gesture_timer_flag = 0;
        set_current_state(TASK_RUNNING);

        gesture_read_point(&point);
        if (GESTURE_TIMER_SCAN == gesture_timer_index)  // scan point
        {
            GESTURE_DBG("Scan point\n");
            if (GESTURE_POINT_PRESSED == point.flag)
            {
                gesture_point_report(&point);
                
                private_ts->last_point = point;
                hrtimer_start(&pressed_scan_timer, ktime_set(0, GESTURE_SCAN_TIMER), HRTIMER_MODE_REL);
            }
            else
            {
                hrtimer_cancel(&pressed_scan_timer);
            }
        }
        else if (GESTURE_TIMER_SINGLE_CLICK == gesture_timer_index)  // single click
        {
            GESTURE_DBG("Single click\n");
            if (GESTURE_POINT_RELEASED == point.flag)
            {
                if (gesture_is_single_click(&private_ts->point[0], &point))
                {
                    private_ts->state = GESTURE_STATE_IDLE;
                    gesture_single_click_report();
                }
            }
        }
        else if (GESTURE_TIMER_LONG_PRESSED == gesture_timer_index)  // long press
        {
            GESTURE_DBG("Long press\n");

            if (GESTURE_POINT_PRESSED == point.flag)
            {
                if (gesture_is_long_pressed(&private_ts->point[0], &point))
                {
                    private_ts->state = GESTURE_STATE_LONG_PRESSED;
                    gesture_long_pressed_report();
                }
            }
        }
    }while(!kthread_should_stop());

    return 0;
}

static int gesture_set_input_dev(void)
{
	int retval;
	int temp;
    
    GESTURE_DBG("\n");

	private_ts->input_dev = input_allocate_device();
	if (private_ts->input_dev == NULL) {
		GESTURE_DBG("Failed to allocate input device\n");
		retval = -ENOMEM;
		goto err_input_device;
	}

	private_ts->input_dev->name = GESTURE_DEVICE;
	private_ts->input_dev->phys = "tgesture-back/input0";
	private_ts->input_dev->id.product = 1;
	private_ts->input_dev->id.version = 1;
	private_ts->input_dev->id.bustype = BUS_I2C;
	private_ts->input_dev->dev.parent = &private_ts->i2c_client->dev;
	input_set_drvdata(private_ts->input_dev, private_ts);

    #if(GESTURE_EVENT_MODE == 0)
	set_bit(EV_SYN, private_ts->input_dev->evbit);
	set_bit(EV_KEY, private_ts->input_dev->evbit);
	set_bit(EV_ABS, private_ts->input_dev->evbit);
	set_bit(BTN_TOUCH, private_ts->input_dev->keybit);
	//set_bit(BTN_TOOL_FINGER, private_ts->input_dev->keybit);

    input_set_abs_params(private_ts->input_dev,
        ABS_MT_POSITION_X, 0/*GESTURE_UI_X_START*/, GESTURE_UI_X_END, 0, 0);
    input_set_abs_params(private_ts->input_dev,
        ABS_MT_POSITION_Y, 0/*GESTURE_UI_Y_START*/, GESTURE_UI_Y_END, 0, 0);
    #else
	set_bit(EV_SYN, private_ts->input_dev->evbit);
	set_bit(EV_KEY, private_ts->input_dev->evbit);
	set_bit(EV_ABS, private_ts->input_dev->evbit);
	set_bit(BTN_TOUCH, private_ts->input_dev->keybit);

    input_set_abs_params(private_ts->input_dev,
        ABS_X, 0/*GESTURE_UI_X_START*/, GESTURE_UI_X_END, 0, 0);
    input_set_abs_params(private_ts->input_dev,
        ABS_Y, 0/*GESTURE_UI_Y_START*/, GESTURE_UI_Y_END, 0, 0);
    #endif /* GESTURE_EVENT_MODE */

	retval = input_register_device(private_ts->input_dev);
	if (retval) {
		GESTURE_DBG("Failed to register input device\n");
		goto err_register_input;
	}

	return 0;

err_register_input:
err_query_device:
	input_free_device(private_ts->input_dev);

err_input_device:
	return retval;
}

static int gesture_probe(struct i2c_client *client, const struct i2c_device_id *id)
{ 
    int retval = 0;
    int chip_id = 0;

    GESTURE_DBG("%s\n",__func__);
    client->timing =  100;

    private_ts = &gesture_ts;
    private_ts->i2c_client = client;

    #if 0
    mt_set_gpio_mode(GPIO_GESTURE_EINT_PIN, GPIO_GESTURE_EINT_PIN_M_EINT);
    mt_set_gpio_dir(GPIO_GESTURE_EINT_PIN, GPIO_DIR_IN);
    mt_set_gpio_pull_enable(GPIO_GESTURE_EINT_PIN, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO_GESTURE_EINT_PIN, GPIO_PULL_UP);
    #endif  /* #if 0 */

    chip_id = gesture_chip_id();
    GESTURE_DBG("chip_id: %2x\n", chip_id);
    if ((0xbb != chip_id) && (0xcc != chip_id))
    { 
        goto read_id_error;
    }

    #if (GESTURE_REPORT_MODE == 0)
    gesture_set_input_dev();
    #endif  /* GESTURE_REPORT_MODE */
    
    gesture_chip_eint_config(1);       
         
    thread = kthread_run(gesture_event_handler, 0, "tgesture-event");
    if(IS_ERR(thread))
    {
        retval = PTR_ERR(thread);
        GESTURE_DBG("failed to create kernel thread: %ld\n", retval);
    }
    
    gesture_timer_thread = kthread_run(gesture_timer_handler, 0, "tgesture-timer");
    if(IS_ERR(gesture_timer_thread))
    {
        retval = PTR_ERR(gesture_timer_thread);
        GESTURE_DBG("failed to create kernel gesture_timer_thread: %ld\n", retval);
    }

    private_ts->state = GESTURE_STATE_IDLE;
    private_ts->point[0].flag = GESTURE_POINT_INVALID;
    
    //---------add in probe----------
    // init timer
    //msleep(50);
    hrtimer_init(&long_pressed_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL); 
    long_pressed_timer.function = gesture_long_pressed_timer_func;	
    
    hrtimer_init(&single_click_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL); 
    single_click_timer.function = gesture_single_click_timer_func;	
    
    hrtimer_init(&pressed_scan_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL); 
    pressed_scan_timer.function = gesture_scan_timer_func;	

    // enable eint
    mt_eint_unmask(CUST_EINT_GESTURE_NUM);

    // Set common module parameter.
    gesture_x_max = GESTURE_X_MAX;
    gesture_y_max = GESTURE_Y_MAX;
    
    return 0;
 
read_id_error:  
    GESTURE_DBG("gesture_probe fail\n");
    mt_eint_mask(GPIO_GESTURE_EINT_PIN);
    return -1;
}

static int gesture_remove(struct i2c_client *client)
{
    GESTURE_DBG("gesture removed\n");
    return 0;
}

static int gesture_detect(struct i2c_client *client, int kind, struct i2c_board_info *info)
{
    strcpy(info->type, GESTURE_DEVICE);
    
    return 0;
}


static struct i2c_board_info __initdata i2c_gesture = { I2C_BOARD_INFO(GESTURE_DEVICE, (0x08))};
static const struct i2c_device_id gesture_id[] = {{GESTURE_DEVICE, 0}, {}};

static struct i2c_driver gesture_i2c_driver =
{
    .driver = {
        .name = GESTURE_DEVICE,
        .owner = THIS_MODULE,
    },
    .probe = gesture_probe,
    .remove =  gesture_remove,
    .id_table = gesture_id,
    .detect = gesture_detect,
   // .address_data = &addr_data,
};

static int __init gesture_driver_init(void)
{
    GESTURE_DBG("cypress driver init\n");
    
    i2c_register_board_info(GESTURE_I2C_NUM, &i2c_gesture, 1);

    if(i2c_add_driver(&gesture_i2c_driver) != 0)
    {
        GESTURE_DBG("unable to add i2c driver.\n");
        return -1;
    }

    gesture_chip_reset();

    return 0;
}

static void __exit gesture_driver_exit(void)
{
    GESTURE_DBG("%s driver exit\n", __func__);
}


module_init(gesture_driver_init);
module_exit(gesture_driver_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("gesture DRIRVER");
MODULE_AUTHOR("jake.liang");

