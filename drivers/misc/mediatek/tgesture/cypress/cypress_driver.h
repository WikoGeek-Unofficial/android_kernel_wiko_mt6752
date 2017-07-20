#ifndef _LINUX_CYPRESS_H
#define _LINUX_CYPRESS_H

#define GESTURE_X_MAX      (150-1) //255
#define GESTURE_Y_MAX      (200-1) //255

#define GESTURE_UI_X_START      0
#define GESTURE_UI_X_END      719
#define GESTURE_UI_Y_START      50
#define GESTURE_UI_Y_END      1185


//Gesture define
#define GESTURE_SINGLE_CLICK      0x01 
#define GESTURE_DOUBLE_CLICK      0x02
#define GESTURE_LONG_PRESS        0x03
#define GESTURE_SLIDE_LEFT        0x04
#define GESTURE_SLIDE_RIGHT       0x05
#define GESTURE_SLIDE_UP          0x06
#define GESTURE_SLIDE_DOWN        0x07


#define GESTURE_DEVICE "tgesture-back"

// Project relation define
#define GESTURE_I2C_NUM 1

//#define GESTURE_POWER_SOURCE_CUSTOM 	MT6325_POWER_LDO_VGP1
#define GPIO_GESTURE_POWER_PIN         (GPIO146 | 0x80000000)

#define GPIO_GESTURE_RST_PIN         (GPIO123 | 0x80000000)
#define GPIO_GESTURE_RST_PIN_M_GPIO    GPIO_MODE_00

#define GPIO_GESTURE_EINT_PIN         (GPIO115 | 0x80000000)
#define GPIO_GESTURE_EINT_PIN_M_GPIO  GPIO_MODE_00
#define GPIO_GESTURE_EINT_PIN_M_EINT  GPIO_MODE_04

#define CUST_EINT_GESTURE_NUM           4
#define CUST_EINT_GESTURE_DEBOUNCE_CN      64


#define GESTURE_PACKAGE_SIZE    3


#if 1
#define GESTURE_DBG(fmt, arg...) \
	printk("[Gesture] %s (line:%d) :" fmt, __func__, __LINE__, ## arg)
#else
#define GESTURE_DBG(fmt, arg...) do {} while (0)
#endif
#endif /* _LINUX_CYPRESS_H */
