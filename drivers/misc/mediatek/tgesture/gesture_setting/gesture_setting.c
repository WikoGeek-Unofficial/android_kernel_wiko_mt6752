#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/types.h>
#include <linux/seq_file.h>
#include <linux/list.h>
#include <linux/proc_fs.h> 


int gesture_x_min = 0;
module_param(gesture_x_min, int, 00664);

int gesture_x_max = 100;
module_param(gesture_x_max, int, 00664);

int gesture_y_min = 0;
module_param(gesture_y_min, int, 00664);

int gesture_y_max = 100;
module_param(gesture_y_max, int, 00664);

