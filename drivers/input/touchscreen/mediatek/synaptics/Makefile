include $(srctree)/drivers/misc/mediatek/Makefile.custom

# Linux driver folder
ccflags-y += -I$(srctree)/drivers/misc/mediatek/mach/$(MTK_PLATFORM)/$(ARCH_MTK_PROJECT)/touchpanel/synaptics/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/synaptics/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/

obj-y	+=  synaptics_dsx_fw_update.o
obj-y	+=  synaptics_dsx_i2c.o
obj-y	+=  synaptics_dsx_rmi_dev.o
obj-y	+=  synaptics_dsx_test_reporting.o

