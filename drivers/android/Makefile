ccflags-y += -I$(src)			# needed for trace events

obj-$(CONFIG_ANDROID_BINDER_IPC)	+= binder.o

# wlj add for FFGALFRA-157 start{
PROJECT_NAME := $(strip $(subst ro.project =,,$(shell cat $(PWD)/../build.ini |grep ^ro.project)))
MTK_TARGET_PROJECT := $(strip $(subst ro.target =,,$(shell cat $(PWD)/../build.ini |grep ^ro.target)))
CONFIG_PROJECT_CCFLAGS := $(shell echo CONFIG_PROJECT_$(MTK_TARGET_PROJECT)_$(PROJECT_NAME) | tr a-z A-Z)
ccflags-y += -D$(CONFIG_PROJECT_CCFLAGS)
# wlj add for FFGALFRA-157 end}

