#
# "main" pseudo-component makefile.
#
# (Uses default behaviour of compiling all source files in directory, adding 'include' to include path.)
ifeq ($(CONFIG_BT),1)
include $(BL60X_SDK_PATH)/components/network/ble/ble_common.mk
endif
# defines the following symbol to be avoid to redefined in openthread_common.mk
OPENTHREAD_FTD := 0
OPENTHREAD_MTD := 0
OPENTHREAD_CONFIG_FILE := ""
OPENTHREAD_CORE_CONFIG_PLATFORM_CHECK_FILE := ""
OPENTHREAD_CORE_CONFIG_PLATFORM_CHECK_FILE := ""
export OPENTHREAD_FTD OPENTHREAD_MTD OPENTHREAD_RADIO OPENTHREAD_CONFIG_FILE OPENTHREAD_PROJECT_CORE_CONFIG_FILE OPENTHREAD_CORE_CONFIG_PLATFORM_CHECK_FILE

include $(BL60X_SDK_PATH)/components/network/thread/openthread_common.mk

ifeq ($(CONFIG_SYS_AOS_CLI_ENABLE),1)
CPPFLAGS += -DSYS_AOS_CLI_ENABLE
endif

ifeq ($(CONFIG_USB_CDC),1)
CPPFLAGS += -DCFG_USB_CDC_ENABLE
endif

ifeq ($(CONFIG_ETHERNET), 1)
CPPFLAGS += -DCFG_ETHERNET_ENABLE
endif
