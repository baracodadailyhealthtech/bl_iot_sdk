#
# "main" pseudo-component makefile.
#
# (Uses default behaviour of compiling all source files in directory, adding 'include' to include path.)
ifeq ($(CONFIG_BT),1)
include $(BL60X_SDK_PATH)/components/network/ble/ble_common.mk
endif

ifeq ($(CONFIG_USB_CDC),1)
CPPFLAGS += -DCFG_USB_CDC_ENABLE
endif

ifeq ($(CONFIG_USE_PSRAM),1)
CPPFLAGS += -DCFG_USE_PSRAM
endif

