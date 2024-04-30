#
# "main" pseudo-component makefile.
#
# (Uses default behaviour of compiling all source files in directory, adding 'include' to include path.)

ifeq ($(CONFIG_BLUETOOTH),1)
include $(BL60X_SDK_PATH)/components/network/ble/ble_common.mk
endif

ifeq ($(CONFIG_ATVV_SERVER_ENABLE), 1)
CPPFLAGS += -DCONFIG_ATVV_SERVER_ENABLE
endif
