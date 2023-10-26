#
# "main" pseudo-component makefile.
#
# (Uses default behaviour of compiling all source files in directory, adding 'include' to include path.)


include $(BL60X_SDK_PATH)/components/network/thread/openthread_common.mk

## This component's src 
COMPONENT_SRCS := main.c

ifeq ($(CONFIG_USE_WIFI_BR), 1)
COMPONENT_SRCS += wifi_lwip.c
ifeq ($(CONFIG_BT),1)
include $(BL60X_SDK_PATH)/components/network/ble/ble_common.mk
COMPONENT_SRCS += blsync_ble_app.c
endif
else
COMPONENT_SRCS += eth_lwip.c
endif

CPPFLAGS += -DSYS_AOS_CLI_ENABLE

ifeq ($(CONFIG_USB_CDC),1)
CPPFLAGS += -DCFG_USB_CDC_ENABLE
CFLAGS += -DCFG_USB_CDC_ENABLE
endif

ifeq ($(CONFIG_USE_PSRAM),1)
CPPFLAGS += -DCFG_USE_PSRAM
endif

ifeq ($(CONFIG_USE_WIFI_BR), 1)
CPPFLAGS += -DCFG_USE_WIFI_BR
CPPFLAGS += -DCONF_USER_ENABLE_VFS_SPI
endif

ifeq ($(CONFIG_THREAD_AUTO_START), 1)
CPPFLAGS += -DCFG_THREAD_AUTO_START
endif

ifeq ($(CONFIG_LINK_CUSTOMER),1)
COMPONENT_ADD_LDFLAGS += -L $(PROJECT_PATH)/$(notdir $(PROJECT_PATH)) $(addprefix -T , otbr_psram_flash.ld)
endif

COMPONENT_OBJS := $(patsubst %.cpp,%.o, $(filter %.cpp,$(COMPONENT_SRCS))) $(patsubst %.c,%.o, $(filter %.c,$(COMPONENT_SRCS))) $(patsubst %.S,%.o, $(filter %.S,$(COMPONENT_SRCS)))
COMPONENT_SRCDIRS := .