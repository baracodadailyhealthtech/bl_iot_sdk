#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#

PROJECT_PATH := $(abspath .)
PROJECT_NAME :=  $(notdir $(PROJECT_PATH))
PROJECT_BOARD := evb
export PROJECT_PATH PROJECT_BOARD
#CONFIG_TOOLPREFIX :=

BLECONTROLLER_LIBS := std m0s1 m0s1s m0s1p m16s1 m0s0sp
BLECONTROLLER_LIB_DEFAULT := std
BLECONTROLLER_LIB_PREFIX := blecontroller_702_

-include ./proj_config.mk

ifeq ($(origin BL60X_SDK_PATH), undefined)
BL60X_SDK_PATH_GUESS ?= $(shell pwd)
BL60X_SDK_PATH ?= $(BL60X_SDK_PATH_GUESS)/../..
$(info ****** Please SET BL70X_SDK_PATH ******)
$(info ****** Trying SDK PATH [$(BL60X_SDK_PATH)])
endif

INCLUDE_COMPONENTS += bl702 bl702_std newlibc bl702_rf hosal vfs yloop utils cli blog
INCLUDE_COMPONENTS += blmtd blfdt
INCLUDE_COMPONENTS += easyflash4 coredump mbedtls_lts
INCLUDE_COMPONENTS += lmac154

ifeq ($(CONFIG_USB_CDC), 1)
INCLUDE_COMPONENTS += bl702_usb_cdc
endif

ifeq ($(CONFIG_USE_PSRAM), 1)
CPPFLAGS += -DCFG_USE_PSRAM=1
endif

ifneq ($(origin CONFIG_USB_CDC_VID), undefined)
CPPFLAGS += -DCFG_USB_CDC_VID=$(CONFIG_USB_CDC_VID)
endif

ifneq ($(origin CONFIG_USB_CDC_PID), undefined)
CPPFLAGS += -DCFG_USB_CDC_PID=$(CONFIG_USB_CDC_PID)
endif

ifeq ($(CONFIG_BT),1)
ifeq ($(CONFIG_BLECONTROLLER_LIB),all)
    INCLUDE_COMPONENTS += $(addprefix $(BLECONTROLLER_LIB_PREFIX), $(BLECONTROLLER_LIBS))
else
    ifeq ($(findstring $(CONFIG_BLECONTROLLER_LIB), $(BLECONTROLLER_LIBS)),)
        INCLUDE_COMPONENTS += $(addprefix $(BLECONTROLLER_LIB_PREFIX), $(BLECONTROLLER_LIB_DEFAULT))
    else
        INCLUDE_COMPONENTS += $(addprefix $(BLECONTROLLER_LIB_PREFIX), $(CONFIG_BLECONTROLLER_LIB))
    endif
endif
INCLUDE_COMPONENTS += blestack
endif

ifeq ($(CONFIG_GEN_ROM),1)
    INCLUDE_COMPONENTS += bl702_freertos_rom
else
    ifeq ($(CONFIG_BUILD_ROM_CODE),1)
        INCLUDE_COMPONENTS += bl702_freertos
    else
        INCLUDE_COMPONENTS += bl702_freertos_rom
    endif
endif

ifeq ($(CONFIG_BT),1)
INCLUDE_COMPONENTS += blestack blecontroller
endif

INCLUDE_COMPONENTS += openthread_port openthread_utils_bl702 openthread
INCLUDE_COMPONENTS += $(PROJECT_NAME)

CPPFLAGS += -DCFG_CPP_ENABLE=1

# CPPFLAGS += -DOPENTHREAD_CONFIG_LOG_LEVEL=OT_LOG_LEVEL_DEBG

include $(BL60X_SDK_PATH)/components/network/thread/openthread_common.mk

include $(BL60X_SDK_PATH)/make_scripts_riscv/project.mk

EXTRA_LDFLAGS += -Wl,--print-memory-usage
ifdef CONFIG_CACHE_SIZE
EXTRA_LDFLAGS += -Wl,--defsym=__CACHE_SIZE=$(CONFIG_CACHE_SIZE)
endif
