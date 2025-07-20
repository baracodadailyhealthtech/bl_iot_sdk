#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#

PROJECT_PATH := $(abspath .)
PROJECT_NAME :=  $(notdir $(PROJECT_PATH))
PROJECT_BOARD := evb
export PROJECT_PATH PROJECT_BOARD
#CONFIG_TOOLPREFIX :=

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
ifeq ($(CONF_ENABLE_COREDUMP), 1)
INCLUDE_COMPONENTS += coredump
endif
INCLUDE_COMPONENTS += lmac154
INCLUDE_COMPONENTS += lwip netutils

ifeq ($(CONFIG_USB_CDC),1)
INCLUDE_COMPONENTS += bl702_usb_cdc
endif

ifeq ($(CONFIG_LITTLEFS), 1)
    INCLUDE_COMPONENTS += littlefs
else ifeq ($(CONFIG_EASYFLASH_ENABLE), 1)
    INCLUDE_COMPONENTS += easyflash4
endif

ifeq ($(CONFIG_USE_WIFI_BR), 1)
INCLUDE_COMPONENTS += virt_net tpsync
else
INCLUDE_COMPONENTS += bl702_ethernet lwip_dhcpd blota
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

INCLUDE_COMPONENTS += thread openthread_port openthread_utils_bl702 openthread_br
INCLUDE_COMPONENTS += $(PROJECT_NAME)

CPPFLAGS += -DINCLUDE_xTaskGetHandle=1

include $(BL60X_SDK_PATH)/components/network/thread/openthread_common.mk

include $(BL60X_SDK_PATH)/make_scripts_riscv/project.mk
