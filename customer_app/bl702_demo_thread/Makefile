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
INCLUDE_COMPONENTS += coredump mbedtls_lts
INCLUDE_COMPONENTS += lmac154

ifeq ($(CONFIG_USB_CDC), 1)
    INCLUDE_COMPONENTS += bl702_usb_cdc
endif

ifeq ($(CONFIG_LITTLEFS), 1)
    INCLUDE_COMPONENTS += littlefs
else ifeq ($(CONFIG_EASYFLASH_ENABLE), 1)
    INCLUDE_COMPONENTS += easyflash4
endif


ifeq ($(CONFIG_COMPONENT_BUGKILLER_ENABLE), 1)
    INCLUDE_COMPONENTS += bugkiller_freertos
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

INCLUDE_COMPONENTS += thread openthread_port openthread_utils_bl702
ifdef OT_NCP
    INCLUDE_COMPONENTS += openthread_ncp
endif

INCLUDE_COMPONENTS += $(PROJECT_NAME)

include $(BL60X_SDK_PATH)/make_scripts_riscv/project.mk
