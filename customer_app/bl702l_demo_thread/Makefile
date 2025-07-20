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

INCLUDE_COMPONENTS += $(PROJECT_NAME)
INCLUDE_COMPONENTS += bl702l bl702l_std newlibc bl702l_rf hosal vfs yloop utils cli blog
INCLUDE_COMPONENTS += blmtd blfdt
INCLUDE_COMPONENTS += coredump mbedtls_lts

ifeq ($(CONFIG_LITTLEFS), 1)
    INCLUDE_COMPONENTS += littlefs
else ifeq ($(CONFIG_EASYFLASH_ENABLE), 1)
    INCLUDE_COMPONENTS += easyflash4
endif

INCLUDE_COMPONENTS += thread openthread_port 
ifdef OT_NCP
    INCLUDE_COMPONENTS += openthread_ncp
endif

ifeq ($(CONFIG_BUILD_ROM_CODE), 1)
    INCLUDE_COMPONENTS += bl702_freertos lmac154_bl702l openthread_utils_bl702l
else
    ifeq ($(CONFIG_CHIP_REVISION), A1)
        INCLUDE_COMPONENTS += bl702l_rom_a1 openthread_utils_bl702l_rom
    else
        $(error BL702L_A0 doesn't support Thread ROM Code.)
    endif
endif

include $(BL60X_SDK_PATH)/components/network/thread/openthread_common.mk
include $(BL60X_SDK_PATH)/make_scripts_riscv/project.mk
