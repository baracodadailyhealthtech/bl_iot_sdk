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
INCLUDE_COMPONENTS += easyflash4 coredump mbedtls_lts

INCLUDE_COMPONENTS += openthread 
INCLUDE_COMPONENTS += openthread_port 
INCLUDE_COMPONENTS += openthread_utils_bl702l 

ifeq ($(CONFIG_CHIP_REVISION),A1)
INCLUDE_COMPONENTS += bl702l_rom_a1
else
INCLUDE_COMPONENTS += bl702l_rom_a0
endif

CPPFLAGS += -DCFG_CPP_ENABLE=1

include $(BL60X_SDK_PATH)/components/network/thread/openthread_common.mk

CPPFLAGS += -DOT_TASK_SIZE=1500
include $(BL60X_SDK_PATH)/make_scripts_riscv/project.mk
