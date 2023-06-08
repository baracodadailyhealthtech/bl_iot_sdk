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

INCLUDE_COMPONENTS += bl702l_rom_ext

ifeq ($(CONFIG_USE_PSRAM), 1)
CPPFLAGS += -DCFG_USE_PSRAM=1
endif

ifeq ($(CONFIG_PDS_ENABLE), 1)
CPPFLAGS += -DCFG_PDS_ENABLE=1
CPPFLAGS += -DCFG_PDS_LEVEL=31
endif

CPPFLAGS += -DCFG_CPP_ENABLE=1

include $(BL60X_SDK_PATH)/components/network/thread/openthread_common.mk

CPPFLAGS += -I$(BL60X_SDK_PATH)/components/platform/soc/bl702l/bl702l_rom_a0

CPPFLAGS += -DOT_TASK_SIZE=1500
include $(BL60X_SDK_PATH)/make_scripts_riscv/project.mk

EXTRA_LDFLAGS += -Wl,--print-memory-usage
ifdef CONFIG_CACHE_SIZE
EXTRA_LDFLAGS += -Wl,--defsym=__CACHE_SIZE=$(CONFIG_CACHE_SIZE)
endif
