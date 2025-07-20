#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#

BTBLECONTROLLER_LIBS := uarthci uarthcilp
BTBLECONTROLLER_LIB_DEFAULT := uarthci
BTBLECONTROLLER_LIB_PREFIX := btblecontroller_702l_

PROJECT_NAME := bl702l_blecontroller_test
PROJECT_PATH := $(abspath .)
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

INCLUDE_COMPONENTS += bl702l bl702l_std bl702l_rf newlibc hosal vfs yloop utils cli blog
INCLUDE_COMPONENTS += blfdt blmtd
INCLUDE_COMPONENTS += btble_pds coredump

ifeq ($(CONFIG_BTBLECONTROLLER_LIB),all)
	INCLUDE_COMPONENTS += $(addprefix $(BTBLECONTROLLER_LIB_PREFIX), $(BTBLECONTROLLER_LIBS))
else
	ifeq ($(findstring $(CONFIG_BTBLECONTROLLER_LIB), $(BTBLECONTROLLER_LIBS)),)
		INCLUDE_COMPONENTS += $(addprefix $(BTBLECONTROLLER_LIB_PREFIX), $(BTBLECONTROLLER_LIB_DEFAULT))
	else
		INCLUDE_COMPONENTS += $(addprefix $(BTBLECONTROLLER_LIB_PREFIX), $(CONFIG_BTBLECONTROLLER_LIB))
	endif
endif

ifeq ($(CONFIG_BUILD_ROM_CODE),1)
INCLUDE_COMPONENTS += bl702l_freertos
else
ifeq ($(CONFIG_CHIP_REVISION),A1)
INCLUDE_COMPONENTS += bl702l_rom_a1
else
INCLUDE_COMPONENTS += bl702l_rom_a0
endif
endif

INCLUDE_COMPONENTS += $(PROJECT_NAME)

ifeq ($(CONFIG_BL702_USE_LHAL),1)
INCLUDE_COMPONENTS += lhal
endif

include $(BL60X_SDK_PATH)/make_scripts_riscv/project.mk

ifeq ($(CONFIG_BTBLECONTROLLER_LIB),all)
LDFLAGS := $(filter-out $(addprefix -l$(BTBLECONTROLLER_LIB_PREFIX), $(filter-out $(BTBLECONTROLLER_LIB_DEFAULT), $(BTBLECONTROLLER_LIBS))), $(LDFLAGS))
endif