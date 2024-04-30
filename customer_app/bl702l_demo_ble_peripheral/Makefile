#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#

PROJECT_PATH := $(abspath .)
PROJECT_NAME :=  $(notdir $(PROJECT_PATH))
PROJECT_BOARD := evb
export PROJECT_PATH PROJECT_BOARD
#CONFIG_TOOLPREFIX :=

BTBLECONTROLLER_LIBS := m0s1p m0s1sp m0s2p m1s1p m0s1rp m2s1p m4s1p m8s1p
BTBLECONTROLLER_LIB_DEFAULT := m1s1p
BTBLECONTROLLER_LIB_PREFIX := btblecontroller_702l_

-include ./proj_config.mk

ifeq ($(origin BL60X_SDK_PATH), undefined)
BL60X_SDK_PATH_GUESS ?= $(shell pwd)
BL60X_SDK_PATH ?= $(BL60X_SDK_PATH_GUESS)/../..
$(info ****** Please SET BL70X_SDK_PATH ******)
$(info ****** Trying SDK PATH [$(BL60X_SDK_PATH)])
endif

INCLUDE_COMPONENTS += bl702l bl702l_std bl702l_rf
INCLUDE_COMPONENTS += newlibc hosal vfs yloop utils cli blog
INCLUDE_COMPONENTS += blfdt
INCLUDE_COMPONENTS += blmtd easyflash4
INCLUDE_COMPONENTS += coredump

ifeq ($(CONFIG_BUILD_ROM_CODE),1)
INCLUDE_COMPONENTS += bl702l_freertos
else
ifeq ($(CONFIG_CHIP_REVISION),A1)
INCLUDE_COMPONENTS += bl702l_rom_a1
else
INCLUDE_COMPONENTS += bl702l_rom_a0
endif
ifeq ($(CONFIG_BUILD_FREERTOS),1)
INCLUDE_COMPONENTS += bl702l_freertos
endif
endif

ifeq ($(CONFIG_BLUETOOTH),1)
ifeq ($(CONFIG_BTBLECONTROLLER_LIB),all)
	INCLUDE_COMPONENTS += $(addprefix $(BTBLECONTROLLER_LIB_PREFIX), $(BTBLECONTROLLER_LIBS))
else
	ifeq ($(findstring $(CONFIG_BTBLECONTROLLER_LIB), $(BTBLECONTROLLER_LIBS)),)
		INCLUDE_COMPONENTS += $(addprefix $(BTBLECONTROLLER_LIB_PREFIX), $(BTBLECONTROLLER_LIB_DEFAULT))
	else
		INCLUDE_COMPONENTS += $(addprefix $(BTBLECONTROLLER_LIB_PREFIX), $(CONFIG_BTBLECONTROLLER_LIB))
	endif
endif
INCLUDE_COMPONENTS += blestack
INCLUDE_COMPONENTS += btble_pds
endif

INCLUDE_COMPONENTS += $(PROJECT_NAME)

include $(BL60X_SDK_PATH)/make_scripts_riscv/project.mk

ifdef CONFIG_CACHE_SIZE
EXTRA_LDFLAGS += -Wl,--defsym=__CACHE_SIZE=$(CONFIG_CACHE_SIZE)
endif

ifdef CONFIG_BLE_EM_SIZE
EXTRA_LDFLAGS += -Wl,--defsym=__EM_SIZE=$(CONFIG_BLE_EM_SIZE)
endif

ifeq ($(CONFIG_BTBLECONTROLLER_LIB),all)
LDFLAGS := $(filter-out $(addprefix -l$(BTBLECONTROLLER_LIB_PREFIX), $(filter-out $(BTBLECONTROLLER_LIB_DEFAULT), $(BTBLECONTROLLER_LIBS))), $(LDFLAGS))
endif
