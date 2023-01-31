#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#

PROJECT_PATH := $(abspath .)
PROJECT_NAME :=  $(notdir $(PROJECT_PATH))
PROJECT_BOARD := evb
export PROJECT_PATH PROJECT_BOARD
#CONFIG_TOOLPREFIX :=

BTBLECONTROLLER_LIBS := m1s1 m1s1p m8s1
BTBLECONTROLLER_LIB_DEFAULT := m1s1
BTBLECONTROLLER_LIB_PREFIX := btblecontroller_702l_

ZBSTACK_ADDITION_LIBS := gpp_bl702l
ZBSTACK_LIB_DEFAULT := zbstack_bl702l
ZBSTACK_LIB_PREFIX := zbstack_

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
INCLUDE_COMPONENTS += lmac154
else
INCLUDE_COMPONENTS += bl702l_rom_ext
endif

ifeq ($(CONFIG_OPENTHREAD),1)
INCLUDE_COMPONENTS += openthread_utils_bl702l
endif

ifeq ($(CONFIG_ZIGBEE),1)
ifeq ($(CONFIG_ZBSTACK_LIB),all)
	INCLUDE_COMPONENTS += $(ZBSTACK_LIB_DEFAULT)
	INCLUDE_COMPONENTS += $(addprefix $(ZBSTACK_LIB_PREFIX), $(ZBSTACK_ADDITION_LIBS))
else
	ifeq ($(findstring $(CONFIG_ZBSTACK_LIB), $(ZBSTACK_ADDITION_LIBS)),)
		INCLUDE_COMPONENTS += $(ZBSTACK_LIB_DEFAULT)
	else
		INCLUDE_COMPONENTS += $(addprefix $(ZBSTACK_LIB_PREFIX), $(CONFIG_ZBSTACK_LIB))
	endif
endif
INCLUDE_COMPONENTS += zbcli ecdsa macphy154_bl702l
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
endif

ifeq ($(CONFIG_BT_MESH),1)
INCLUDE_COMPONENTS += blemesh
ifeq ($(CONFIG_BT_MESH_MODEL),1)
INCLUDE_COMPONENTS += blemesh_model
endif
endif

ifeq ($(CONFIG_ZIGBEE_PROV),1)
INCLUDE_COMPONENTS += blsync_ble cjson
endif

ifeq ($(CONFIG_AUTO_PTS),1)
INCLUDE_COMPONENTS += bleptstest
endif

INCLUDE_COMPONENTS += $(PROJECT_NAME)

include $(BL60X_SDK_PATH)/make_scripts_riscv/project.mk
ifeq ($(CONFIG_ZIGBEE), 1)
include $(BL60X_SDK_PATH)/components/network/zigbee/zigbee_def.mk
endif

ifdef CONFIG_CACHE_SIZE
EXTRA_LDFLAGS += -Wl,--defsym=__CACHE_SIZE=$(CONFIG_CACHE_SIZE)
endif

ifeq ($(CONFIG_BTBLECONTROLLER_LIB),all)
LDFLAGS := $(filter-out $(addprefix -l$(BTBLECONTROLLER_LIB_PREFIX), $(filter-out $(BTBLECONTROLLER_LIB_DEFAULT), $(BTBLECONTROLLER_LIBS))), $(LDFLAGS))
endif

ifeq ($(CONFIG_ZBSTACK_LIB),all)
LDFLAGS := $(filter-out $(addprefix -l$(ZBSTACK_LIB_PREFIX), $(ZBSTACK_ADDITION_LIBS)), $(LDFLAGS))
endif
