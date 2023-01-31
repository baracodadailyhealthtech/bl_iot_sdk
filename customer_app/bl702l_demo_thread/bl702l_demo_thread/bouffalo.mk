#
# "main" pseudo-component makefile.
#
# (Uses default behaviour of compiling all source files in directory, adding 'include' to include path.)
ifeq ($(CONFIG_BT),1)
include $(BL60X_SDK_PATH)/components/network/ble/ble_common.mk
endif

include $(BL60X_SDK_PATH)/components/network/thread/openthread_common.mk

EXT_MK := $(PROJECT_PATH)/$(notdir $(PROJECT_PATH))/bouffalo_ext.mk
ifeq ($(EXT_MK), $(wildcard $(EXT_MK)))
include $(PROJECT_PATH)/$(notdir $(PROJECT_PATH))/bouffalo_ext.mk
endif

ifeq ($(CONFIG_SYS_AOS_CLI_ENABLE),1)
CPPFLAGS += -DSYS_AOS_CLI_ENABLE
endif

ifeq ($(CONFIG_DBG_RUN_ON_FPGA), 1)
CPPFLAGS += -DCFG_FPGA
endif

ifeq ($(CONFIG_OT_ROM_ENABLE), 1)
CPPFLAGS += -DCFG_OT_ROM_ENABLE
endif

ifeq ($(CONFIG_PDS_ENABLE),1)
CONFIG_PDS_LEVEL ?= 31
CPPFLAGS += -DCFG_PDS_ENABLE
CPPFLAGS += -DCFG_PDS_LEVEL=$(CONFIG_PDS_LEVEL)
endif

ifeq ($(CONFIG_CSL_RX), 1)
CPPFLAGS += -DCFG_CSL_RX
endif

CPPFLAGS += -D$(CONFIG_CHIP_NAME)