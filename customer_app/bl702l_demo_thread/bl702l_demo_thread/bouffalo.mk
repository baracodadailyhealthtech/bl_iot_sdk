#
# "main" pseudo-component makefile.
#
# (Uses default behaviour of compiling all source files in directory, adding 'include' to include path.)
include $(BL60X_SDK_PATH)/components/network/thread/openthread_common.mk

ifeq ($(CONFIG_SYS_AOS_CLI_ENABLE),1)
CPPFLAGS += -DSYS_AOS_CLI_ENABLE
endif

ifeq ($(CONFIG_DBG_RUN_ON_FPGA), 1)
CPPFLAGS += -DCFG_FPGA
endif

ifeq ($(CONFIG_BUILD_ROM_CODE),1)
CPPFLAGS += -DCFG_USE_FLASH_CODE
else
CPPFLAGS += -DCFG_USE_ROM_CODE
endif

ifeq ($(CONFIG_CSL_RX), 1)
CPPFLAGS += -DCFG_CSL_RX
endif

ifdef OT_NCP
CPPFLAGS += -DOT_NCP
endif

## not be exported to project level
COMPONENT_PRIV_INCLUDEDIRS :=

## This component's src 
ifeq ($(CONFIG_PDS_ENABLE), 1)
	COMPONENT_SRCS := main_pds.c
else
	COMPONENT_SRCS := main.c
endif

COMPONENT_OBJS := $(patsubst %.cpp,%.o, $(filter %.cpp,$(COMPONENT_SRCS))) $(patsubst %.c,%.o, $(filter %.c,$(COMPONENT_SRCS))) $(patsubst %.S,%.o, $(filter %.S,$(COMPONENT_SRCS)))
COMPONENT_SRCDIRS := .
