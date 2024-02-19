#
# "main" pseudo-component makefile.
#
# (Uses default behaviour of compiling all source files in directory, adding 'include' to include path.)

include $(BL60X_SDK_PATH)/components/network/ble/ble_common.mk

ifeq ($(CONFIG_LINK_CUSTOMER),1)
LINKER_SCRIPTS := flash_lp.ld
$(info use $(LINKER_SCRIPTS))
COMPONENT_ADD_LDFLAGS += -L $(BL60X_SDK_PATH)/components/platform/soc/bl702/bl702/evb/ld/ $(addprefix -T ,$(LINKER_SCRIPTS))
COMPONENT_ADD_LINKER_DEPS := $(addprefix $(BL60X_SDK_PATH)/components/platform/soc/bl702/bl702/evb/ld/,$(LINKER_SCRIPTS))
endif ## CONFIG_LINK_CUSTOMER

ifeq ($(CONFIG_PDS_ENABLE),1)
CONFIG_PDS_LEVEL ?= 31
CPPFLAGS += -DCFG_PDS_ENABLE
CPPFLAGS += -DCFG_PDS_LEVEL=$(CONFIG_PDS_LEVEL)
CPPFLAGS += -DCONFIG_HW_SEC_ENG_DISABLE
endif