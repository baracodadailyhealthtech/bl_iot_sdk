#
# "main" pseudo-component makefile.
#
# (Uses default behaviour of compiling all source files in directory, adding 'include' to include path.)
ifeq ($(CONFIG_BLUETOOTH),1)
include $(BL60X_SDK_PATH)/components/network/ble/ble_common.mk
endif

EXT_MK := $(PROJECT_PATH)/$(notdir $(PROJECT_PATH))/bouffalo_ext.mk
ifeq ($(EXT_MK), $(wildcard $(EXT_MK)))
include $(PROJECT_PATH)/$(notdir $(PROJECT_PATH))/bouffalo_ext.mk
endif

ifeq ($(CONFIG_LINK_CUSTOMER),1)
LINKER_SCRIPTS := flash_rom_lp.ld
$(info use ble_only/$(LINKER_SCRIPTS))
COMPONENT_ADD_LDFLAGS += -L $(BL60X_SDK_PATH)/components/platform/soc/bl702l/bl702l/evb/ld/ble_only/ $(addprefix -T ,$(LINKER_SCRIPTS))
COMPONENT_ADD_LINKER_DEPS := $(addprefix $(BL60X_SDK_PATH)/components/platform/soc/bl702l/bl702l/evb/ld/ble_only/,$(LINKER_SCRIPTS))
endif ## CONFIG_LINK_CUSTOMER