# Component Makefile
#
## These include paths would be exported to project level
COMPONENT_ADD_INCLUDEDIRS += inc
## not be exported to project level
COMPONENT_PRIV_INCLUDEDIRS :=
## This component's src 
COMPONENT_SRCS := src/easyflash.c src/ef_env.c \
                  src/ef_env_legacy_wl.c src/ef_env_legacy.c \
                  src/ef_port.c src/ef_utils.c \
                  src/easyflash_cli.c

COMPONENT_OBJS := $(patsubst %.c,%.o, $(COMPONENT_SRCS))
COMPONENT_SRCDIRS := src

##
#CPPFLAGS += 
ifneq ($(CONFIG_CLI_CMD_ENABLE),0)
CPPFLAGS += -DCONFIG_CLI_CMD_ENABLE
endif

CPPFLAGS += -D USE_UTILS_CRC
CPPFLAGS += -D$(CONFIG_CHIP_NAME)

ifeq ($(CONFIG_CHIP_NAME),BL702)
# It is enough enable -Wundef and -Werror only on BL702 platform 
# to ensure code quality of easyflash4
CPPFLAGS += -Wundef
CPPFLAGS += -Werror
endif