#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#

PROJECT_PATH := $(abspath .)
PROJECT_NAME :=  $(notdir $(PROJECT_PATH))
PROJECT_BOARD := evb
export PROJECT_PATH PROJECT_BOARD
#CONFIG_TOOLPREFIX :=

BLECONTROLLER_LIBS := m1s1 m0s1 m0s1s m0s1p m16s1 m0s0sp
BLECONTROLLER_LIB_DEFAULT := m1s1
BLECONTROLLER_LIB_PREFIX := blecontroller_702_

-include ./proj_config.mk

ifeq ($(origin BL60X_SDK_PATH), undefined)
BL60X_SDK_PATH_GUESS ?= $(shell pwd)
BL60X_SDK_PATH ?= $(BL60X_SDK_PATH_GUESS)/../..
$(info ****** Please SET BL70X_SDK_PATH ******)
$(info ****** Trying SDK PATH [$(BL60X_SDK_PATH)])
endif

INCLUDE_COMPONENTS += bl702 bl702_std newlibc bl702_rf hosal vfs yloop utils cli blog 
INCLUDE_COMPONENTS += blmtd blfdt
INCLUDE_COMPONENTS += easyflash4 coredump mbedtls_lts
ifeq ($(CONF_ENABLE_COREDUMP), 1)
INCLUDE_COMPONENTS += coredump
endif
INCLUDE_COMPONENTS += lmac154
INCLUDE_COMPONENTS += lwip netutils

ifeq ($(CONFIG_USB_CDC),1)
INCLUDE_COMPONENTS += bl702_usb_cdc
endif

ifeq ($(CONFIG_BT),1)
ifeq ($(CONFIG_BLECONTROLLER_LIB),all)
    INCLUDE_COMPONENTS += $(addprefix $(BLECONTROLLER_LIB_PREFIX), $(BLECONTROLLER_LIBS))
else
    ifeq ($(findstring $(CONFIG_BLECONTROLLER_LIB), $(BLECONTROLLER_LIBS)),)
        INCLUDE_COMPONENTS += $(addprefix $(BLECONTROLLER_LIB_PREFIX), $(BLECONTROLLER_LIB_DEFAULT))
    else
        INCLUDE_COMPONENTS += $(addprefix $(BLECONTROLLER_LIB_PREFIX), $(CONFIG_BLECONTROLLER_LIB))
    endif
endif
INCLUDE_COMPONENTS += blestack
endif

ifeq ($(CONFIG_USE_WIFI_BR), 1)
INCLUDE_COMPONENTS += virt_net tpsync
else
INCLUDE_COMPONENTS += bl702_ethernet lwip_dhcpd blota
endif
ifeq ($(CONFIG_GEN_ROM),1)
    INCLUDE_COMPONENTS += bl702_freertos_rom
else
    ifeq ($(CONFIG_BUILD_ROM_CODE),1)
        INCLUDE_COMPONENTS += bl702_freertos
    else
        INCLUDE_COMPONENTS += bl702_freertos_rom
    endif
endif

ifeq ($(CONFIG_BT),1)
INCLUDE_COMPONENTS += blestack blecontroller blsync_ble cjson
endif

INCLUDE_COMPONENTS += openthread_port openthread_utils_bl702 openthread
INCLUDE_COMPONENTS += mdns openthread_br
INCLUDE_COMPONENTS += $(PROJECT_NAME)

CPPFLAGS += -DCFG_CPP_ENABLE=1

ifeq ($(CONFIG_LWIP_DEBUG), 1)
# LWIP_DBG_ON
CPPFLAGS += -DLWIP_DEBUG=1
CPPFLAGS += -DDHCP6_DEBUG=LWIP_DBG_ON
CPPFLAGS += -DICMP_DEBUG=LWIP_DBG_ON
CPPFLAGS += -DIP6_DEBUG=LWIP_DBG_ON
CPPFLAGS += -DUDP_DEBUG=LWIP_DBG_ON
CPPFLAGS += -DNETIF_DEBUG=LWIP_DBG_ON
CPPFLAGS += -DDHCP_DEBUG=LWIP_DBG_ON
CPPFLAGS += -DETHARP_DEBUG=LWIP_DBG_ON
endif

CPPFLAGS += -DPBUF_POOL_SIZE=20
CPPFLAGS += -DPBUF_POOL_BUFSIZE=1600
CPPFLAGS += -DCFG_TCPIP_STACKSIZE=1536

CPPFLAGS += -DOTRADIO_RX_FRAME_BUFFER_NUM=12
CPPFLAGS += -DINCLUDE_xTaskGetHandle=1

#CPPFLAGS += -DOPENTHREAD_CONFIG_LOG_LEVEL=OT_LOG_LEVEL_DEBG

include $(BL60X_SDK_PATH)/components/network/thread/openthread_br/otbr_config.mk
include $(BL60X_SDK_PATH)/components/network/thread/openthread_common.mk

include $(BL60X_SDK_PATH)/make_scripts_riscv/project.mk

EXTRA_LDFLAGS += -Wl,--print-memory-usage
ifdef CONFIG_CACHE_SIZE
EXTRA_LDFLAGS += -Wl,--defsym=__CACHE_SIZE=$(CONFIG_CACHE_SIZE)
endif
