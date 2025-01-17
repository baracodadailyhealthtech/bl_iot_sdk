# Component Makefile
#
## These include paths would be exported to project level
COMPONENT_ADD_INCLUDEDIRS += bl702_hal

## not be exported to project level
COMPONENT_PRIV_INCLUDEDIRS +=

## This component's src
COMPONENT_SRCS := bl702_hal/bl_adc.c \
                  bl702_hal/bl_boot2.c \
                  bl702_hal/bl_chip.c \
                  bl702_hal/bl_dma.c \
                  bl702_hal/bl_dma_uart.c \
                  bl702_hal/bl_efuse.c \
                  bl702_hal/bl_emac.c \
                  bl702_hal/bl_flash.c \
                  bl702_hal/bl_gpio.c \
                  bl702_hal/bl_hbn.c \
                  bl702_hal/bl_ir.c \
                  bl702_hal/bl_irq.c \
                  bl702_hal/bl_pds.c \
                  bl702_hal/bl_pwm.c \
                  bl702_hal/bl_rtc.c \
                  bl702_hal/bl_sec.c \
                  bl702_hal/bl_sec_aes.c \
                  bl702_hal/bl_sys.c \
                  bl702_hal/bl_timer.c \
                  bl702_hal/bl_uart.c \
                  bl702_hal/bl_wdt.c \
                  bl702_hal/bl_wireless.c \
                  bl702_hal/hal_board.c \
                  bl702_hal/hal_boot2.c \
                  bl702_hal/hal_button.c \
                  bl702_hal/hal_gpio.c \
                  bl702_hal/hal_hbn.c \
                  bl702_hal/hal_hwtimer.c \
                  bl702_hal/hal_pds.c \
                  bl702_hal/hal_spi_flash.c \
                  bl702_hal/hal_sys.c \
                  bl702_hal/hal_tcal.c \
                  bl702_hal/hosal_adc.c \
                  bl702_hal/hosal_dac.c \
                  bl702_hal/hosal_dma.c \
                  bl702_hal/hosal_flash.c \
                  bl702_hal/hosal_gpio.c \
                  bl702_hal/hosal_i2c.c \
                  bl702_hal/hosal_ota.c \
                  bl702_hal/hosal_pwm.c \
                  bl702_hal/hosal_rng.c \
                  bl702_hal/hosal_rtc.c \
                  bl702_hal/hosal_spi.c \
                  bl702_hal/hosal_timer.c \
                  bl702_hal/hosal_uart.c \
                  bl702_hal/hosal_wdg.c \
                  sec_common/bl_sec_pka.c \
                  sec_common/bl_sec_aes.c \
                  sec_common/bl_sec_common.c \
                  sec_common/bl_sec_pka.c \
                  sec_common/bl_sec_sha.c \
                  sec_common/bl_sec_psk.c \

ifeq ($(CONFIG_USE_CAMERA),1)
COMPONENT_SRCS += bl702_hal/bl_cam.c
endif

ifeq ($(CONFIG_USE_PSRAM),1)
COMPONENT_SRCS +=  bl702_hal/bl_psram.c
endif

COMPONENT_SRCDIRS += bl702_hal sec_common

COMPONENT_OBJS := $(patsubst %.c,%.o, $(COMPONENT_SRCS))
COMPONENT_OBJS := $(patsubst %.S,%.o, $(COMPONENT_OBJS))

##
CPPFLAGS += -DARCH_RISCV -DBFLB_CRYPT_HARDWARE
ifndef CONFIG_USE_STD_DRIVER
CPPFLAGS += -DBFLB_USE_HAL_DRIVER
endif

ifeq ($(CONFIG_BT),1)
CPPFLAGS += -DCFG_BLE_ENABLE
endif

ifeq ($(CONFIG_BLE_MFG),1)
CPPFLAGS += -DCONFIG_BLE_MFG
endif

ifeq ($(CONFIG_ZIGBEE),1)
CPPFLAGS += -DCFG_ZIGBEE_ENABLE
endif

ifeq ($(CONFIG_PDS_ENABLE),1)
CPPFLAGS += -DCFG_PDS_ENABLE
CONFIG_PDS_LEVEL ?= 31
CPPFLAGS += -DCFG_PDS_LEVEL=$(CONFIG_PDS_LEVEL)
ifeq ($(CONFIG_PDS_LEVEL),31)
CPPFLAGS += -DCFG_PDS_OPTIMIZE
endif
CPPFLAGS += -DCONFIG_HW_SEC_ENG_DISABLE
endif

ifeq ($(CONFIG_HBN_ENABLE),1)
CPPFLAGS += -DCFG_HBN_ENABLE
CPPFLAGS += -DCFG_HBN_OPTIMIZE
endif

ifeq ($(CONFIG_USE_PSRAM),1)
CPPFLAGS += -DCFG_USE_PSRAM
endif

ifeq ($(CONFIG_USE_XTAL32K),1)
CPPFLAGS += -DCFG_USE_XTAL32K
endif
