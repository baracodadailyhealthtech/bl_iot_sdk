# Component Makefile
#
## These include paths would be exported to project level
COMPONENT_ADD_INCLUDEDIRS += include
COMPONENT_ADD_INCLUDEDIRS += include/arch
COMPONENT_ADD_INCLUDEDIRS += include/arch/risc-v/t-head
COMPONENT_ADD_INCLUDEDIRS += include/arch/risc-v/t-head/Core/Include

## not be exported to project level
COMPONENT_PRIV_INCLUDEDIRS :=

## This component's src
COMPONENT_SRCS :=   src/bflb_acomp.c   \
					src/bflb_cks.c     \
					src/bflb_ef_ctrl.c \
					src/bflb_gpio.c    \
					src/bflb_i2c.c     \
					src/bflb_dma.c     \
					src/bflb_rtc.c     \
					src/bflb_sec_aes.c \
					src/bflb_sec_sha.c \
					src/bflb_sec_trng.c\
					src/bflb_spi.c     \
					src/bflb_timer.c   \
					src/bflb_uart.c    \
					src/bflb_wdg.c     \
					src/bflb_adc.c     

COMPONENT_SRCS += src/bflb_irq.c src/bflb_l1c.c src/bflb_mtimer.c

CHIP_NAME = $(shell echo $(CONFIG_CHIP_NAME) | tr A-Z a-z)

COMPONENT_SRCS += config/$(CHIP_NAMFAE)/device_table.c
COMPONENT_ADD_INCLUDEDIRS += config/$(CHIP_NAME)

CFLAGS += -D$(CONFIG_CHIP_NAME)
CFLAGS += -DCONFIG_IRQ_NUM=80
CFLAGS += -DBL_IOT_SDK

ifeq ($(CONFIG_CHIP_NAME),BL702)
COMPONENT_SRCS += 	src/bflb_dac.c    \
					src/bflb_emac.c   \
					src/bflb_ir.c     \
					src/bflb_pwm_v1.c \
					src/bflb_cam.c

endif

ifeq ($(CONFIG_CHIP_NAME),BL616)
COMPONENT_SRCS += src/bflb_dac.c \
				src/bflb_emac.c \
				src/bflb_ir.c \
				src/bflb_mjpeg.c \
				src/bflb_pwm_v2.c \
				src/bflb_cam.c \
				src/bflb_sdio2.c \
				src/bflb_i2s.c \
				src/bflb_dbi.c \
				src/bflb_audac.c \
				src/bflb_auadc.c \
				src/bflb_wo.c

COMPONENT_SRCS += include/arch/risc-v/t-head/rv_hart.c
COMPONENT_SRCS += include/arch/risc-v/t-head/rv_pmp.c

ifeq ($(CONFIG_CHERRYUSB),1)
COMPONENT_SRCS += src/bflb_usb_v2.c
endif

endif

ifeq ($(CONFIG_CHIP_NAME),BL702L)
COMPONENT_SRCS += 	src/bflb_pwm_v1.c   \
					src/bflb_pwm_v2.c   \
					src/bflb_ir.c 

endif

COMPONENT_OBJS := $(patsubst %.c,%.o, $(COMPONENT_SRCS))

COMPONENT_SRCDIRS := src include/arch/risc-v/t-head config/$(CHIP_NAME)
