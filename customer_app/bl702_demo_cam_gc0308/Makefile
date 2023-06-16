#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#

PROJECT_PATH := $(abspath .)
PROJECT_NAME := $(notdir $(PROJECT_PATH))
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
INCLUDE_COMPONENTS += easyflash4
INCLUDE_COMPONENTS += bl702_freertos
#INCLUDE_COMPONENTS += camera_source

ifeq ($(CONFIG_USB_CDC),1)
INCLUDE_COMPONENTS += bl702_usb_cdc
endif

ifeq ($(CONFIG_BT),1)
ifeq ($(findstring $(CONFIG_BLECONTROLLER_LIB), $(BLECONTROLLER_LIBS)),)
	INCLUDE_COMPONENTS += $(addprefix $(BLECONTROLLER_LIB_PREFIX), $(BLECONTROLLER_LIB_DEFAULT))
else
	INCLUDE_COMPONENTS += $(addprefix $(BLECONTROLLER_LIB_PREFIX), $(CONFIG_BLECONTROLLER_LIB))
endif
INCLUDE_COMPONENTS += blestack
endif

INCLUDE_COMPONENTS += $(PROJECT_NAME)
include $(BL60X_SDK_PATH)/make_scripts_riscv/project.mk
