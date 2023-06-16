#
#compiler flag config domain
#

#
#board config domain
#


#
#app specific config domain
#

CONFIG_CHIP_NAME := BL702
CONFIG_USB_CDC := 0
CONFIG_USE_STDLIB_MALLOC := 0
CONFIG_BUILD_ROM_CODE := 1

ifeq ($(CONFIG_USB_CDC),1)
CONFIG_BL702_USE_USB_DRIVER := 1
endif

CONFIG_BL702_USE_ROM_DRIVER := 1

CONFIG_USE_CAMERA := 1
CONFIG_USE_PSRAM := 1

LOG_ENABLED_COMPONENTS := hosal vfs
