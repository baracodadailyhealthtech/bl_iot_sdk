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

CONFIG_USE_STDLIB_MALLOC := 0
CONFIG_COEX_ENABLE := 1

CONFIG_USB_CDC := 1

#CONFIG_LINK_RAM := 1
CONFIG_BUILD_ROM_CODE := 1

#CONFIG_DBG_RUN_ON_FPGA := 1
#CONFIG_LINK_CUSTOMER := 1

CONFIG_EASYFLASH_ENABLE:=1
CONFIG_USE_PSRAM := 0
# use internal RC32K by default; may set to 1 for better accuracy if there is XTAL32K on the board
CONFIG_USE_XTAL32K := 0
ifeq ($(CONFIG_PDS_ENABLE),1)
CONFIG_PDS_LEVEL ?= 31
ifeq ($(CONFIG_PDS_LEVEL),31)
CONFIG_PDS_CPU_PWROFF := 1
endif

ifeq ($(CONFIG_BT),1)
# use XTAL32K for ble pds31
CONFIG_USE_XTAL32K := 1
endif
endif

ifeq ($(CONFIG_BT_MESH),1)
CONFIG_BT_MESH_MOD_BIND_CB := 1
CONFIG_BT_MESH_APPKEY_ADD_CB := 1
CONFIG_BT_MESH_MOD_SUB_ADD_CB := 1
endif

# if CONFIG_PDS_CPU_PWROFF is defined, CONFIG_LINK_CUSTOMER must be defined to avoid linking the default .ld file
ifeq ($(CONFIG_PDS_CPU_PWROFF),1)
CONFIG_LINK_CUSTOMER := 1
endif

ifeq ($(CONFIG_USB_CDC),1)
CONFIG_BL702_USE_USB_DRIVER := 1
endif

CONFIG_BL702_USE_ROM_DRIVER := 1

LOG_ENABLED_COMPONENTS := hosal vfs

CONF_ENABLE_COREDUMP:=1
