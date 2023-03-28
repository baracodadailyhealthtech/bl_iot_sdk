EXTRA_CPPFLAGS  ?=
ifeq ("$(CONFIG_CHIP_NAME)", "BL702")
EXTRA_CPPFLAGS  += -D BL_SDK_VER=\"release_bl_iot_sdk_1.6.40-946-g480d2dbb2-dirty\"
EXTRA_CPPFLAGS  += -D BL_SDK_STDDRV_VER=\"dff9318-dirty\"
EXTRA_CPPFLAGS  += -D BL_SDK_STDCOM_VER=\"9fe1995-dirty\"
EXTRA_CPPFLAGS  += -D BL_SDK_RF_VER=\"e203dee-dirty\"
endif
ifeq ("$(CONFIG_CHIP_NAME)", "BL702L")
EXTRA_CPPFLAGS  += -D BL_SDK_VER=\"release_bl_iot_sdk_1.6.40-946-g480d2dbb2-dirty\"
EXTRA_CPPFLAGS  += -D BL_SDK_STDDRV_VER=\"6bf6ea7-dirty\"
EXTRA_CPPFLAGS  += -D BL_SDK_STDCOM_VER=\"88edf7b-dirty\"
EXTRA_CPPFLAGS  += -D BL_SDK_RF_VER=\"365f865-dirty\"
endif
