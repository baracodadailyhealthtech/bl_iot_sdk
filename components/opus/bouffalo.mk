# Component Makefile
#
## These include paths would be exported to project level
COMPONENT_ADD_INCLUDEDIRS += src celt

## not be exported to project level
COMPONENT_PRIV_INCLUDEDIRS := silk silk/fixed

## This component's src
COMPONENT_SRCDIRS := src celt silk silk/fixed

##
CFLAGS += -DHAVE_CONFIG_H
CPPFLAGS += -DHAVE_CONFIG_H
