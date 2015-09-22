# This file is a makefile included from the top level makefile which
# defines the sources built for the target.

# Define the prefix to this directory.
# Note: The name must be unique within this build and should be
#       based on the root of the project
TARGET_USB_HS_SRC_PATH = $(TARGET_USB_HS_PATH)/src

# C source files included in this build.
CSRC += $(TARGET_USB_HS_SRC_PATH)/usbh_core.c
CSRC += $(TARGET_USB_HS_SRC_PATH)/usbh_hcs.c
CSRC += $(TARGET_USB_HS_SRC_PATH)/usbh_ioreq.c
CSRC += $(TARGET_USB_HS_SRC_PATH)/usbh_stdreq.c
CSRC += $(TARGET_USB_HS_SRC_PATH)/usbh_hid_core.c
CSRC += $(TARGET_USB_HS_SRC_PATH)/usbh_hid_keybd.c
CSRC += $(TARGET_USB_HS_SRC_PATH)/usbh_hid_mouse.c

# C++ source files included in this build.
CPPSRC +=

# ASM source files included in this build.
ASRC +=

