# -*- coding: utf-8 -*-
import os

NOSERUNNER = "NOSERUNNER"

SDP = "USB"
CDP = "USB_CDP"
DCP = "USB_DCP"
AC = "AC"

# default config
TESTLIB_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CONFIG = os.path.join(TESTLIB_DIR, "configures.conf")

# Build version
O_MR1 = "8.1.0"
O_MR0 = "8.0.0"

# Product name
BXT_M = "r0_bxtp_abl"
BXT_O = "gordon_peak"
CHT_T4 = "r2_cht_ffd"
AIA_CERULEAN = "androidia"
AIA_CELADON = "celadon"

# permissions
PERMISSION_ACCESS_FINE_LOCATION = "android.permission.ACCESS_FINE_LOCATION"
PERMISSION_CALL_PHONE = "android.permission.CALL_PHONE"
PERMISSION_CAMERA = "android.permission.CAMERA"
PERMISSION_READ_EXTERNAL_STORAGE = "android.permission.READ_EXTERNAL_STORAGE"
PERMISSION_RECORD_AUDIO = "android.permission.RECORD_AUDIO"
PERMISSION_WRITE_EXTERNAL_STORAGE = "android.permission.WRITE_EXTERNAL_STORAGE"

