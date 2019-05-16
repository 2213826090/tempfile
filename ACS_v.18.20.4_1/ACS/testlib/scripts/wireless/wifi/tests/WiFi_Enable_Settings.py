#!/usr/bin/env python

#######################################################################
#
# @filename:    WiFi_Enable_Settings.py
# @description: Tests if WiFi switch can be turned on from settings
# @author:      ion-horia.petrisor@intel.com
#
#######################################################################


import sys
from testlib.scripts.wireless.wifi import wifi_steps
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.ui import ui_steps

adb_steps.connect_device(serial = sys.argv[1] + ":5555")()

wifi_steps.set_from_wifi_settings()()

ui_steps.press_home()()
