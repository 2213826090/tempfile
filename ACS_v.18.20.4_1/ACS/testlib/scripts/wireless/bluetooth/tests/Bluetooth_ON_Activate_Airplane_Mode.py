#!/usr/bin/env python

#######################################################################
#
# @filename:    Bluetooth_ON_Activate_Airplane_Mode.py
# @description: Tests if WiFi module is turned off when Airplane mode
#				is turned on
# @author:      nicolas.paccou@intel.com
#
#######################################################################

import sys
from testlib.scripts.wireless.bluetooth import bluetooth_steps_old
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.ui import ui_steps

adb_steps.connect_device(serial = sys.argv[1] + ":5555")()

###############################################################
# Turn on the airplane mode
###############################################################
bluetooth_steps_old.set_airplane_mode(state = '1')()

###############################################################
# Take the system to the initial status
###############################################################
ui_steps.press_home()()
