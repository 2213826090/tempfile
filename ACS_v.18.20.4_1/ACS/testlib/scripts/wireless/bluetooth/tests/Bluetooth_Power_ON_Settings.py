#!/usr/bin/env python

#######################################################################
#
# @filename:    Bluetooth_Power_ON_Settings.py
# @description: Tests if Bluetooth module can be turned on from settings
# @author:      nicolas.paccou@intel.com
#
#######################################################################

import sys
from testlib.scripts.wireless.bluetooth import bluetooth_steps_old
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.ui import ui_steps

adb_steps.connect_device(serial = sys.argv[1] + ":5555")()

###############################################################
# Start the bluetooth interface from the Settings app
###############################################################
bluetooth_steps_old.set_from_settings(state = "ON")()

###############################################################
# Set the system to the initial status
###############################################################
ui_steps.press_home()()
