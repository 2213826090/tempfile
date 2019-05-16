#!/usr/bin/env python

#######################################################################
#
# @filename:    Bluetooth_Power_OFF_Settings.py
# @description: Tests if Bluetooth module is turned off when disabling it
#                  from the Settings application menu
# @author:      nicolas.paccou@intel.com
#
#######################################################################

import sys
from testlib.scripts.wireless.bluetooth import bluetooth_steps_old
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.ui import ui_steps

adb_steps.connect_device(serial = sys.argv[1] + ":5555")()

###############################################################
# Turn the bluetooth module off from the Settings app
###############################################################
bluetooth_steps_old.set_from_settings(state = "OFF")()

###############################################################
# Reset the system to initial status
###############################################################
ui_steps.press_home()()
