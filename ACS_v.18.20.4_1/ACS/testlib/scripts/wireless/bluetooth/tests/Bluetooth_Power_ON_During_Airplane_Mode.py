#!/usr/bin/env python

#######################################################################
#
# @filename:    test_bluetooth_power_on_during_airplane_mode.py
# @description: Tests if Bluetooth module can be turned on after the device
#				 goes into airplane mode.
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
# Turn the bluetooth module on. It should be off from
# the last step's action
###############################################################
bluetooth_steps_old.set_from_settings(state = "ON")()

###############################################################
# Press home to take the system to initial status
###############################################################
ui_steps.press_home()()
