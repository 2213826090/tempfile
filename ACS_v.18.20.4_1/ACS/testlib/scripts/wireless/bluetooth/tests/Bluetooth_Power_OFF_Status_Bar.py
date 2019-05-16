#!/usr/bin/env python

#######################################################################
#
# @filename:    Bluetooth_Power_OFF_Status_Bar.py
# @description: Tests if Bluetooth module is turned off when disabling
#                  it from the upper-right screen status bar
# @author:      nicolas.paccou@intel.com
#
#######################################################################

import sys
from testlib.scripts.wireless.bluetooth import bluetooth_steps_old
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.ui import ui_steps

adb_steps.connect_device(serial = sys.argv[1] + ":5555")()

###############################################################
# Disable the bluetooth module from the status bar
###############################################################
bluetooth_steps_old.bluetooth_set_from_status_bar(state = "OFF")()

###############################################################
# Reset the system to initial status
###############################################################
ui_steps.press_home()()
