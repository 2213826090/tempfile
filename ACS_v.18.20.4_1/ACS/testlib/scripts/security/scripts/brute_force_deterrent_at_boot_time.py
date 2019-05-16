#!/usr/bin/env python

# #############################################################################
#
# @filename:    brute_force_deterrent_at_boot_time.py
#
# @description: Enter wrong PIN 10 times in a row at boot time
#
# @author:      costin.carabas@intel.com
#
##############################################################################

from testlib.scripts.android.adb import adb_steps
from testlib.base.base_utils import get_args
from testlib.scripts.security.scripts import prerequisites
import sys
import time
globals().update(vars(get_args(sys.argv)))

#Run Prerequisites
prerequisites.run_prereq(serial = serial,
                         pin = "1234",
                         set_screen_lock = True,
                         require_pin_to_start_device = True,
                         blocking = True)()

time.sleep(5)

adb_steps.reboot(serial = serial,
                no_ui = True,
                boot_to_Android = False,
                pin = "1234",
                blocking = True)()

adb_steps.wait_for_ui_processes(serial=serial,
                                imeout=30000)()

adb_steps.wait_for_text(serial=serial,
                        text_to_find="To start Android, enter your PIN")()

#Check feature: Enter wrong PIN 10 times in a row
adb_steps.block_device_at_boot_time(serial = serial, pin = "2222")()

#Reboot to Android
adb_steps.reboot(serial = serial,
                pin = "1234")()
