#!/usr/bin/env python

# #############################################################################
#
# @filename:    unlock_to_lock_switch.py
#
# @description: Switches from unlocked to lock bootloader states.
#
# @author:      costin.carabas@intel.com
#
##############################################################################


from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.adb import adb_utils
from testlib.scripts.android.fastboot import fastboot_steps
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.security.scripts import prerequisites
from testlib.base.base_utils import get_args
import sys
globals().update(vars(get_args(sys.argv)))

dut_dessert = adb_utils.get_android_version(serial = serial)

# Run Prerequisites
prerequisites.run_prereq(serial = serial,
                         pin = "1234")()

ui_steps.enable_oem_unlock(serial = serial, enabled = False, blocking = True)()

# reboot to fastboot
adb_steps.reboot(serial = serial,
                command = "fastboot",
                ip_enabled = False,
                blocking = True)()

fastboot_steps.change_state(serial = serial,
                            unlock_bootloader = "no",
                            dessert = dut_dessert)()

fastboot_steps.continue_to_adb(serial = serial)()
adb_steps.wait_for_ui(serial = serial)()
