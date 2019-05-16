#!/usr/bin/env python

# #############################################################################
#
# @filename:    gvb_orange.py
#
# @description: Checks for the right color of the bootloader after it is
#               unlocked. Google Verified Boot. Results: Orange
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

# check for fastboot variables
fastboot_steps.change_state(serial = serial,
                            unlock_bootloader = "yes",
                            dessert = dut_dessert)()

fastboot_steps.continue_to_adb(serial = serial)()

adb_steps.check_gvb_state(serial = serial,
                          color = "orange")()
