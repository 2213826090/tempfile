#!/usr/bin/env python

# #############################################################################
#
# @filename:    lock_screen_at_boot_time.py
#
# @description: Checks of screen lock appears at boot time.
#
# @author:      costin.carabas@intel.com
#
##############################################################################


from testlib.scripts.android.adb import adb_steps
from testlib.scripts.security.scripts import prerequisites
from testlib.base.base_utils import get_args
import sys
import time
globals().update(vars(get_args(sys.argv)))

# Run Prerequisites
prerequisites.run_prereq(serial = serial,
                         pin = "1234",
                         set_screen_lock = True,
                         require_pin_to_start_device = True)()
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
