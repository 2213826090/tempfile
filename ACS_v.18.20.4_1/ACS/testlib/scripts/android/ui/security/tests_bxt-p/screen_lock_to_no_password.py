#!/usr/bin/env python

##### imports #####
import sys
from testlib.base import base_utils
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.ui.security import security_steps

##### initialization #####
globals().update(vars(base_utils.get_args(sys.argv)))
adb_steps.connect_device(serial = serial, port = adb_server_port)()

#### test start ####

security_steps.reboot_system(serial = serial)()
ui_steps.open_security_settings(serial = serial)()
security_steps.set_pin_screen_lock(serial = serial)()
security_steps.screen_lock_to_no_password(serial = serial, dut_info = "bxt",
										  dut_pin = "1234", tries = 1)()

##### test end #####