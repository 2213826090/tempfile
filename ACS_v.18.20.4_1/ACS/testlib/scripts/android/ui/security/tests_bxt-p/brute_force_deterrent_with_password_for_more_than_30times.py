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
security_steps.set_password_screen_lock(serial = serial)()
security_steps.enter_incorrect_password(serial = serial, dut_info = "bxt",
										dut_pin = "test1234", tries = 30)()
security_steps.enter_correct_password(serial = serial, dut_info = "bxt", dut_pin = "test1234")()
security_steps.remove_password_screen_lock(serial = serial)()

##### test end #####