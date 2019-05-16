#!/usr/bin/env python

##### imports #####
import sys
from testlib.base import base_utils
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.ui.security import security_steps

##### initialization #####
globals().update(vars(base_utils.get_args(sys.argv)))
adb_steps.connect_device(serial = serial, port = adb_server_port)()

#### test start ####

security_steps.reboot_system(serial = serial)()
security_steps.screen_lock_update(serial = serial, dut_info = "bxt", dut_pin = "1234",
								  dut_password = "test1234", tries = 1, times = 10)()

##### test end #####