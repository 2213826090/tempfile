#!/usr/bin/env python

##### imports #####
import sys
from testlib.base import base_utils
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.ui.security import security_steps
from testlib.scripts.android.ui.security import security_utils

##### initialization #####
globals().update(vars(base_utils.get_args(sys.argv)))
adb_steps.connect_device(serial = serial, port = adb_server_port)()

#### test start ####
platform_name = security_utils.get_platform_name()

if platform_name == "bxtp_abl":
	security_steps.initialize_environment(serial = serial)()

if platform_name == "gordon_peak":
	security_steps.initialize_environment_on_ivio(serial = serial)()

##### test end #####