#!/usr/bin/env python

##### imports #####
import os
import sys
import time
from testlib.base.base_utils import get_args
from testlib.scripts.android.adb import adb_steps

##### initialization #####
globals().update(vars(get_args(sys.argv)))

##### test start #####
adb_steps.connect_device(serial = serial)()
adb_steps.root_connect_device(serial = serial)()
time.sleep(5)
adb_steps.remount(should_remount = True, serial = serial)()
time.sleep(5)

return_result = os.popen("adb shell mount | grep system").readlines()
if "seclabel,relatime,data=ordered" not in return_result[0].strip("\r\n"):
	raise Exception("The test result did not achieve the desired results")
##### test end #####