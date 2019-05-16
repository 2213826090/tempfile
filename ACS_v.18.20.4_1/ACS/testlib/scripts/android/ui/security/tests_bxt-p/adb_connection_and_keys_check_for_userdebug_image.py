#!/usr/bin/env python

##### imports #####
import os
import sys
import time
from testlib.scripts.android.adb import adb_steps
from testlib.base.base_utils import get_args

##### initialization #####
globals().update(vars(get_args(sys.argv)))

##### test start #####
adb_steps.connect_device(serial = serial)()
adb_steps.root_connect_device(serial = serial)()
time.sleep(5)

command = "adb shell ls -la /adb_keys && adb shell cat /adb_keys"
r = os.popen(command)
info = r.readlines()
if info == None:
    raise Exception("The test result did not achieve the desired results")
##### test end #####