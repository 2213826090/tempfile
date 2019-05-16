#!/usr/bin/env python

##### imports #####
import os
import sys
import time
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.android.adb import adb_steps
from testlib.base.base_utils import get_args
from testlib.scripts.android.ui.security import security_steps
from testlib.scripts.android.ui.security import security_utils

##### initialization #####
globals().update(vars(get_args(sys.argv)))

##### test start #####
platform_name = security_utils.get_platform_name()

adb_steps.root_connect_device(serial = serial)()
time.sleep(5)

command = "adb shell getprop | grep crypt"
result = False
r = os.popen(command)
info = r.readlines()
for line in info:
    line = line.strip("\r\n")
    if "[ro.crypto.state]: [encrypted]" in line:
        result = True
if not result:
    raise Exception("The test result did not achieve the desired results")

command = "adb shell cat /fstab." + platform_name + " | grep forceencrypt"
result = False
r = os.popen(command)
info = r.readlines()
for line in info:
    line = line.strip("\r\n")
    line = line.split()
    for l in line:
        if "noatime,nosuid,nodev,discard,noauto_da_alloc,errors=panic" in l:
            result = True
if not result:
    raise Exception("The test result did not achieve the desired results")

command = "adb shell mount | grep /data | grep /dev/block/dm"
result = False
r = os.popen(command)
info = r.readlines()
for line in info:
    line = line.strip("\r\n")
    line = line.split()
    for l in line:
        if "rw,seclabel,nosuid,nodev,noatime,discard,noauto_da_alloc,errors=panic,data=ordered" in l:
            result = True
if not result:
    raise Exception("The test result did not achieve the desired results")

security_steps.reboot_system(serial = serial)()
ui_steps.open_security_settings(serial = serial)()
security_steps.disk_encryption_flag_check(serial = serial, platform_name = platform_name)()
##### test end #####