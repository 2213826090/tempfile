#!/usr/bin/env python

##### imports #####
import os
import sys
from testlib.base import base_utils
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.ui.security import security_steps
from testlib.scripts.android.ui.security import security_utils

##### initialization #####
globals().update(vars(base_utils.get_args(sys.argv)))
adb_steps.connect_device(serial = serial, port = adb_server_port)()

#### test start ####
platform_name = security_utils.get_platform_name()

if platform_name == "bxtp_abl":
    security_steps.reboot_system(serial = serial)()
    ui_steps.open_security_settings(serial = serial)()
    security_steps.hardware_backed_keystore(serial = serial)()

if platform_name == "gordon_peak":
    result = True
    adb_command = "adb shell ls "
    no_such_file = ": No such file or directory"

    command = "/system/lib/hw/keystore.*.so"
    return_result = os.popen(adb_command + command).readlines()
    if return_result[0].strip("\r\n") == (command + no_such_file):
        result = False

    command = "/system/lib64/hw/keystore.*.so"
    return_result = os.popen(adb_command + command).readlines()
    if return_result[0].strip("\r\n") == (command + no_such_file):
        result = False

    if not result:
        raise Exception("The test result did not achieve the desired results")
##### test end #####