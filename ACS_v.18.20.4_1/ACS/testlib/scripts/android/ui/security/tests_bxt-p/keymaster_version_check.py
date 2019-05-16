#!/usr/bin/env python

##### imports #####
import os
import sys
import time
from testlib.scripts.android.ui.security import security_utils
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.connections.local import local_steps
from testlib.base.base_utils import get_args

##### initialization #####
globals().update(vars(get_args(sys.argv)))

##### test start #####
platform_name = security_utils.get_platform_name()

if platform_name == "bxtp_abl":
	local_steps.command("adb -s {} reboot".format(serial))()
	local_steps.wait_for_adb(timeout = 300, serial=serial)()
	if platform_name == "bxtp_abl": adb_steps.wait_for_ui_processes(serial=serial)()
	if platform_name == "gordon_peak": time.sleep(20)

	result = False
	command = "adb logcat -d | grep keymaster"
	check_point = "Found keymaster1 module, using keymaster1 API."

	return_result = os.popen(command).readlines()
	for line in return_result:
	    line = line.strip("\r\n")
	    if check_point in line:
	        result = True

	if not result:
	    raise Exception("The test result did not achieve the desired results")

if platform_name == "gordon_peak":
	result = True
	lib_name = None
	if platform_name == "bxtp_abl": lib_name = "keystore.gmin.so"
	if platform_name == "gordon_peak": lib_name = "keystore.*.so"
	adb_command = "adb shell ls "
	no_such_file = ": No such file or directory"

	command = "/system/lib/hw/" + lib_name
	return_result = os.popen(adb_command + command).readlines()
	if return_result[0].strip("\r\n") == (command + no_such_file):
	    result = False

	command = "/system/lib64/hw/" + lib_name
	return_result = os.popen(adb_command + command).readlines()
	if return_result[0].strip("\r\n") == (command + no_such_file):
	    result = False

	if not result:
	    raise Exception("The test result did not achieve the desired results")
##### test end #####