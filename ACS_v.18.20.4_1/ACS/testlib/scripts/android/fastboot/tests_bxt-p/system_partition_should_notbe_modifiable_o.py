#!/usr/bin/env python

##### imports #####
import os
import sys
import time
from testlib.base.base_utils import get_args
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.fastboot import fastboot_utils

##### initialization #####
globals().update(vars(get_args(sys.argv)))
args = {}
for entry in script_args:
	key, val = entry.split("=")
	args[key] = val
flash_files = args["flash_files"]

##### test start #####
try:
	os.system("mkdir -p ./temp/files/flash")

	check_point1 = False

	fastboot_utils.start_minicom(serial=serial)

	adb_steps.root_connect_device(serial = serial)()
	time.sleep(5)
	adb_steps.remount(serial = serial)()
	time.sleep(30)

	fastboot_utils.kill_minicom()

	file_path = "./temp/files/minicom_result.txt"
	return_result = open(file_path).readlines()
	for line in return_result:
		if "Buffer I/O error on dev dm" in line and "lost sync page" in line: check_point1 = True

	if not check_point1:
		raise Exception("The test result did not achieve the desired results")

	os.system("sudo rm -rf ./temp")

except:
	fastboot_utils.kill_minicom()
	fastboot_utils.download_flash_scripts()
	fastboot_utils.flash_bxt(zip_file=flash_files, serial=serial)
	os.system("sudo rm -rf ./temp")
	raise
##### test end #####