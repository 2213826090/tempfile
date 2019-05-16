#!/usr/bin/env python

##### imports #####
import os
import sys
import time
from testlib.base.base_utils import get_args
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
	fastboot_utils.download_flash_scripts()

	fastboot_utils.creat_big_file("./temp/files/temp.txt", 0.1)
	os.system("adb push ./temp/files/temp.txt /sdcard")
	time.sleep(10)

	fastboot_utils.flash_bxt(zip_file=flash_files, serial=serial)

	file_exists = fastboot_utils.adb_command_file_or_directory_exists(name = "temp.txt", command = "adb shell ls /sdcard")
	if file_exists:
		raise Exception("The test result did not achieve the desired results")

	os.system("sudo rm -rf ./temp")

except:
	fastboot_utils.flash_bxt(zip_file=flash_files, serial=serial)
	os.system("sudo rm -rf ./temp")
	raise
##### test end #####