#!/usr/bin/env python

##### imports #####
import os
import sys
import time
from testlib.base.base_utils import get_args
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.connections.local import local_steps
from testlib.scripts.android.fastboot import fastboot_steps
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

	adb_steps.reboot(command = "fastboot", reboot_timeout = 300, serial = serial)()
	fastboot_steps.unlock_device(serial = serial)()
	fastboot_steps.lock_device(serial = serial)()
	fastboot_steps.reboot_fastboot(serial = serial)()

	fastboot_steps.command(command = "getvar device-state > ./temp/files/device_state.txt 2>&1", serial = serial)()
	device_state = fastboot_utils.get_device_state(file_path = "./temp/files/device_state.txt")
	if device_state == "unlocked": raise Exception("The test result did not achieve the desired results")

	fastboot_steps.continue_to_adb(serial=serial)()
	time.sleep(60)
	local_steps.wait_for_adb(serial = serial, timeout = 300)()

	fastboot_utils.flash_bxt(zip_file=flash_files, serial=serial)
	os.system("sudo rm -rf ./temp")

except:
	fastboot_utils.flash_bxt(zip_file=flash_files, serial=serial)
	os.system("sudo rm -rf ./temp")
	raise
##### test end #####