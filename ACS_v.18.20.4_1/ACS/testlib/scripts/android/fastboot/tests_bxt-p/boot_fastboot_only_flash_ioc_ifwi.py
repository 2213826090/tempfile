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
from testlib.scripts.connections.local import local_utils

##### initialization #####
globals().update(vars(get_args(sys.argv)))
args = {}
for entry in script_args:
	key, val = entry.split("=")
	args[key] = val
flash_files = args["flash_files"]

##### test start #####
try:
	os.system("mkdir -p ./temp/files/flash ./temp/image/n")
	fastboot_utils.download_flash_scripts()

	adb_steps.reboot(command = "fastboot", reboot_timeout = 300, serial = serial)()
	fastboot_steps.corrupt_esp_partition(unlock_dut=True, lock_dut=False, reboot=False, serial=serial)()
	fastboot_utils.flash_bxt(flash_ioc="True", flash_ifwi="True", flash_android="False", zip_file=flash_files, serial=serial, sleep_time=60, wait_for_adb=False)

	if serial in local_utils.get_connected_android_devices()['android']:
		raise Exception("The test result did not achieve the desired results")

	fastboot_utils.to_fastboot_by_script(serial=serial)
	local_steps.wait_for_fastboot(timeout=300, serial=serial)()
	os.system("fastboot reboot > /dev/null 2>&1")
	time.sleep(60)

	if serial not in local_utils.get_connected_android_devices()['fastboot']:
		raise Exception("The test result did not achieve the desired results")

	fastboot_utils.flash_bxt(zip_file=flash_files, serial=serial)
	os.system("sudo rm -rf ./temp")

except:
	fastboot_utils.flash_bxt(zip_file=flash_files, serial=serial)
	os.system("sudo rm -rf ./temp")
	raise
##### test end #####