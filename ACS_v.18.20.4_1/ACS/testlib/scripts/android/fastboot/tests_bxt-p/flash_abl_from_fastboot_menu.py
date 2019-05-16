#!/usr/bin/env python

##### imports #####
import os
import sys
import time
from testlib.base.base_utils import get_args
from testlib.scripts.android.adb import adb_steps
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

	adb_steps.reboot(command="fastboot", reboot_timeout=300, serial=serial)()
	fastboot_steps.unlock_device(serial=serial)()
	fastboot_steps.command(command="oem fw-update m1:@0 > ./temp/files/temp.txt 2>&1", serial=serial)()
	return_result = fastboot_utils.fastboot_command_result(file_name="./temp/files/temp.txt")
	if not return_result: raise Exception("The test result did not achieve the desired results")
	fastboot_steps.lock_device(serial=serial)()

	check_point1 = False
	check_point2 = False
	check_point3 = False
	check_point4 = False
	check_point5 = False

	fastboot_utils.start_minicom(serial=serial)

	os.system("fastboot reboot > /dev/null 2>&1")
	time.sleep(120)
	fastboot_utils.to_fastboot_by_script(serial=serial)

	fastboot_utils.kill_minicom()

	file_path = "./temp/files/minicom_result.txt"
	return_result = open(file_path).readlines()
	for line in return_result:
		if "Loading mmc1:/@0" in line: check_point1 = True
		if "SPI H1 signature" in line: check_point2 = True
		if "SPI H2 signature" in line: check_point3 = True
		if "FILE H1 signature" in line: check_point4 = True
		if "FILE H2 signature" in line: check_point5 = True

	if not check_point1 and not check_point2 and not check_point3 and not check_point4 and not check_point5:
		raise Exception("The test result did not achieve the desired results")

	fastboot_utils.flash_bxt(zip_file=flash_files, serial=serial)
	os.system("sudo rm -rf ./temp")

except:
	fastboot_utils.kill_minicom()
	fastboot_utils.flash_bxt(zip_file=flash_files, serial=serial)
	os.system("sudo rm -rf ./temp")
	raise
##### test end #####