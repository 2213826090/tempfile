#!/usr/bin/env python

##### imports #####
import os
import sys
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
	os.system("mkdir -p ./temp/files/flash ./temp/image/n")
	fastboot_utils.download_flash_scripts()

	check_point1 = False
	fastboot_utils.start_minicom(serial=serial)
	fastboot_utils.flash_bxt(flash_ioc="True", flash_ifwi="True", flash_android="False", zip_file=flash_files, serial=serial, sleep_time=60, wait_for_adb=False)
	fastboot_utils.to_fastboot_by_script(serial=serial)
	fastboot_utils.kill_minicom()

	file_path = "./temp/files/minicom_result.txt"
	return_result = open(file_path).readlines()
	for line in return_result:
		if "image#2 - ELK: copy ELK from SPI" in line: check_point1 = True

	if not check_point1:
		raise Exception("The test result did not achieve the desired results")

	fastboot_utils.flash_bxt(zip_file=flash_files, serial=serial)
	os.system("sudo rm -rf ./temp")

except:
	fastboot_utils.kill_minicom()
	fastboot_utils.flash_bxt(zip_file=flash_files, serial=serial)
	os.system("sudo rm -rf ./temp")
	raise
##### test end #####