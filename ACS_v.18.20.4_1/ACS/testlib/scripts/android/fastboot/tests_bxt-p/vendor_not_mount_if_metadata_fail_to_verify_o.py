#!/usr/bin/env python

##### imports #####
import os
import sys
import time
import ConfigParser
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
	os.system("mkdir -p ./temp/files/flash ./temp/image/n/img")
	fastboot_utils.download_flash_scripts()

	conf_url = "https://shstor001.sh.intel.com/artifactory/acs_test_artifacts/OTC_Android_Auto_Test_Suite/resources/EBImage/System_FastBoot/fastboot.conf"
	fastboot_utils.download_file(url=conf_url, local_filename="./temp/fastboot.conf")
	config = ConfigParser.ConfigParser()
	config.read("./temp/fastboot.conf")
	fastboot_path = config.get("fastboot", "path")

	platform_name = fastboot_utils.get_platform_name(serial=serial)
	if platform_name == "bxtp_abl": image_platform = "image_m_bxt"
	if platform_name == "gordon_peak":
		ram_value = fastboot_utils.get_bxt_ram(serial=serial)
		if ram_value == "2g": image_platform = "image_o_bxt_2g"
		if ram_value == "4g": image_platform = "image_o_bxt_4g"
		if ram_value == "8g": image_platform = "image_o_bxt_4g"
	vbmeta_corrupted_vendor_metadata_path = config.get(image_platform, "vbmeta_corrupted_vendor_metadata")
	vbmeta_corrupted_vendor_metadata_name = vbmeta_corrupted_vendor_metadata_path.split("/")[-1]
	fastboot_utils.download_file(url = fastboot_path + vbmeta_corrupted_vendor_metadata_path, local_filename = "./temp/image/n/img/" + vbmeta_corrupted_vendor_metadata_name)

	adb_steps.reboot(command = "fastboot", reboot_timeout = 300, serial = serial)()
	fastboot_steps.fastboot_erase_partition(partition_name = "vbmeta", reboot = False, serial = serial)()
	fastboot_steps.flash_image(partition_name = "vbmeta", file_name = "./temp/image/n/img/"+vbmeta_corrupted_vendor_metadata_name, lock_dut = True, serial = serial)()

	check_point1 = False
	check_point2 = False

	fastboot_utils.start_minicom(serial=serial)

	os.system("fastboot reboot > /dev/null 2>&1")
	time.sleep(60)
	fastboot_utils.to_fastboot_by_script(serial=serial)

	fastboot_utils.kill_minicom()

	file_path = "./temp/files/minicom_result.txt"
	return_result = open(file_path).readlines()
	for line in return_result:
		if "ERROR: Padding check failed" in line: check_point1 = True
		if "Error verifying vbmeta image: SIGNATURE_MISMATCH" in line: check_point2 = True

	if not check_point1 or not check_point2:
		raise Exception("The test result did not achieve the desired results")

	fastboot_utils.flash_bxt(zip_file=flash_files, serial=serial)
	os.system("sudo rm -rf ./temp")

except:
	fastboot_utils.kill_minicom()
	fastboot_utils.flash_bxt(zip_file=flash_files, serial=serial)
	os.system("sudo rm -rf ./temp")
	raise
##### test end #####