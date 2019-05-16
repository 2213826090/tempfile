#!/usr/bin/env python

##### imports #####
import os
import sys
import time
import ConfigParser
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
partition_name = args["partition_name"]
file_name = args["file_name"]

##### test start #####
try:
	os.system("mkdir -p ./temp/files/flash ./temp/image/n ./temp/image/n-1")
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
	n_1_flashfiles_zip_path = config.get(image_platform, "n_1_flashfiles_zip")
	n_1_flashfiles_zip_name = n_1_flashfiles_zip_path.split("/")[-1]
	fastboot_utils.download_file(url = fastboot_path + n_1_flashfiles_zip_path, local_filename = "./temp/image/n-1/" + n_1_flashfiles_zip_name)

	fastboot_utils.flash_bxt(zip_file="./temp/image/n-1/"+n_1_flashfiles_zip_name, serial=serial)

	fastboot_utils.unpack_the_zip(file_name = flash_files, temp_path = r"./temp/image/n/flashfiles")
	adb_steps.reboot(command = "fastboot", reboot_timeout = 300, serial = serial)()
	fastboot_steps.flash_image(partition_name = partition_name, file_name = "./temp/image/n/flashfiles/"+file_name, serial = serial)()
	fastboot_steps.continue_to_adb(serial=serial)()
	time.sleep(60)
	local_steps.wait_for_adb(timeout = 300, serial=serial)()

	fastboot_utils.flash_bxt(zip_file=flash_files, serial=serial)
	os.system("sudo rm -rf ./temp")

except:
	fastboot_utils.flash_bxt(zip_file=flash_files, serial=serial)
	os.system("sudo rm -rf ./temp")
	raise
##### test end #####