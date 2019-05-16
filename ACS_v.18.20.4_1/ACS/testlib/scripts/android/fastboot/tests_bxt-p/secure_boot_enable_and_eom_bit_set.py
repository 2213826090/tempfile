#!/usr/bin/env python

##### imports #####
import os
import sys
import time
import ConfigParser
from testlib.base.base_utils import get_args
from testlib.scripts.connections.local import local_steps
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
	os.system("mkdir -p ./temp/files/flash ./temp/image/n ./temp/image/eb/user ./temp/image/eb/userdebug")
	fastboot_utils.download_flash_scripts()

	conf_url = "https://shstor001.sh.intel.com/artifactory/acs_test_artifacts/OTC_Android_Auto_Test_Suite/resources/EBImage/System_FastBoot/fastboot.conf"
	fastboot_utils.download_file(url=conf_url, local_filename="./temp/fastboot.conf")
	config = ConfigParser.ConfigParser()
	config.read("./temp/fastboot.conf")
	fastboot_path = config.get("fastboot", "path")

	platform_name = fastboot_utils.get_platform_name(serial=serial)
	if platform_name == "bxtp_abl":
		image_platform = "image_m_bxt"
		eb_user_patch = "eb_579061_user_flashfiles_zip"
		eb_userdebug_patch = "eb_579061_userdebug_flashfiles_zip"
	if platform_name == "gordon_peak":
		zip_name = fastboot_utils.get_zip_name(zip_path=flash_files)
		if zip_name == "r0_bxtp_abl":
			image_platform = "image_m_bxt"
			eb_user_patch = "eb_579061_user_flashfiles_zip"
			eb_userdebug_patch = "eb_579061_userdebug_flashfiles_zip"
		if zip_name == "gordon_peak":
			ram_value = fastboot_utils.get_bxt_ram(serial=serial)
			if ram_value == "2g": image_platform = "image_o_bxt_2g"
			if ram_value == "4g": image_platform = "image_o_bxt_4g"
			if ram_value == "8g": image_platform = "image_o_bxt_4g"
			eb_user_patch = None
			eb_userdebug_patch = "eb_615671_flashfiles_zip"

	if eb_user_patch != None:
		eb_user_patch_flashfiles_zip_path = config.get(image_platform, eb_user_patch)
		eb_user_patch_flashfiles_zip_name = eb_user_patch_flashfiles_zip_path.split("/")[-1]
		fastboot_utils.download_file(url=fastboot_path+eb_user_patch_flashfiles_zip_path, local_filename="./temp/image/eb/user/"+eb_user_patch_flashfiles_zip_name)

	eb_userdebug_patch_flashfiles_zip_path = config.get(image_platform, eb_userdebug_patch)
	eb_userdebug_patch_flashfiles_zip_name = eb_userdebug_patch_flashfiles_zip_path.split("/")[-1]
	fastboot_utils.download_file(url=fastboot_path+eb_userdebug_patch_flashfiles_zip_path, local_filename="./temp/image/eb/userdebug/"+eb_userdebug_patch_flashfiles_zip_name)

	if platform_name == "bxtp_abl":
		fastboot_utils.flash_bxt_m_fused(eb_user_patch_flashfiles_zip_name=eb_user_patch_flashfiles_zip_name, eb_userdebug_patch_flashfiles_zip_name=eb_userdebug_patch_flashfiles_zip_name, serial=serial)
	if platform_name == "gordon_peak":
		zip_name = fastboot_utils.get_zip_name(zip_path=flash_files)
		if zip_name == "r0_bxtp_abl":
			fastboot_utils.flash_bxt_m_fused(eb_user_patch_flashfiles_zip_name=eb_user_patch_flashfiles_zip_name, eb_userdebug_patch_flashfiles_zip_name=eb_userdebug_patch_flashfiles_zip_name, serial=serial)
		if zip_name == "gordon_peak":
			fastboot_utils.flash_bxt(zip_file="./temp/image/eb/userdebug/"+eb_userdebug_patch_flashfiles_zip_name, serial=serial)

	check_point1 = False

	fastboot_utils.start_minicom(serial=serial)

	local_steps.command("adb -s {} reboot".format(serial))()
	time.sleep(60)

	fastboot_utils.kill_minicom()

	file_path = "./temp/files/minicom_result.txt"
	return_result = open(file_path).readlines()
	for line in return_result:
		if "SB: On" in line: check_point1 = True

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