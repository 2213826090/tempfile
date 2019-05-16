#!/usr/bin/env python

##### imports #####
import os
import sys
import ConfigParser
from testlib.base.base_utils import get_args
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
	os.system("mkdir -p ./temp/files/flash ./temp/files/resources")
	conf_url = "https://shstor001.sh.intel.com/artifactory/acs_test_artifacts/OTC_Android_Auto_Test_Suite/resources/EBImage/System_FastBoot/fastboot.conf"
	fastboot_utils.download_file(url=conf_url, local_filename="./temp/fastboot.conf")
	config = ConfigParser.ConfigParser()
	config.read("./temp/fastboot.conf")
	fastboot_path = config.get("fastboot", "path")

	test_apk_path = config.get("files", "test_apk")
	test_apk_name = test_apk_path.split("/")[-1]
	fastboot_utils.download_file(url = fastboot_path + test_apk_path, local_filename = "./temp/files/resources/" + test_apk_name)

	fastboot_utils.push_uiautomator_jar(serial = serial)
	fastboot_steps.config_first_boot_wizard(serial = serial)()

	platform_name = fastboot_utils.get_platform_name(serial=serial)
	if platform_name == "bxtp_abl":
		return_result = fastboot_utils.adb_install_apk(platform_name = platform_name, apk_path = "./temp/files/resources/" + test_apk_name, serial = serial)
	if platform_name == "gordon_peak":
		install_command = "adb -s {0} install ./temp/files/resources/{1}  > ./temp/files/temp.txt 2>&1".format(serial, test_apk_name)
		os.system(install_command)
		return_result = fastboot_utils.adb_install_apk(platform_name = platform_name, file_path = "./temp/files/temp.txt")

	if not return_result:
		raise Exception("The test result did not achieve the desired results")

	fastboot_steps.factory_data_reset(serial = serial)()

	file_exists = fastboot_utils.adb_command_file_or_directory_exists(name = test_apk_name[:-4], command = "adb shell ls /data/app")
	if file_exists:
		raise Exception("The test result did not achieve the desired results")

	os.system("sudo rm -rf ./temp")

except:
	fastboot_utils.download_flash_scripts()
	fastboot_utils.flash_bxt(zip_file=flash_files, serial=serial)
	os.system("sudo rm -rf ./temp")
	raise
##### test end #####