#!/usr/bin/env python

##### imports #####
import os
import sys
import time
import ConfigParser
from testlib.base.base_utils import get_args
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
	os.system("mkdir -p ./temp/files/flash ./temp/image/eb/testos")
	fastboot_utils.download_flash_scripts()

	conf_url = "https://shstor001.sh.intel.com/artifactory/acs_test_artifacts/OTC_Android_Auto_Test_Suite/resources/EBImage/System_FastBoot/fastboot.conf"
	fastboot_utils.download_file(url=conf_url, local_filename="./temp/fastboot.conf")
	config = ConfigParser.ConfigParser()
	config.read("./temp/fastboot.conf")
	fastboot_path = config.get("fastboot", "path")

	platform_name = fastboot_utils.get_platform_name(serial=serial)
	if platform_name == "bxtp_abl": image_platform = "image_m_bxt"
	ifwi_gr_mrb_b1_bin_path = config.get(image_platform, "eb_testos_ifwi_gr_mrb_b1_bin")
	ifwi_gr_mrb_b1_bin_name = ifwi_gr_mrb_b1_bin_path.split("/")[-1]
	fastboot_utils.download_file(url=fastboot_path+ifwi_gr_mrb_b1_bin_path, local_filename="./temp/image/eb/testos/"+ifwi_gr_mrb_b1_bin_name)
	testos_img_path = config.get(image_platform, "eb_testos_testos_img")
	testos_img_name = testos_img_path.split("/")[-1]
	fastboot_utils.download_file(url=fastboot_path+testos_img_path, local_filename="./temp/image/eb/testos/"+testos_img_name)

	fastboot_utils.make_the_zip(dir_name="./temp/image/eb/testos/", file_name="./temp/image/eb/flashfiles.zip")
	fastboot_utils.flash_bxt(flash_ioc="False", flash_ifwi="True", flash_android="False", zip_file="./temp/image/eb/flashfiles.zip", serial=serial, sleep_time=60, wait_for_adb=False)
	fastboot_utils.to_fastboot_by_script(serial=serial)
	local_steps.wait_for_fastboot(timeout=300, serial=serial)()
	fastboot_steps.flash_image(partition_name="boot", file_name="./temp/image/eb/testos/"+testos_img_name, serial=serial)()
	fastboot_steps.continue_to_adb(serial=serial)()
	time.sleep(60)
	local_steps.wait_for_adb(timeout = 300, serial=serial)()

	return_result = fastboot_utils.adb_command_process_exists(serial=serial)
	if return_result:
		raise Exception("The test result did not achieve the desired results")

	fastboot_utils.flash_bxt(zip_file=flash_files, serial=serial)
	os.system("sudo rm -rf ./temp")

except:
	fastboot_utils.flash_bxt(zip_file=flash_files, serial=serial)
	os.system("sudo rm -rf ./temp")
	raise
##### test end #####