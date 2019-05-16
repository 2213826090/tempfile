#!/usr/bin/env python

##### imports #####
import os
import sys
import time
import ConfigParser
from testlib.base.base_utils import get_args
from testlib.scripts.android.adb import adb_steps
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
test_result_list = {}
fail_case_list = {}

try:
	conf_url = "https://shstor001.sh.intel.com/artifactory/acs_test_artifacts/OTC_Android_Auto_Test_Suite/resources/EBImage/Security/M/security.conf"
	os.system("mkdir -p ./temp/files ./temp/negative ./temp/new_image")
	fastboot_utils.download_file(url = conf_url, local_filename = "./temp/security.conf")
	config = ConfigParser.ConfigParser()
	config.read("./temp/security.conf")
	security_path = config.get("security", "path")

	boot_device_script_path = config.get("files", "boot_device_script")
	boot_device_script_name = boot_device_script_path.split("/")[-1]
	fastboot_utils.download_file(url = security_path + boot_device_script_path, local_filename = "./temp/files/" + boot_device_script_name)
	flash_image_script_path = config.get("files", "flash_image_script")
	flash_image_script_name = flash_image_script_path.split("/")[-1]
	fastboot_utils.download_file(url = security_path + flash_image_script_path, local_filename = "./temp/files/" + flash_image_script_name)
	negative_file_path = config.get("negative", "file_path")
	negative_file_name = negative_file_path.split("/")[-1]
	fastboot_utils.download_file(url = security_path + negative_file_path, local_filename = "./temp/negative/" + negative_file_name)
	negative_zip_path = config.get("negative", "zip_path")
	negative_zip_name = negative_zip_path.split("/")[-1]
	fastboot_utils.download_file(url = security_path + negative_zip_path, local_filename = "./temp/negative/" + negative_zip_name)

	fastboot_utils.unpack_the_zip(file_name = flash_files, temp_path = r"./temp/new_image")

	fastboot_utils.unpack_the_zip(file_name = "./temp/negative/" + negative_zip_name, temp_path = r"./temp/negative")
	os.system("sudo python ./temp/files/flash_image.py --bin ./temp/negative/ifwi_gr_mrb_b1.bin --zip ./temp/negative/" + negative_zip_name)
	time.sleep(60)
	local_steps.wait_for_adb(timeout = 300, serial=serial)()

	os.system("sudo chmod 777 ./temp/negative/" + negative_file_name)
	adb_steps.root_connect_device(serial = serial)()
	time.sleep(5)
	adb_steps.push_file(local = "./temp/negative/" + negative_file_name, remote = "/data/", serial = serial)()

	return_result = os.popen("adb shell ./data/negative_ca.trusty32").readlines()
	for line in return_result:
		if "-" not in line: continue
		case_name = line.split("-")[0].strip()
		if case_name == "neg_api_send_msg" or case_name == "neg_scene_ta_calls_after_ta_close_chan": continue
		test_result_list[case_name] = fastboot_utils.tipc_negativetipc_execution_result(case_type = "negative_tipc_test", command = "adb shell ./data/negative_ca.trusty32 " \
												+ case_name, case_name = case_name)

	print "The results of the negative tipc test case execution are as follows:"
	for key, value in test_result_list.items():
		if value == False: fail_case_list[key] = value
		print "Test case name: " + key + "and test case result: " + str(value)

	if fail_case_list: raise Exception("The test result did not achieve the desired results")

	os.system("sudo python ./temp/files/flash_image.py --bin ./temp/new_image/ifwi_gr_mrb_b1.bin --zip " + flash_files)
	time.sleep(60)
	local_steps.wait_for_adb(timeout = 300, serial=serial)()
	os.system("sudo rm -rf ./temp")

except:
	os.system("sudo python ./temp/files/flash_image.py --bin ./temp/new_image/ifwi_gr_mrb_b1.bin --zip " + flash_files)
	time.sleep(60)
	local_steps.wait_for_adb(timeout = 300, serial=serial)()
	os.system("sudo rm -rf ./temp")
	raise
##### test end #####