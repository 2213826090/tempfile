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
test_case_list = {"tipc_test_connect_10": "tipc-test -t connect -r 10", "tipc_test_connect_foo_10": "tipc-test -t connect_foo -r 10", \
		 "tipc_test_echo_10": "tipc-test -t echo -r 10", "tipc_test_select_10": "tipc-test -t select -r 10", \
		 "tipc_test_blocked_read": "tipc-test -t blocked_read", "tipc_test_closer1_10": "tipc-test -t closer1 -r 10", \
		 "tipc_test_closer2_10": "tipc-test -t closer2 -r 10", "tipc_test_dev_uuid": "tipc-test -t dev-uuid", \
		 "tipc_test_ta_access": "tipc-test -t ta-access", "tipc_test_closer3_10": "tipc-test -t closer3 -r 10", \
		 "tipc_test_connect_64_64": "tipc-test -t connect -m 64 -b 64"}
test_result_list = {}
fail_case_list = {}

try:
	conf_url = "https://shstor001.sh.intel.com/artifactory/acs_test_artifacts/OTC_Android_Auto_Test_Suite/resources/EBImage/Security/M/security.conf"
	os.system("mkdir -p ./temp/files ./temp/tipc ./temp/new_image")
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
	tipc_file_path = config.get("tipc", "file_path")
	tipc_file_name = tipc_file_path.split("/")[-1]
	fastboot_utils.download_file(url = security_path + tipc_file_path, local_filename = "./temp/tipc/" + tipc_file_name)
	tipc_zip_path = config.get("tipc", "zip_path")
	tipc_zip_name = tipc_zip_path.split("/")[-1]
	fastboot_utils.download_file(url = security_path + tipc_zip_path, local_filename = "./temp/tipc/" + tipc_zip_name)

	fastboot_utils.unpack_the_zip(file_name = flash_files, temp_path = r"./temp/new_image")

	fastboot_utils.unpack_the_zip(file_name = "./temp/tipc/" + tipc_zip_name, temp_path = r"./temp/tipc")
	os.system("sudo python ./temp/files/flash_image.py --bin ./temp/tipc/ifwi_gr_mrb_b1.bin --zip ./temp/tipc/" + tipc_zip_name)
	time.sleep(60)
	local_steps.wait_for_adb(timeout = 300, serial=serial)()

	os.system("sudo chmod 777 ./temp/tipc/" + tipc_file_name)
	adb_steps.root_connect_device(serial = serial)()
	time.sleep(5)
	adb_steps.push_file(local = "./temp/tipc/" + tipc_file_name, remote = "/data/", serial = serial)()

	for key, value in test_case_list.items():
		if key == "tipc_test_blocked_read": continue
		test_result_list[key] = fastboot_utils.tipc_negativetipc_execution_result(case_type = "tipc_test", command = "adb shell ./data/tipc-test32 " + value)

	print "The results of the tipc test case execution are as follows:"
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