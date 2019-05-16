#!/usr/bin/env python

##### imports #####
import os
import sys
from testlib.base.base_utils import get_args
from testlib.scripts.android.fastboot import fastboot_steps
from testlib.scripts.android.fastboot import fastboot_utils
from testlib.scripts.relay import relay_steps

##### initialization #####
globals().update(vars(get_args(sys.argv)))
args = {}
for entry in script_args:
	key, val = entry.split("=")
	args[key] = val
relay_type = args["relay_type"]
relay_port = args["relay_port"]
power_port = args["power_port"]

##### test start #####
try:
	fastboot_utils.push_uiautomator_jar(serial = serial)
	fastboot_steps.config_first_boot_wizard(serial = serial)()

	os.system("mkdir -p ./temp/files/flash")
	fastboot_utils.creat_big_file("./temp/files/temp.txt", 0.1)
	os.system("adb push ./temp/files/temp.txt /data")

	fastboot_steps.factory_data_reset(serial = serial)()

	file_exists = fastboot_utils.adb_command_file_or_directory_exists(name = "temp.txt", command = "adb shell ls /data")
	if file_exists:
		raise Exception("The test result did not achieve the desired results")

	os.system("sudo rm -rf ./temp")

except:
	relay_steps.reboot_main_os(serial=serial, relay_type=relay_type, relay_port=relay_port, power_port=power_port,
					wait_ui=False, timeout=300, delay_power_on=30, device_info="broxtonp", force_reboot=True)()
	fastboot_steps.factory_data_reset(serial = serial)()
	os.system("sudo rm -rf ./temp")
	raise
##### test end #####