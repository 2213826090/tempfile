#!/usr/bin/env python

##### imports #####
import os
import sys
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
	fastboot_utils.push_uiautomator_jar(serial = serial)
	fastboot_steps.config_first_boot_wizard(serial = serial)()
	fastboot_steps.connect_to_internet(serial = serial)()
	fastboot_steps.visit_web_page_by_browser(serial = serial)()
	fastboot_steps.factory_data_reset(serial = serial)()

	fastboot_utils.push_uiautomator_jar(serial = serial)
	fastboot_steps.config_first_boot_wizard(serial = serial)()
	fastboot_steps.browser_history_is_empty(serial = serial)()
	fastboot_steps.factory_data_reset(serial = serial)()

except:
	os.system("mkdir -p ./temp/files/flash")
	fastboot_utils.download_flash_scripts()
	fastboot_utils.flash_bxt(zip_file=flash_files, serial=serial)
	os.system("sudo rm -rf ./temp")
	raise
##### test end #####