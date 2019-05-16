#!/usr/bin/env python

##### imports #####
import os
import sys
import time
from testlib.base.base_utils import get_args
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.fastboot import fastboot_steps
from testlib.scripts.android.fastboot import fastboot_utils
from testlib.scripts.connections.local import local_steps

##### initialization #####
globals().update(vars(get_args(sys.argv)))

##### test start #####
try:
	os.system("mkdir -p ./temp/files/flash")
	fastboot_utils.download_flash_scripts()

	adb_steps.reboot(command="fastboot", reboot_timeout=300, serial=serial)()
	local_steps.wait_for_fastboot(timeout=300, serial=serial)()
	fastboot_steps.continue_to_adb(serial=serial)()
	local_steps.wait_for_adb(timeout=300, serial=serial)()

	os.system("adb -s {0} shell reboot -p".format(serial))
	time.sleep(30)
	fastboot_utils.to_fastboot_by_script(serial=serial)
	local_steps.wait_for_fastboot(timeout=300, serial=serial)()
	fastboot_steps.continue_to_adb(serial=serial)()
	local_steps.wait_for_adb(timeout=300, serial=serial)()
	os.system("sudo rm -rf ./temp")

except:
	fastboot_utils.to_fastboot_by_script(serial=serial)
	local_steps.wait_for_fastboot(timeout=300, serial=serial)()
	fastboot_steps.continue_to_adb(serial=serial)()
	local_steps.wait_for_adb(timeout=300, serial=serial)()
	os.system("sudo rm -rf ./temp")
##### test end #####