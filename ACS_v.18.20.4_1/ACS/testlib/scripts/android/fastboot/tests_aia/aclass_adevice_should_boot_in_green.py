#!/usr/bin/env python

##### imports #####
import sys
import time
from testlib.base.base_utils import get_args
from testlib.scripts.android.adb import adb_steps
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
	relay_steps.reboot_main_os(serial=serial, relay_type=relay_type, relay_port=relay_port, power_port=power_port,
					wait_ui=False, timeout=300, delay_power_on=30, device_info="broxtonp")()

	adb_steps.root_connect_device(serial=serial)()
	time.sleep(5)

	if not fastboot_utils.is_verified_boot_state(serial=serial, state="green"):
		raise Exception("The test result did not achieve the desired results")

except:
	raise

finally:
	relay_steps.reboot_main_os(serial=serial, relay_type=relay_type, relay_port=relay_port, power_port=power_port,
					wait_ui=False, timeout=300, delay_power_on=30, device_info="broxtonp")()
##### test end #####