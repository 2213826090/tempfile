#!/usr/bin/env python

##### imports #####
import sys
import time
from testlib.base.base_utils import get_args
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

	fastboot_utils.push_uiautomator_jar(serial = serial)

	relay_steps.gracefully_power_off_device(serial=serial, relay_type=relay_type, relay_port=relay_port, power_port=power_port)()

	time.sleep(30)

	relay_steps.power_on_device(serial=serial, relay_type=relay_type, relay_port=relay_port, power_port=power_port)()

except:
	raise

finally:
	relay_steps.reboot_main_os(serial=serial, relay_type=relay_type, relay_port=relay_port, power_port=power_port,
					wait_ui=False, timeout=300, delay_power_on=30, device_info="broxtonp")()
##### test end #####