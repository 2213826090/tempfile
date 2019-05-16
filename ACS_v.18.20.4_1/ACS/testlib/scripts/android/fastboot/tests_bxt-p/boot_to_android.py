#!/usr/bin/env python

##### imports #####
import os
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
platform_name = fastboot_utils.get_platform_name(serial=serial)
if platform_name == "bxtp_abl": wait_ui = True
if platform_name == "gordon_peak": wait_ui = False

try:
	relay_steps.reboot_main_os(serial=serial, relay_type=relay_type, relay_port=relay_port, power_port=power_port,
					wait_ui=wait_ui, timeout=300, delay_power_on=30, device_info="broxtonp")()

	relay_steps.reboot_main_os(serial=serial, relay_type=relay_type, relay_port=relay_port, power_port=power_port,
					wait_ui=wait_ui, timeout=300, delay_power_on=30, device_info="broxtonp", force_reboot=True)()

except:
	raise

finally:
	relay_steps.reboot_main_os(serial=serial, relay_type=relay_type, relay_port=relay_port, power_port=power_port,
					wait_ui=wait_ui, timeout=300, delay_power_on=30, device_info="broxtonp")()
##### test end #####