#!/usr/bin/env python

##### imports #####
import sys
from testlib.base.base_utils import get_args
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

	relay_steps.reboot_main_os(serial=serial, relay_type=relay_type, relay_port=relay_port, power_port=power_port,
					wait_ui=False, timeout=300, delay_power_on=30, device_info="broxtonp", force_reboot=True)()

except:
	raise

finally:
	relay_steps.reboot_main_os(serial=serial, relay_type=relay_type, relay_port=relay_port, power_port=power_port,
					wait_ui=False, timeout=300, delay_power_on=30, device_info="broxtonp")()
##### test end #####