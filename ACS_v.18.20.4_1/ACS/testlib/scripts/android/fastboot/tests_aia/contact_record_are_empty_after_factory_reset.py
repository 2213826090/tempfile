#!/usr/bin/env python

##### imports #####
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
	fastboot_steps.create_some_contacts(serial = serial)()
	fastboot_steps.factory_data_reset(serial = serial)()

	fastboot_utils.push_uiautomator_jar(serial = serial)
	fastboot_steps.config_first_boot_wizard(serial = serial)()
	fastboot_steps.contact_is_empty(serial = serial)()
	fastboot_steps.factory_data_reset(serial = serial)()

except:
	relay_steps.reboot_main_os(serial=serial, relay_type=relay_type, relay_port=relay_port, power_port=power_port,
					wait_ui=False, timeout=300, delay_power_on=30, device_info="broxtonp", force_reboot=True)()
	fastboot_steps.factory_data_reset(serial = serial)()
	raise
##### test end #####