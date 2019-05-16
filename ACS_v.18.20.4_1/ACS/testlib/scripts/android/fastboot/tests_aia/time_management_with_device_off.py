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
def get_system_time(cmd=None):
	system_time = None
	return_result = os.popen(cmd).readlines()
	for line in return_result:
		date = line.split()[3]
		system_time = date.split(":")
	return system_time

def calculate_system_time(system_time=None):
	hours = int(system_time[0])
	minutes = int(system_time[1])
	seconds = int(system_time[2])
	return hours * 3600 + minutes * 60 + seconds


fastboot_utils.push_uiautomator_jar(serial=serial)
fastboot_steps.time_management_with_device_off(serial=serial)()

system_time_before_restart = None
system_time_after_restart = None
get_date_cmd = "adb -s {} shell date".format(serial)

system_time_before_restart = get_system_time(cmd=get_date_cmd)
print "system_time_before_restart: " + str(system_time_before_restart)
relay_steps.reboot_main_os(serial=serial, relay_type=relay_type, relay_port=relay_port, power_port=power_port,
				wait_ui=False, timeout=300, delay_power_on=120, device_info="broxtonp", force_reboot=True)()
system_time_after_restart = get_system_time(cmd=get_date_cmd)
print "system_time_after_restart: " + str(system_time_after_restart)

time_before_restart = calculate_system_time(system_time=system_time_before_restart)
print "time_before_restart: " + str(time_before_restart)
time_after_restart = calculate_system_time(system_time=system_time_after_restart)
print "time_after_restart: " + str(time_after_restart)

if (time_after_restart - time_before_restart) < 120 or 240 < (time_after_restart - time_before_restart):
	raise Exception("The test result did not achieve the desired results")
##### test end #####