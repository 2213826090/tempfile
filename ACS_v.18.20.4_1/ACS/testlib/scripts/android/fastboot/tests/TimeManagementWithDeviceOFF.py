#!/usr/bin/env python

######################################################################
#
# @filename:    TimeManagementWithDeviceOFF.py
# @description: Tests that the DUT time updates when the device is powered off and
#               the auto update time date option is disabled.
#
# @run example:
#
#            python TimeManagementWithDeviceOFF.py --serial=A6EC8F70 --script-args
#                                               relay_type=RLY08B
#                                               relay_port=/dev/ttyACM0
#                                               power_port=4
#                                               v_up_port=1
#                                               v_down_port=2
#
# @author:      gabriel.porumb@intel.com
#
#######################################################################

##### imports #####
import sys
from cgitb import enable

from testlib.base.base_utils import get_args
from testlib.scripts.relay import relay_steps
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.android.ui.ui_step import step as ui_step
from testlib.scripts.android.adb.adb_step import step as adb_step
from testlib.scripts.android.adb import adb_steps
import datetime
import time

##### initialization #####
globals().update(vars(get_args(sys.argv)))

args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val

relay_type = args["relay_type"]
relay_port = args["relay_port"]
power_port = args["power_port"]
v_up_port = args["v_up_port"]
v_down_port = args["v_down_port"]
adb_shell_date_format = "%a %b  %d %H:%M:%S %Z %Y"
dut_time_off = 120


class update_time_device_off(ui_step, adb_step):
    def __init__(self, serial, **kwargs):
        self.serial = serial
        ui_step.__init__(self, serial=serial, **kwargs)
        adb_step.__init__(self, serial=serial, **kwargs)
        self.set_errorm("", "Time not updated while device was off")
        self.set_passm("Time successfully updated while device was off")
        self.first_date = None
        self.second_date = None

    def do(self):
        # Enable auto time date
        ui_steps.enable_disable_auto_time_date(serial=self.serial,
                                               enable=True)()

        # Get the value of the DUT date via adb shell
        output = self.adb_connection.parse_cmd_output(cmd="date").strip()
        self.first_date = datetime.datetime.strptime(output, adb_shell_date_format)

        # Close Settings from recent apps
        ui_steps.close_app_from_recent(serial=self.serial,
                                       view_to_find={"text": "Settings"})()

        # Disable auto time date
        ui_steps.enable_disable_auto_time_date(serial=self.serial,
                                               enable=False)()

        # Power off device
        relay_steps.power_off_device(serial=self.serial,
                                     except_charging=True,
                                     relay_type=relay_type,
                                     relay_port=relay_port,
                                     power_port=power_port)()

        # keep the device off for 2 minutes
        for i in range(0, dut_time_off, 10):
            print "DUT SLEEPING FOR {} SECONDS".format(dut_time_off-i)
            time.sleep(10)

        # Power on device
        relay_steps.power_on_device(serial=self.serial,
                                    relay_type=relay_type,
                                    relay_port=relay_port,
                                    power_port=power_port)()
        adb_steps.wait_for_ui(serial=self.serial)()

        # Get the value of the DUT date via adb shell
        output = self.adb_connection.parse_cmd_output(cmd="date").strip()
        self.second_date = datetime.datetime.strptime(output, adb_shell_date_format)

        # Enable auto time and date
        ui_steps.wake_up_device(serial=self.serial)()
        ui_steps.unlock_device(serial=self.serial)()
        ui_steps.enable_disable_auto_time_date(serial=self.serial,
                                               enable=True)()

        # Close Settings from recent apps
        ui_steps.close_app_from_recent(serial=self.serial,
                                       view_to_find={"text": "Settings"})()

    def check_condition(self):
        delta = self.second_date - self.first_date
        return delta.seconds > dut_time_off

# Test start #
try:
    relay_steps.reboot_main_os(serial=serial,
                               relay_type=relay_type,
                               relay_port=relay_port,
                               power_port=power_port)()

    ui_steps.wake_up_device(serial=serial)()
    ui_steps.unlock_device(serial=serial)()

    update_time_device_off(serial=serial)()

finally:
    relay_steps.reboot_main_os(serial=serial,
                               relay_type=relay_type,
                               relay_port=relay_port,
                               power_port=power_port)()
# Test end #
