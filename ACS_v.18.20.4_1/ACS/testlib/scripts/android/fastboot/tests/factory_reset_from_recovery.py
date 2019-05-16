#!/usr/bin/env python

######################################################################
#
# @filename:    factory_reset_from_recovery.py
# @description: Test factory reset from recovery OS.
#
# @run example:
#
#            python factory_reset_from_recovery.py --serial=A6EC8F70 --script-args
#                                               relay_type=RLY08B
#                                               relay_port=/dev/serial/by-id/usb-Devantech_Ltd._USB-RLY08_00015234-if00
#                                               power_port=4
#                                               v_down_port=8
#                                               v_up_port=7
# @author:      aurel.constantin@intel.com
#
#######################################################################

##### imports #####
import sys
import time
from testlib.base.base_utils import get_args
from testlib.scripts.relay import relay_steps
from testlib.scripts.connections.local import local_steps
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.android.adb import adb_steps

##### initialization #####
globals().update(vars(get_args(sys.argv)))

args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val

relay_type = args["relay_type"]
relay_port = args["relay_port"]
power_port = args["power_port"]
v_down_port = args["v_down_port"]
v_up_port = args["v_up_port"]

##### test start #####
relay_steps.reboot_main_os(serial=serial,
                           relay_type = relay_type,
                           relay_port = relay_port,
                           power_port = power_port)()

ui_steps.wake_up_device(serial = serial)()
ui_steps.unlock_device(serial = serial)()

# reboot to recovery mode
adb_steps.reboot_recovery(serial = serial)()

# factory reset
relay_steps.recovery_factory_reset(serial = serial,
                                   relay_type = relay_type,
                                   relay_port = relay_port,
                                   power_port = power_port,
                                   v_down_port = v_down_port)()
time.sleep(20)
relay_steps.recovery_reboot(serial = serial,
                            mode = "android",
                            relay_type = relay_type,
                            relay_port = relay_port,
                            power_port = power_port,
                            v_up_port = v_up_port,
                            v_down_port = v_down_port)()

# wait for adb to become online
local_steps.wait_for_adb(serial = serial,
                         timeout = 120)()

# wait for UI
adb_steps.wait_for_ui(serial = serial,
                    timeout = 1200)()

# setup stay awake
adb_steps.command(serial = serial,
                command = "svc power stayon true")()

ui_steps.press_home(serial = serial)()

#### test end #####
