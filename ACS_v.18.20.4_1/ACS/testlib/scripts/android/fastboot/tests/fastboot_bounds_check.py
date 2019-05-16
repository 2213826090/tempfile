#!/usr/bin/env python

######################################################################
#
# @filename:    fastboot_bounds_check.py
# @description: Boots into fastboot via adb and attempts to flash a
#               given partition. The process should fail (the test file
#               size should be greater than the partition size)
#
#
# @run example:
#
#            python fastboot_bounds_check.py -s 0BA8F2A0 --script-args
#                                               relay_type=RLY08B
#                                               relay_port=/dev/ttyACM0
#                                               power_port=4
#                                               v_up_port=1
#                                               v_down_port=2
#                                               partition = misc
#                                               size = 10000
#
#
# @author:      gabriel.porumb@intel.com
#
#######################################################################

# Imports #
import sys
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.adb import adb_utils
from testlib.scripts.android.fastboot import fastboot_steps
from testlib.base.base_utils import get_args
from testlib.scripts.relay import relay_steps
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.wireless.wifi import wifi_utils
from testlib.scripts.connections.local import local_steps
import os

# Initialisation #
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
size = int(args["size"])
test_file = ""

# Test start #
try:
    relay_steps.reboot_main_os(serial=serial,
                               relay_type=relay_type,
                               relay_port=relay_port,
                               power_port=power_port,
                               v_down_port=v_down_port,
                               v_up_port=v_up_port)()
    adb_steps.wait_for_ui(serial=serial)()

    ui_steps.wake_up_device(serial=serial)()
    ui_steps.unlock_device(serial=serial)()

    # Enable OEM unlock in developer options #
    ui_steps.enable_oem_unlock(serial=serial, enabled=False, blocking=False)()

    dessert = adb_utils.get_android_version(serial=serial)
    adb_steps.reboot(command="fastboot", serial=serial)()

    # Unlock the bootloader #
    relay_steps.change_state(serial=serial,
                             dessert=dessert,
                             unlock_bootloader="yes",
                             relay_type=relay_type,
                             relay_port=relay_port,
                             power_port=power_port,
                             v_down_port=v_down_port,
                             v_up_port=v_up_port)()

    # Create the test file to flash the misc partition with
    wifi_utils.create_file(file_name="test_dummy.tmp", file_size=size, file_path=".")
    test_file = os.path.join(".", "test_dummy.tmp")

    # Flash the partition
    fastboot_steps.check_partition_size_bounds(serial=serial,
                                               test_file=test_file)()

    # Lock the bootloader #
    relay_steps.change_state(serial=serial,
                             dessert=dessert,
                             unlock_bootloader="no",
                             relay_type=relay_type,
                             relay_port=relay_port,
                             power_port=power_port,
                             v_down_port=v_down_port,
                             v_up_port=v_up_port)()
except:
    raise
finally:
    local_steps.delete_file(test_file)
    relay_steps.reboot_main_os(serial=serial,
                               relay_type=relay_type,
                               relay_port=relay_port,
                               power_port=power_port,
                               v_down_port=v_down_port,
                               v_up_port=v_up_port)()
    adb_steps.wait_for_ui(serial=serial)()
# Test end #
