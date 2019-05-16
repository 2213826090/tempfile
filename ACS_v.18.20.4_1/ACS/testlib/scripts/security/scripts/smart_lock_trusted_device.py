#!/usr/bin/env python

# #############################################################################
#
# @filename:    smart_lock_trusted_device.py
#
# @description: Once this feature is enabled, it does no longer require a PIN
#               to unlock the device. It requires access to WiFi and another
#               device to connect with.
#
# @author:      claudiu.i.lataretu@intel.com
#
##############################################################################

from testlib.scripts.android.android_step import step as android_step
from testlib.scripts.security.scripts import prerequisites
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.ui import ui_steps
from testlib.base.base_utils import get_args
from testlib.scripts.wireless.bluetooth import bluetooth_steps
from testlib.scripts.android.ui import ui_utils
import sys
import time

globals().update(vars(get_args(sys.argv)))

args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val

# mandatory params
serial2 = args["serial2"]
ddwrt_ap_name = args["net_ap_ssid"]
ddwrt_ap_pass = args["net_ap_password"]


class check_smart_lock(android_step):

    """ description:
            Check smart lock: checks if DUT is pin locked

        usage:
            check_lock_timer()()

        tags:
            adb, android, lock, PIN
    """

    def __init__(self, timeout = 5, **kwargs):
        android_step.__init__(self, **kwargs)
        self.timeout = timeout

    def do(self):
        adb_steps.put_device_into_sleep_mode(serial = self.serial)()
        time.sleep(self.timeout)
        adb_steps.wake_up_device(serial = self.serial)()
        ui_steps.unlock_device_swipe(serial = self.serial)()

    def check_condition(self):
        return not ui_utils.is_device_pin_locked(serial = self.serial)


if __name__ == '__main__':
    # Run Prerequisites
    adb_steps.root_connect_device(serial=serial)()
    adb_steps.root_connect_device(serial=serial2)()

    prerequisites.run_prereq(serial=serial,
                             pin="1234",
                             set_screen_lock=True,
                             require_pin_to_start_device=True,
                             set_wifi=True,
                             ap_name=ddwrt_ap_name,
                             ap_password=ddwrt_ap_pass)()
    prerequisites.run_prereq(serial=serial2,
                             pin="1234",
                             set_screen_lock=True,
                             require_pin_to_start_device=True,
                             set_wifi=True,
                             ap_name=ddwrt_ap_name,
                             ap_password=ddwrt_ap_pass)()

    #Enable Bluetooth tethering
    bluetooth_steps.BtSetTethering(serial=serial,
                                   state="ON",
                                   timeout=30000)()
    bluetooth_steps.BtSetTethering(serial=serial2,
                                   state="ON",
                                   timeout=30000)()

    bluetooth_steps.OpenBluetoothSettings(serial=serial,
                                          use_intent=True)()

    bluetooth_steps.BtChangeDeviceName(serial=serial,
                                       name=serial)()

    bluetooth_steps.OpenBluetoothSettings(serial=serial2,
                                          use_intent=True)()

    bluetooth_steps.BtChangeDeviceName(serial=serial2,
                                       name=serial2)()

    bluetooth_steps.WaitBtScanning(serial=serial)()

    bluetooth_steps.InitiatePairRequest(serial=serial,
                                         dev_to_pair_name=serial2,
                                         scan_max_attempts=5)()

    bluetooth_steps.PerformActionPairRequest(serial=serial)()
    bluetooth_steps.PerformActionPairRequest(serial=serial2)()

    bluetooth_steps.ConnectPairedDevices(serial=serial,
                                         dev_to_connect_name=serial2)()

    ui_steps.add_trusted_device(serial=serial,
                                pin="1234",
                                device_name=serial2)()
    check_smart_lock(serial=serial)()

    bluetooth_steps.OpenBluetoothSettings(serial=serial,
                                          use_intent=True)()
    bluetooth_steps.WaitBtScanning(serial=serial)()
    bluetooth_steps.UnpairDevice(serial=serial,
                                 device_name=serial2)()
    bluetooth_steps.ClickBluetoothSwitch(serial=serial,
                                         state="OFF")()

    bluetooth_steps.OpenBluetoothSettings(serial=serial2,
                                          use_intent=True)()
    bluetooth_steps.WaitBtScanning(serial=serial2)()
    bluetooth_steps.UnpairDevice(serial=serial2,
                                 device_name=serial)()
    bluetooth_steps.ClickBluetoothSwitch(serial=serial2,
                                         state="OFF")()
