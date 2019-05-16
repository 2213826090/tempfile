#!/usr/bin/env python

# #############################################################################
#
# @filename:    smart_lock_trusted_location.py
#
# @description: Once this feature is enabled, it does no longer require a PIN
#               to unlock the device. It requires access to WiFi.
#
# @author:      costin.carabas@intel.com
#
##############################################################################

from testlib.scripts.android.android_step import step as android_step
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.android.ui import ui_utils
from testlib.scripts.security.scripts import prerequisites
from testlib.base.base_utils import get_args
import sys
import time
globals().update(vars(get_args(sys.argv)))

args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val

# mandatory params
ap_name =  args["net_ap_ssid"]
ap_password = args["net_ap_password"]

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


if __name__ == "__main__":
    prerequisites.run_prereq(serial = serial,
                             pin = "1234",
                             set_screen_lock = True,
                             set_wifi = True,
                             ap_name = ap_name,
                             ap_password = ap_password)()
    ui_steps.add_trusted_location(serial = serial, pin = "1234", location_name = "Locatia mea")()
    check_smart_lock(serial = serial)()
    ui_steps.remove_trusted_location(serial = serial, pin = "1234", location_name = "Locatia mea")()

