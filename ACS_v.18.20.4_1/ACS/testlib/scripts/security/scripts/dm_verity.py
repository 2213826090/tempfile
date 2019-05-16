#!/usr/bin/env python

# #############################################################################
#
# @filename:    dm_verity.py
#
# @description: Checks the device-mapper-verity (dm-verity) feature
#
# @author:      costin.carabas@intel.com
#
##############################################################################

from testlib.utils.statics.android import statics
from testlib.scripts.android.adb import adb_steps
from testlib.base.base_utils import get_args
from testlib.scripts.android.android_step import step as android_step
import sys
globals().update(vars(get_args(sys.argv)))

class dm_verity_prop_enabled(android_step):

    """ description:
            Checks if variables exists in fastboot mode

        usage:
            variable_exist(var = ["device-state",..])()

        tags:
            fastboot, vars
    """

    def do(self):
        self.step_data = adb_steps.command(serial = serial,
                                           command = "getprop"
                                           " partition.system.verified")()
    def check_condition(self):
        stdout = self.step_data.stdout.read().strip()
        if not stdout:
            return False
        return True

if __name__ == "__main__":
    adb_steps.command(serial = serial,
                      command = "cat /fstab* | grep ^/ | grep /system",
                      stdout_grep = "verify")()

    if statics.Device(serial = serial).build_type == "user":
        dm_verity_prop_enabled(serial = serial)()
