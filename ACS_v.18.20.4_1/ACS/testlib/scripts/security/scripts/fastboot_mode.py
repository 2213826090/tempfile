#!/usr/bin/env python

# #############################################################################
#
# @filename:    fastboot_mode.py
#
# @description: Checks if DUT enters fastboot mode and has variables set.
#
# @author:      costin.carabas@intel.com
#
##############################################################################


from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.fastboot import fastboot_steps
from testlib.scripts.android.fastboot import fastboot_utils
from testlib.scripts.connections.local import local_utils
from testlib.scripts.android.android_step import step as android_step
from testlib.scripts.security.scripts import prerequisites
from testlib.base.base_utils import get_args
import sys
import time
globals().update(vars(get_args(sys.argv)))

class variables_exist(android_step):

    """ description:
            Checks if variables exists in fastboot mode

        usage:
            variable_exist(var = ["device-state",..])()

        tags:
            fastboot, vars
    """

    def __init__(self, vars, **kwargs):
        self.vars = vars
        android_step.__init__(self, **kwargs)

    def do(self):
        if local_utils.has_adb_serial(serial = self.serial):
            adb_steps.reboot(serial = self.serial,
                             command = "fastboot",
                             ip_enabled = False)()

    def check_condition(self):
        for var in self.vars:
            if not fastboot_utils.var_exists(serial = self.serial,
                                             var = var):

                self.set_errorm("", "Could not find variable {0}".format(var))
                return False
        return True

if __name__ == "__main__":
    # Run Prerequisites
    prerequisites.run_prereq(serial = serial,
                             pin = "1234")()

    time.sleep(5)
    # check for fastboot variables
    variables_exist(serial = serial,
                    vars = ["product",
                            "version-bootloader",
                            "secure",
                            "unlocked"])()

    # return to Android OS
    fastboot_steps.continue_to_adb(serial = serial)()
    adb_steps.wait_for_ui(serial = serial)()
