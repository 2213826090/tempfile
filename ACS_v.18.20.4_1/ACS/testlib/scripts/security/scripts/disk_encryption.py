#!/usr/bin/env python

# #############################################################################
#
# @filename:    disk_encryption.py
#
# @description: Check if storage is encrypted
#
# @author:      costin.carabas@intel.com
#
##############################################################################

from testlib.scripts.android.android_step import step as android_step
from testlib.scripts.security import security_utils
from testlib.scripts.security.scripts import prerequisites
from testlib.base.base_utils import get_args
import sys
globals().update(vars(get_args(sys.argv)))

class is_dut_encrypted(android_step):

    """ description:
            Checks if the storage is encrypted

        usage:
            is_dut_encrypted()()

        tags:
            adb, android, storage, encrypted
    """

    def do(self):
        # Run Prerequisites
        prerequisites.run_prereq(serial = serial,
                                 pin = "1234")()

    def check_condition(self):
        # Check feature
        return security_utils.is_dut_encrypted(serial = self.serial)


if __name__ == "__main__":
    is_dut_encrypted(serial = serial)()
