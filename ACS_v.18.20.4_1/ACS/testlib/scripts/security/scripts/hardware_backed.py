#!/usr/bin/env python

# #############################################################################
#
# @filename:    hardware_backed.py
#
# @description: Checks if the the KeyStore is hardware backed
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

####################  ENCRYPTED #############################
class is_storage_hardware_backed(android_step):

    """ description:
            Checks if the the KeyStore is hardware backed

        usage:
            is_storage_hardware_backed()()

        tags:
            adb, android, storage, keystore, hardware backed
    """


    def do(self):
        prerequisites.run_prereq(serial = serial,
                                 pin = "1234")()

    def check_condition(self):
        # Check feature
        return security_utils.is_storage_hardware_backed(serial = self.serial)

if __name__ == "__main__":
    is_storage_hardware_backed(serial = serial)()
