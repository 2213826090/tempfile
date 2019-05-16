#!/usr/bin/env python

#######################################################################
#
# @filename:    test_01_sp_update.py
# @description: Checks sp_update is available on DUT
# @author:      ion-horia.petrisor@intel.com
#
#######################################################################

from testlib.scripts.android.adb import adb_steps

import sys

adb_steps.connect_device(serial = sys.argv[1] + ":5555",
                         port = sys.argv[2])()

adb_steps.command(command = "ls /system/bin/sp_update",
                  stdout_grep = "/system/bin/sp_update",
                  stdout_not_grep = "No such file or directory")()
