#!/usr/bin/env python

# #############################################################################
#
# @filename:    system_mount_ro.py
#
# @description: Checks if "/system" file system has permissions of read-only
#
# @author:      costin.carabas@intel.com
#
##############################################################################

from testlib.scripts.android.adb import adb_steps
from testlib.base.base_utils import get_args
import sys
globals().update(vars(get_args(sys.argv)))

adb_steps.command(serial = serial,
        command = "mount | grep system | grep rw",
        stdout_not_grep = "/system")()
