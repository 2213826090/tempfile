#!/usr/bin/env python

# #############################################################################
#
# @filename:    selinux.py
#
# @description: Checks if SE Linux policies are enforced
#
# @author:      costin.carabas@intel.com
#
##############################################################################

from testlib.scripts.android.adb import adb_steps
from testlib.base.base_utils import get_args
import sys
globals().update(vars(get_args(sys.argv)))

# Check feature
adb_steps.command(serial = serial,
        command = "getenforce",
        stdout_grep = "Enforcing")()
