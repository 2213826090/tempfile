#!/usr/bin/env python

# #############################################################################
#
# @filename:    keymaster.py
#
# @description: Provide a basic set of cryptrographic primitives to allow the
#		implementation of protocols using access-controlled,
#		hardware-backed keys
#
# @author:      claudiu.i.lataretu@intel.com
#
##############################################################################

from testlib.scripts.android.adb import adb_steps
from testlib.base.base_utils import get_args
import sys

globals().update(vars(get_args(sys.argv)))

adb_steps.reboot(serial=serial,
                 disable_uiautomator=True,
                 boot_to_Android=False,
                 no_ui=True)()
adb_steps.wait_for_ui_processes(serial=serial)()
adb_steps.command(serial=serial,
                  command="logcat -d | grep -i keymaster",
                  stdout_grep="Found keymaster1 module,"
                  " using keymaster1 API")()
