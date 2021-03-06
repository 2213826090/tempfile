#!/usr/bin/env python

#######################################################################
#
# @filename:    write_to_usb.py
# @description: Write a file to usb
# @author:      andreeax.a.vlad@intel.com
#
#######################################################################

from testlib.base.base_utils import get_args
from testlib.scripts.storage import storage_steps
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.adb import adb_utils
from testlib.scripts.android.ui import ui_steps

import sys

args = get_args(sys.argv)
globals().update(vars(args))

adb_steps.connect_device(
    serial = serial,
    port = adb_server_port
)()
globals().update({"version": adb_utils.get_android_version()})

file_name = script_args[0]

ui_steps.press_home()()
storage_steps.write_to_usb(value = file_name)()
ui_steps.press_home()()

