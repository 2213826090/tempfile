#!/usr/bin/env python

#######################################################################
#
# @filename:    erase_from_usb.py
# @description: Checks if a file can be deleted from usb
# @author:      andreeax.a.vlad@intel.com
#
#######################################################################

from testlib.scripts.storage import storage_steps
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.adb import adb_utils
from testlib.scripts.android.ui import ui_steps
from testlib.base.base_utils import get_args

import sys

args = get_args(sys.argv)
globals().update(vars(args))

adb_steps.connect_device(
    serial = serial,
    port = adb_server_port
)()
globals().update({"version": adb_utils.get_android_version()})

pic_name = script_args[0]

ui_steps.press_home()()
storage_steps.erase_copied_file(value = pic_name)()
ui_steps.press_home()()

