#!/usr/bin/env python

# #############################################################################
#
# @filename:    gvb_red.py
#
# @description: Checks for the right color of the bootloader after it is
#               unlocked. Google Verified Boot. Results: Red
#
# @author:      alexandra.munteanu@intel.com
#
##############################################################################


from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.adb import adb_utils
from testlib.scripts.android.fastboot import fastboot_steps
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.security.scripts import prerequisites
from testlib.scripts.connections.local import local_steps
from testlib.base.base_utils import get_args
import sys
import time
import subprocess
import getpass


globals().update(vars(get_args(sys.argv)))

args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val

# mandatory params
android_build_top = args["android_build_top"]
boot_img = args["boot_img"]

dut_dessert = adb_utils.get_android_version(serial = serial)
host_username = getpass.getuser()

# Run Prerequisites
prerequisites.run_prereq(serial = serial,
                         pin = "1234")()

ui_steps.enable_oem_unlock(serial = serial, enabled = False, blocking = True)()

#run the script that generates the boot.img files
local_steps.init_android_tree(serial = serial,
                              android_build_top = android_build_top,
                              boot_img = boot_img,
                              username = host_username)()

fastboot_steps.change_color_state(serial = serial,
                                  state = "red",
                                  boot_img = "/tmp/{0}/img/user_signed_corrupted_boot.img"
                                  .format(host_username),
                                  dessert=dut_dessert)()

fastboot_steps.change_color_state(serial = serial,
                                  state = "green",
                                  boot_img = boot_img,
                                  dessert=dut_dessert)()
