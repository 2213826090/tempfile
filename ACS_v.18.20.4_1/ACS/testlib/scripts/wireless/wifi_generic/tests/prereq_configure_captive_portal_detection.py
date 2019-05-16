#!/usr/bin/env python

######################################################################
#
# @filename:    prereq_disable_captive_portal_detection.py
# @description: Prerequisite to disable captive_portal_detection (the one
#               responsable withy "No Internet Access Detected, won't
#               No Internet Access Detected, won't" message)
#
#
# @run example:
#
#            python prereq_disable_captive_portal_detection.py -s 0BA8F2A0
#
# @author:      aurel.constantin@intel.com
#
#######################################################################

##### imports #####
import sys
import time
from testlib.scripts.wireless.wifi_generic import wifi_generic_steps
from testlib.scripts.ap import ap_steps
from testlib.scripts.android.adb import adb_steps
from testlib.base.base_utils import get_args
from testlib.utils.defaults import wifi_defaults
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.android.adb import adb_utils
import subprocess
import StringIO

##### initialization #####
globals().update(vars(get_args(sys.argv)))
args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val

# mandatory params
state = args["state"]
serial2 = args.get("serial2")
stdout, _ = subprocess.Popen("adb devices", stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True).communicate()
out = StringIO.StringIO(stdout)
device_type1 = adb_utils.get_device_orientation_type(serial = serial)
devices_name = []

for line in out.readlines():
    if "device" in line:
        devices_name.append(line.split()[0])
out.close()

##### set state #####
adb_steps.command(serial = serial,
            command = "settings put global captive_portal_detection_enabled {}".format(state),
            mode = "sync",
            timeout = 10)()

# rotate to portrait
ui_steps.set_orientation(serial = serial,
                     orientation = "portrait",
                     target = device_type1)()
if serial2 and serial2 in devices_name:
    device_type2 = adb_utils.get_device_orientation_type(serial=serial2)
    ui_steps.set_orientation(serial = serial2,
                     orientation = "portrait",
                     target = device_type2)()
##### test end #####
