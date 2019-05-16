#!/usr/bin/env python

######################################################################
#
# @filename:    prereq_root_devices.py
# @description: Set root on devices.
#
#
# @run example:
#
#            python prereq_root_devices.py -s 0BA8F2A0
#                                               --script-args
#                                                       serial2=XXXXXXXX
#
# @author:      aurel.constantin@intel.com
#
#######################################################################

##### imports #####
import sys
from testlib.scripts.android.adb import adb_steps
from testlib.base.base_utils import get_args

##### initialization #####
globals().update(vars(get_args(sys.argv)))
args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val

# mandatory params
serial2 = args["serial2"]

##### test start #####
# set root on both devices
adb_steps.root_connect_device(serial = serial)()
adb_steps.root_connect_device(serial = serial2)()

##### test end #####
