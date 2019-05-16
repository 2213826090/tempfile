#!/usr/bin/env python
#######################################################################
#
# @filename:    app_and_back.py
# @description: Hackaton script 
# @author:      ion-horia.petrisor@intel.com
#
# python uninstall_app.py --serial 10.237.104.142:5555 
#                         --media-path /home/oane/Work/automation/\
#                                      testlib/resources/apks/
#                         --apks com.shazam.android_408300.apk 
#                         --script-args uninstall-time=10000
#
#######################################################################

from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.file import file_steps
from testlib.base.base_utils import get_args
import sys
import time

globals().update(vars(get_args(sys.argv)))

adb_steps.connect_device(
    serial = serial,
    port = adb_server_port
)()

args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val

if "uninstall-time" in args:
    uninstall_time = int(args["uninstall-time"])
else:
    uninstall_time = None

adb_steps.uninstall_apk(apk_path = media_path + apks[0],
                        uninstall_time = uninstall_time)()
