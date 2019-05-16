#!/usr/bin/env python

#######################################################################
#
# @filename:    test_01_dalvik_suite.py
# @description: Runs Dalvik suite
# @author:      ion-horia.petrisor@intel.com
#
#######################################################################

import sys

from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.adb import adb_utils
from testlib.base.base_utils import get_args

globals().update(vars(get_args(sys.argv)))
adb_steps.connect_device(serial = serial, port = adb_server_port)()
globals().update({"version": adb_utils.get_android_version()})

adb_steps.root_connect_device()()
adb_steps.create_folder(path = "/sdcard/Music", folder = "test_folder")()

adb_steps.delete_folder(folder = "/sdcard/Music/test_folder", blocking = True)()

