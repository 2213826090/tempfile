#!/usr/bin/env python

#######################################################################
#
# @filename:    test_01_internal_storage_read.py
# @description: Checks read rights of the internal storage
# @author:      ion-horia.petrisor@intel.com
#
#######################################################################

import sys

from testlib.scripts.storage import storage_steps
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.adb import adb_utils
from testlib.base.base_utils import get_args

globals().update(vars(get_args(sys.argv)))
adb_steps.connect_device(serial = serial, port = adb_server_port)()
globals().update({"version": adb_utils.get_android_version()})

adb_steps.root_connect_device()()
storage_steps.entry_in_path(path = "/sdcard/", inode = "Music")()

