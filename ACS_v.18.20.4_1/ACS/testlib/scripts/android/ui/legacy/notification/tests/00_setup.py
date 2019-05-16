#!/usr/bin/env python

##############################################################################
#
# @filename:    .py
#
# @description: Install NotificationTrigger.apk which is needed by all
#               notification tests.
#
# @author:      silviux.l.andrei@intel.com
#
##############################################################################

from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.adb import adb_utils
import os

import sys

from testlib.base.base_utils import get_args

globals().update(vars(get_args(sys.argv)))

adb_steps.connect_device(
    serial = serial,
    port = adb_server_port
)()
globals().update({"version": adb_utils.get_android_version()})

apk_path = os.path.join(media_path, apks[0])

adb_steps.install_apk(print_error = "Error - App was not installed",
                      blocking = False,
                      apk_path = apk_path)()

