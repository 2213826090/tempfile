#!/usr/bin/env python

########################################################################
#
# @filename:    adb_step.py
# @description: adb test step
# @author:      ion-horia.petrisor@intel.com
#
########################################################################

from testlib.scripts.android.android_step import step as android_step
from testlib.utils.connections.adb import Adb as connection_adb
from testlib.base import base_utils


class step(android_step):
    '''helper class for all adb test steps'''
    adb_connection = None

    def __init__(self, **kwargs):
        android_step.__init__(self, **kwargs)
        self.adb_connection = connection_adb(**kwargs)

    def take_picture(self, file_name):
        # catch all exceptions and ignore them
        try:
            self.adb_connection.run_cmd("screencap -p /sdcard/screen.png")
            self.adb_connection.get_file("/sdcard/screen.png", file_name)
            self.adb_connection.run_cmd("rm /sdcard/screen.png")
        except Exception, e:
            if "device not found" in e.message:
                raise base_utils.DeviceNotFoundError("Serial {0} not found".format(self.serial))
