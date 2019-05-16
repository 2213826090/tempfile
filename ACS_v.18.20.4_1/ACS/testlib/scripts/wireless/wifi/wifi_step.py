#!/usr/bin/env python

########################################################################
#
# @filename:    wifi_step.py
# @description: Wifi test step
# @author:      alexandru.i.nemes@intel.com
#
########################################################################

from testlib.scripts.android.ui.ui_step import step as ui_step
from testlib.scripts.android.adb.adb_step import step as adb_step


class step(adb_step, ui_step):
    '''helper class for all wifi test steps'''

    def __init__(self, **kwargs):
        ui_step.__init__(self, **kwargs)
        adb_step.__init__(self, **kwargs)

    def get_driver_logs(self, file_name):
        try:
            self.adb_connection.run_cmd("echo 'see dmesg' > /sdcard/locallogfile")
            self.adb_connection.get_file("/sdcard/locallogfile", file_name)
        except:
            pass

    def get_failed_dmsg(self, file_name):
        try:
            self.adb_connection.run_cmd("dmesg > /sdcard/localdmsgfile")
            self.adb_connection.get_file("/sdcard/localdmsgfile", file_name)
        except:
            pass
