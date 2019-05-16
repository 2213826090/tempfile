#!/usr/bin/env python

########################################################################
#
# @filename:    audio_step.py
# @description: audio test step
# @author:      saddam.hussain.abbas@intel.com
#
########################################################################

# Standard library
from uiautomator import Device

# Userdefined library
from testlib.scripts.wireless.wifi.wifi_step import step as wifi_step
from testlib.scripts.android.adb.adb_step import step as adb_step

class AudioStep(adb_step):
    '''helper class for all audio test steps
       Refere wifi step and wifi_steps for more details
    '''
    def __init__(self, **kwargs):
        adb_step.__init__(self, **kwargs)

        # replacing old uidevice available in testlib/external with standard
        #  uiautomator device class
        self.uidevice = Device(serial=self.serial)

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

    def get_ui_dump(self, file_name):
        # save UI dump
        try:
            self.uidevice.dump(out_file = file_name,
                          compressed = False,
                          serial = self.serial)
        except:
            pass