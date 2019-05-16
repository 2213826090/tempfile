#!/usr/bin/env python

# #############################################################################
#
# @filename:    prerequisites.py
#
# @description: Prerequisites for Security TCs
#
# @author:      costin.carabas@intel.com
#
##############################################################################


from testlib.scripts.android.android_step import step as android_step
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.connections.local import local_utils
from testlib.scripts.connections.local import local_steps
from testlib.scripts.android.fastboot import fastboot_steps
from testlib.scripts.wireless.wifi import wifi_steps
from testlib.base.base_utils import get_args
import sys
globals().update(vars(get_args(sys.argv)))

class run_prereq(android_step):

    """ description:
            Runs the prerequisites for security TCs

        usage:
            prerequisites.run_prereq(serial = serial,
                                     pin = "1234",
                                     set_screen_lock = True,
                                     require_pin_to_start_device = True,
                                     set_wifi = True,
                                     ap_name = "sp_gpt",
                                     ap_password = "Starpeakqwe123!")()

        tags:
            adb, android, security, prerequisites
    """

    def __init__(self, pin,
                       set_screen_lock = False,
                       set_wifi = False,
                       require_pin_to_start_device = False,
                       ap_name = None,
                       ap_password = None,
                       blocking = True,
                       **kwargs):
        android_step.__init__(self, **kwargs)
        self.set_screen_lock = set_screen_lock
        self.require_pin_to_start_device = require_pin_to_start_device
        self.pin = pin
        self.set_wifi = set_wifi
        self.ap_name = ap_name
        self.ap_password = ap_password

    def do(self):
        if local_utils.has_fastboot_serial(serial = self.serial):
            fastboot_steps.continue_to_adb(serial = self.serial,
                                           blocking = True)()

        local_steps.wait_for_adb(serial = self.serial,
                                 blocking = True)()

        adb_steps.wait_for_ui(serial = self.serial,
                              pin = self.pin,
                              blocking = True)()

        adb_steps.command(serial = self.serial,
                          command = "svc power stayon true")()

        ui_steps.remove_pin_screen_lock(serial = self.serial,
                                        pin = self.pin,
                                        blocking = True)()

        if self.set_screen_lock:
            ui_steps.set_pin_screen_lock(
                        serial = self.serial,
                        dut_pin = self.pin,
                        require_pin_to_start_device = self.require_pin_to_start_device,
                        blocking = True)()

        if self.set_wifi:
            wifi_steps.remove_network(serial = self.serial,
                                      ap_name = self.ap_name)()
            wifi_steps.connect_with_password(serial = self.serial,
                                             ap_name = self.ap_name,
                                             password = self.ap_password,
                                             blocking = True)()
        ui_steps.press_home(serial=self.serial)()

    def check_condition(self):
        # Check feature
        return True
