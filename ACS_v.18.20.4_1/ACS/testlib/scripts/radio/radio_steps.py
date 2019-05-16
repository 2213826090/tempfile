#!/usr/bin/env python

#######################################################################
#
# @filename:    rds_steps.py
# @description: Radio FM test steps
# @author:      dragosx.nicolaescu@intel.com
#
#######################################################################


from testlib.scripts.android.ui import ui_steps
from testlib.scripts.android.ui.ui_step import step as ui_step
from testlib.scripts.android.ui import ui_utils
from testlib.utils.ui.uiandroid import UIDevice as ui_device
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.adb import adb_utils
from testlib.base.base_step import step as base_step
from testlib.utils.connections.adb import Adb as connection_adb
from testlib.utils.connections.local import Local as connection_local
from testlib.utils.statics.android import statics
from testlib.scripts.android.logcat import logcat_steps
import time
import os
import re


class start_fmradio(ui_step):
    """ description:
        Start the FM Radio app on desired frequency with needed parameters
        Possible choices:
            Enable/Disable RDS
            Enable/Disable TA
            Enable/Disable AF

        usage:
            radio_steps.start_fmradio(serial, frequency, rds = "ON", ta = "ON", af = "ON")()

        tags:
            ui, android, radio, rds, ta, af
    """

    def __init__(self, frequency, rds = "ON", ta = "ON", af = "ON", wait_time = 10, **kwargs):
        ui_step.__init__(self, **kwargs)
        self.frequency = float(frequency)/100.0
        self.rds = rds
        self.ta = ta
        self.af = af
        self.wait_time = wait_time

    def do(self):
        # Clear App data
        adb_steps.command(serial = self.serial, command = "adb shell pm clear com.intel.fmapp")()
        # start the FM Radio app
        adb_steps.am_start_command(serial = self.serial,
                                    component = "com.intel.fmapp/.MainActivity")()
        ui_steps.click_button_if_exists(serial = self.serial,
                                    view_to_find = {"resourceId":"android:id/button2"})()
        # go to Settings
        ui_steps.click_button(serial = self. serial,
                                    view_to_find = {"className": "android.widget.ImageButton"})()
        ui_steps.click_button(serial = self.serial,
                                    view_to_find = {"className": "android.widget.TextView"},
                                    view_to_check = {"text": "Settings"})()
        # check or configure the RDS, TA, AF state
        if self.rds == "OFF" :
            ui_steps.click_checkbox_button(serial = self.serial,
                                    view_to_find = {"text":"Enable RDS"}, state = "OFF", relationship="right")()
        else:
            ui_steps.click_checkbox_button(serial = self.serial,
                        view_to_find = {"text":"Enable RDS"}, state = "ON", relationship="right")()
        if self.ta == "OFF" :
            ui_steps.click_checkbox_button(serial = self.serial,
                                    view_to_find = {"text":"Enable TA"}, state = "OFF", relationship="right")()
        else:
            ui_steps.click_checkbox_button(serial = self.serial,
                        view_to_find = {"text":"Enable TA"}, state = "ON", relationship="right")()
        if self.af == "OFF" :
            ui_steps.click_checkbox_button(serial = self.serial,
                                    view_to_find = {"text":"Enable AF"}, state = "OFF", relationship="right")()
        else:
            ui_steps.click_checkbox_button(serial = self.serial,
                                    view_to_find = {"text":"Enable AF"}, state = "ON", relationship="right")()
        ui_steps.press_back(serial = self. serial,
                                    view_to_check = {"text": "FM Radio"})()
        ui_steps.click_button(serial = self.serial,
                                    view_to_find = {"resourceId":"com.intel.fmapp:id/button_power"})()
        ui_steps.click_button_if_exists(serial = self.serial,
                                    view_to_find = {"resourceId":"android:id/button3"})()
        ui_steps.click_button(serial = self.serial,
                                    view_to_find = {"resourceId":"com.intel.fmapp:id/button_direct"})()
        ui_steps.edit_text(serial = self.serial,
                                    view_to_find = {"resourceId": "com.intel.fmapp:id/edit_frequency"},
                                    value = self.frequency)()
        ui_steps.click_button_if_exists(serial = self.serial,
                                    view_to_find = {"resourceId":"android:id/button1"})()
        time.sleep(self.wait_time)

    def check_condition(self):
        # Check performed in do()
        return True


class toggle_rds(ui_step):
    """ description:
        Turns RDS off then on for x iterations from the Radio FM App Settings menu.

        usage:
            radio_steps.toggle_rds(serial, iterations)()

        tags:
            ui, android, radio, rds
    """
    def __init__(self, iterations = 1, wait_time = 10, **kwargs):
        ui_step.__init__(self, **kwargs)
        self.iterations = iterations
        self.wait_time = wait_time

    def do(self):
        for i in range(self.iterations):
            # go to Settings
            ui_steps.click_button(serial = self. serial,
                                    view_to_find = {"className": "android.widget.ImageButton"})()
            ui_steps.click_button(serial = self.serial,
                                    view_to_find = {"className": "android.widget.TextView"},
                                    view_to_check = {"text": "Settings"})()
            ui_steps.click_checkbox_button(serial = self.serial,
                                    view_to_find = {"text":"Enable RDS"}, state = "OFF", relationship="right")()
            logcat_steps.grep_for(serial = self.serial, grep_for_text = "radio_hw: disable RDS decoding", text_presence = True)()
            logcat_steps.clear_logcat(serial = self.serial)()
            ui_steps.click_checkbox_button(serial = self.serial,
                                    view_to_find = {"text":"Enable RDS"}, state = "ON", relationship="right")()
            logcat_steps.grep_for(serial = self.serial, grep_for_text = "radio_hw: enable RDS decoding", text_presence = True)()
            ui_steps.press_back(serial = self. serial,
                                    view_to_check = {"text": "FM Radio"})()
            time.sleep(self.wait_time)

    def check_condition(self):
        # Check performed in do()
        return True
