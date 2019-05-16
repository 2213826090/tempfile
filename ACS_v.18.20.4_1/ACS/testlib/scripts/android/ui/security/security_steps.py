#!/usr/bin/env python
#coding:utf-8

import time
import random
import string
from testlib.scripts.android.ui.security import security_utils
from testlib.scripts.android.ui import ui_utils
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.android.ui.ui_step import step as ui_step
from testlib.scripts.android.adb import adb_utils
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.adb.adb_step import step as adb_step
from testlib.scripts.connections.local import local_steps


class open_settings(ui_step, adb_step):
    """
        description:
            opens the settings on the device
    """
    def __init__(self, settings = "Settings", settings_process = "settings",
           view_to_check = {"packageName": "com.android.settings"}, **kwargs):
      ui_step.__init__(self, **kwargs)
      adb_step.__init__(self, **kwargs)
      self.settings = settings
      self.settings_process = settings_process
      self.view_to_check = view_to_check

    def do(self):
      settings_pid_list = self.adb_connection.pgrep(self.settings_process)
      self.adb_connection.kill_all(settings_pid_list)
      ui_steps.open_app_from_allapps(serial = self.serial,
                       view_to_find = {"text": self.settings},
                                       wait_time = 10000,
                       view_to_check = self.view_to_check)()

    def check_condition(self):
      return self.adb_connection.get_pid(self.settings_process) != None


class open_security_menu(ui_step):
    """
        description:
            opens the security menu of settings
    """
    def __init__(self, wait_time = 5000, **kwargs):
        ui_step.__init__(self, **kwargs)
        self.wait_time = wait_time

    def do(self):
        ui_steps.click_button_with_scroll(serial = self.serial,
                                          view_to_find = {"resourceId":
                                                          "com.android.settings:id/title",
                                                          "text": "Security"},
                                          view_to_check = {"resourceId":
                                                           "android:id/title",
                                                           "text": "Device security"})()

    def check_condition(self):
        return self.uidevice(resourceId = "android:id/title", text = "Device security").wait.exists(timeout = self.wait_time)


class set_pin_screen_lock(ui_step):
    """
        description:
            sets screen lock method to PIN <selected PIN>
            if already set to PIN, it will skip
    """
    def __init__(self, dut_pin = "1234", require_pin_to_start_device = False,
                 wait_time = 5000, **kwargs):
        ui_step.__init__(self, **kwargs)
        self.dut_pin = dut_pin
        self.require_pin_to_start_device = require_pin_to_start_device
        self.wait_time = wait_time

    def do(self):
        if self.uidevice(resourceId = "android:id/summary", text = "Swipe").exists:
            ui_steps.click_button_with_scroll(serial = self.serial,
                                              view_to_find = {"resourceId":
                                                              "android:id/title",
                                                              "text": "Screen lock"})()
            if self.uidevice(text = "Choose screen lock").wait.exists(timeout = self.wait_time):
                ui_steps.click_button(serial = self.serial,
                                      view_to_find = {"resourceId": "android:id/title",
                                                      "text": "PIN"},
                                      view_to_check = {"text": "Secure start-up"})()
                # if not ui_utils.is_radio_button_enabled(serial = self.serial, instance = 1):
                if self.require_pin_to_start_device:
                    ui_steps.click_button(serial = self.serial,
                                          view_to_find = {"resourceId":
                                                          "com.android.settings:id/encrypt_require_password",
                                                          "text": "Require PIN to start device"})()
                    if self.uidevice(resourceId = "android:id/alertTitle",
                                     text = "Require password?").wait.exists(timeout = self.wait_time):
                        ui_steps.click_button(serial = self.serial,
                                              view_to_find = {"resourceId": "android:id/button1", "text": "OK"})()
                else:
                    ui_steps.click_button(serial = self.serial,
                                          view_to_find = {"resourceId":
                                                          "com.android.settings:id/encrypt_dont_require_password",
                                                          "text": "No thanks"})()
                ui_steps.click_button(serial = self.serial,
                                      view_to_find = {"resourceId":
                                                      "com.android.settings:id/next_button",
                                                      "text": "Continue"},
                                      view_to_check = {"resourceId":
                                                       "com.android.settings:id/headerText",
                                                       "text": "PIN must be at least 4 digits"})()
                ui_steps.edit_text(serial = self.serial,
                                   view_to_find = {"resourceId":
                                                   "com.android.settings:id/password_entry"},
                                   value = self.dut_pin,
                                   is_password = True)()
                ui_steps.click_button(serial = self.serial,
                                      view_to_find = {"resourceId":
                                                      "com.android.settings:id/next_button",
                                                      "text": "Continue"})()
                ui_steps.edit_text(serial = self.serial,
                                   view_to_find = {"resourceId":
                                                   "com.android.settings:id/password_entry"},
                                   value = self.dut_pin,
                                   is_password = True)()
                ui_steps.click_button(serial = self.serial,
                                      view_to_find = {"resourceId":
                                                      "com.android.settings:id/next_button",
                                                      "text": "OK"},
                                      view_to_check = {"text": "Notifications"})()
                if not ui_utils.is_radio_button_enabled(serial = self.serial,
                                                        instance = 0):
                    ui_steps.click_button(serial = self.serial,
                                          view_to_find = {"resourceId":
                                                          "com.android.settings:id/show_all",
                                                          "text": "Show all notification content"})()
                ui_steps.click_button(serial = self.serial,
                                      view_to_find = { "resourceId":
                                                       "com.android.settings:id/next_button",
                                                       "text": "Done"})()

    def check_condition(self):
        return self.uidevice(resourceId = "android:id/summary", text = "PIN").wait.exists(timeout = self.wait_time)


class remove_pin_screen_lock(ui_step):
    """
        description:
            remove screen lock method to PIN
    """
    def __init__(self, dut_pin = "1234", wait_time = 5000, **kwargs):
        ui_step.__init__(self, **kwargs)
        self.dut_pin = dut_pin
        self.wait_time = wait_time

    def do(self):
        if self.uidevice(resourceId = "android:id/summary", text = "PIN").exists:
            ui_steps.click_button_with_scroll(serial = self.serial,
                                              view_to_find = {"resourceId":
                                                              "android:id/title",
                                                              "text": "Screen lock"})()
            if self.uidevice(resourceId = "com.android.settings:id/headerText",
                             text = "Confirm your PIN").wait.exists(timeout = self.wait_time):
                ui_steps.edit_text(serial = self.serial,
                                   view_to_find = {"resourceId":
                                                   "com.android.settings:id/password_entry"},
                                   value = self.dut_pin,
                                   is_password = True)()
                self.uidevice.press("enter")
                time.sleep(3)
                if self.uidevice(text = "Choose screen lock").wait.exists(timeout = self.wait_time):
                    ui_steps.click_button(serial = self.serial,
                                          view_to_find = {"resourceId": "android:id/title",
                                          "text": "Swipe"})()
                    if self.uidevice(resourceId = "android:id/alertTitle",
                                     text = "Remove device protection?").wait.exists(timeout = self.wait_time):
                        ui_steps.click_button(serial = self.serial,
                                              view_to_find = {"resourceId": "android:id/button1",
                                                              "text": "Yes, remove"})()

    def check_condition(self):
        return self.uidevice(resourceId = "android:id/summary",
                                text = "Swipe").wait.exists(timeout = self.wait_time)


class enter_incorrect_pin(ui_step, adb_step):
    """
        description:
            enter incorrect PIN for more than 30 times
    """
    def __init__(self, dut_info = None, dut_pin = "1234", tries = 30,
                 wait_time = 5000, **kwargs):
        ui_step.__init__(self, **kwargs)
        adb_step.__init__(self, **kwargs)
        self.dut_info = dut_info
        self.dut_pin = dut_pin
        self.tries = tries
        self.tmp_number = tries + 1
        self.wait_time = wait_time

    def do(self):
        # adb_steps.put_device_into_sleep_mode(serial = self.serial)()
        # adb_steps.wake_up_device(serial = self.serial)()
        local_steps.command("adb -s {} shell input keyevent 26 | adb -s {} shell input keyevent 26".format(self.serial, self.serial))()
        time.sleep(3)
        if self.dut_info == "bxt":
            adb_steps.swipe(serial = self.serial, sx = 956, sy = 956, ex = 1042, ey = 431)()
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"resourceId":
                                              "com.android.systemui:id/delete_button"})()
        while self.uidevice(resourceId = "com.android.systemui:id/keyguard_message_area").wait.gone(timeout = 40000) and self.tries > 0:
            tmp_result = False
            tmp_string = ""
            while self.dut_pin == tmp_string or tmp_string == "":
                tmp_string = ''.join(random.sample(string.digits, 4))
            for i in tmp_string:
               ui_steps.click_button(serial = self.serial,
                                     view_to_find = {"resourceId":
                                                     "com.android.systemui:id/digit_text",
                                                     "text": i})()
            ui_steps.click_button(serial = self.serial,
                                  view_to_find = {"resourceId":
                                                  "com.android.systemui:id/key_enter"})()
            if (self.tmp_number - 5) == self.tries or (self.tmp_number - 10) >= self.tries:
                if self.uidevice(resourceId = "android:id/message").wait.exists(timeout = self.wait_time):
                    tmp_result = True
                    ui_steps.click_button(serial = self.serial,
                                          view_to_find = {"resourceId": "android:id/button3", "text": "OK"})()
            else:
                if self.uidevice(resourceId = "com.android.systemui:id/keyguard_message_area",
                                 text = "Wrong PIN").wait.exists(timeout = self.wait_time):
                    tmp_result = True
            self.tries -= 1
            if not tmp_result:
                raise Exception("The test result did not achieve the desired results")

    def check_condition(self):
        self.uidevice.press.back()
        return ui_utils.is_device_locked(serial = self.serial)


class enter_correct_pin(ui_step, adb_step):
    """
        description:
            enter correct PIN
    """
    def __init__(self, dut_info = None, dut_pin = "1234", wait_time = 5000, **kwargs):
        ui_step.__init__(self, **kwargs)
        adb_step.__init__(self, **kwargs)
        self.dut_info = dut_info
        self.dut_pin = dut_pin
        self.wait_time = wait_time

    def do(self):
        if self.dut_info == "bxt":
            adb_steps.swipe(serial = self.serial, sx = 956, sy = 956, ex = 1042, ey = 431)()
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"resourceId":
                                              "com.android.systemui:id/delete_button"})()
        for i in self.dut_pin:
            ui_steps.click_button(serial = self.serial,
                                  view_to_find = {"resourceId":
                                                  "com.android.systemui:id/digit_text",
                                                  "text": i})()
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"resourceId":
                                              "com.android.systemui:id/key_enter"})()

    def check_condition(self):
        return self.uidevice(resourceId = "com.android.systemui:id/key_enter").wait.gone(timeout = self.wait_time) and\
               not self.uidevice(resourceId = "android:id/message").wait.exists(timeout = self.wait_time)


class enter_incorrect_pin_to_start_device(ui_step, adb_step):
    """
        description:
            enter incorrect PIN to start device for 10 times
    """
    def __init__(self, dut_pin = "1234", tries = 10, wait_time = 5000, **kwargs):
        ui_step.__init__(self, **kwargs)
        adb_step.__init__(self, **kwargs)
        self.dut_pin = dut_pin
        self.tries = tries
        self.wait_time = wait_time

    def do(self):
        if self.uidevice(resourceId = "com.android.settings:id/status",
                         text = "To start Android, enter your PIN").wait.exists(timeout = 40000):
            while self.tries > 0:
                tmp_string = ""
                while self.dut_pin == tmp_string or tmp_string == "":
                    tmp_string = ''.join(random.sample(string.digits, 4))
                ui_steps.edit_text(serial = self.serial,
                                   view_to_find = {"resourceId":
                                                   "com.android.settings:id/passwordEntry"},
                                   value = tmp_string,
                                   is_password = True)()
                self.uidevice.press("enter")
                time.sleep(3)

    def check_condition(self):
        return self.uidevice(resourceId = "com.android.settings:id/status",
                             text = "To unlock your phone, turn it off and then on.").wait.exists(timeout = self.wait_time)


class set_password_screen_lock(ui_step):
    """
        description:
            sets screen lock method to password <selected Password>
            if already set to password, it will skip
    """
    def __init__(self, dut_password = "test1234", require_password_to_start_device = False,
                 wait_time = 5000, **kwargs):
        ui_step.__init__(self, **kwargs)
        self.dut_password = dut_password
        self.require_password_to_start_device = require_password_to_start_device
        self.wait_time = wait_time

    def do(self):
        if self.uidevice(resourceId = "android:id/summary", text = "Swipe").exists:
            ui_steps.click_button_with_scroll(serial = self.serial,
                                              view_to_find = {"resourceId":
                                                              "android:id/title",
                                                              "text": "Screen lock"})()
            if self.uidevice(text = "Choose screen lock").wait.exists(timeout = self.wait_time):
                ui_steps.click_button(serial = self.serial,
                                      view_to_find = {"resourceId": "android:id/title",
                                                      "text": "Password"},
                                      view_to_check = {"text": "Secure start-up"})()
                # if not ui_utils.is_radio_button_enabled(serial = self.serial, instance = 1):
                if self.require_password_to_start_device:
                    ui_steps.click_button(serial = self.serial,
                                          view_to_find = {"resourceId":
                                                          "com.android.settings:id/encrypt_require_password",
                                                          "text": "Require password to start device"})()
                    if self.uidevice(resourceId = "android:id/alertTitle",
                                     text = "Require password?").wait.exists(timeout = self.wait_time):
                        ui_steps.click_button(serial = self.serial,
                                              view_to_find = {"resourceId": "android:id/button1", "text": "OK"})()
                else:
                    ui_steps.click_button(serial = self.serial,
                                          view_to_find = {"resourceId":
                                                          "com.android.settings:id/encrypt_dont_require_password",
                                                          "text": "No thanks"})()
                ui_steps.click_button(serial = self.serial,
                                      view_to_find = {"resourceId":
                                                      "com.android.settings:id/next_button",
                                                      "text": "Continue"},
                                      view_to_check = {"resourceId":
                                                       "com.android.settings:id/headerText",
                                                       "text": "Password must be at least 4 characters"})()
                ui_steps.edit_text(serial = self.serial,
                                   view_to_find = {"resourceId":
                                                   "com.android.settings:id/password_entry"},
                                   value = self.dut_password,
                                   is_password = True)()
                ui_steps.click_button(serial = self.serial,
                                      view_to_find = {"resourceId":
                                                      "com.android.settings:id/next_button",
                                                      "text": "Continue"})()
                ui_steps.edit_text(serial = self.serial,
                                   view_to_find = {"resourceId":
                                                   "com.android.settings:id/password_entry"},
                                   value = self.dut_password,
                                   is_password = True)()
                ui_steps.click_button(serial = self.serial,
                                      view_to_find = {"resourceId":
                                                      "com.android.settings:id/next_button",
                                                      "text": "OK"},
                                      view_to_check = {"text": "Notifications"})()
                if not ui_utils.is_radio_button_enabled(serial = self.serial,
                                                        instance = 0):
                    ui_steps.click_button(serial = self.serial,
                                          view_to_find = {"resourceId":
                                                          "com.android.settings:id/show_all",
                                                          "text": "Show all notification content"})()
                ui_steps.click_button(serial = self.serial,
                                      view_to_find = { "resourceId":
                                                       "com.android.settings:id/next_button",
                                                       "text": "Done"})()

    def check_condition(self):
        return self.uidevice(resourceId = "android:id/summary", text = "Password").wait.exists(timeout = self.wait_time)


class remove_password_screen_lock(ui_step):
    """
        description:
            remove screen lock method to Password
    """
    def __init__(self, dut_password = "test1234", wait_time = 5000, **kwargs):
        ui_step.__init__(self, **kwargs)
        self.dut_password = dut_password
        self.wait_time = wait_time

    def do(self):
        if self.uidevice(resourceId = "android:id/summary", text = "Password").exists:
            ui_steps.click_button_with_scroll(serial = self.serial,
                                              view_to_find = {"resourceId":
                                                              "android:id/title",
                                                              "text": "Screen lock"})()
            if self.uidevice(resourceId = "com.android.settings:id/headerText",
                             text = "Confirm your password").wait.exists(timeout = self.wait_time):
                ui_steps.edit_text(serial = self.serial,
                                   view_to_find = {"resourceId":
                                                   "com.android.settings:id/password_entry"},
                                   value = self.dut_password,
                                   is_password = True)()
                self.uidevice.press("enter")
                time.sleep(3)
                if self.uidevice(text = "Choose screen lock").wait.exists(timeout = self.wait_time):
                    ui_steps.click_button(serial = self.serial,
                                          view_to_find = {"resourceId": "android:id/title",
                                          "text": "Swipe"})()
                    if self.uidevice(resourceId = "android:id/alertTitle",
                                     text = "Remove device protection?").wait.exists(timeout = self.wait_time):
                        ui_steps.click_button(serial = self.serial,
                                              view_to_find = {"resourceId": "android:id/button1",
                                              "text": "Yes, remove"})()

    def check_condition(self):
        return self.uidevice(resourceId = "android:id/summary",
                                text = "Swipe").wait.exists(timeout = self.wait_time)


class enter_incorrect_password(ui_step, adb_step):
    """
        description:
            enter incorrect password for more than 30 times
    """
    def __init__(self, dut_info = None, dut_password = "test1234", tries = 30,
                 wait_time = 5000, **kwargs):
        ui_step.__init__(self, **kwargs)
        adb_step.__init__(self, **kwargs)
        self.dut_info = dut_info
        self.dut_password = dut_password
        self.tries = tries
        self.tmp_number = tries + 1
        self.wait_time = wait_time

    def do(self):
        # adb_steps.put_device_into_sleep_mode(serial = self.serial)()
        # adb_steps.wake_up_device(serial = self.serial)()
        local_steps.command("adb -s {} shell input keyevent 26 | adb -s {} shell input keyevent 26".format(self.serial, self.serial))()
        time.sleep(3)
        if self.dut_info == "bxt":
            adb_steps.swipe(serial = self.serial, sx = 956, sy = 956, ex = 1042, ey = 431)()
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"resourceId": "com.android.systemui:id/passwordEntry"})()
        while self.uidevice(resourceId = "com.android.systemui:id/keyguard_message_area").wait.gone(timeout = 40000) and self.tries > 0:
            tmp_result = False
            tmp_string = ""
            while self.dut_password == tmp_string or tmp_string == "":
                tmp_string = ''.join(random.sample(string.ascii_letters + string.digits, 8))
            ui_steps.edit_text(serial = self.serial,
                               view_to_find = {"resourceId":
                                               "com.android.systemui:id/passwordEntry"},
                               value = tmp_string,
                               is_password = True)()
            self.uidevice.press("enter")
            time.sleep(3)
            if (self.tmp_number - 5) == self.tries or (self.tmp_number - 10) >= self.tries:
                if self.uidevice(resourceId = "android:id/message").wait.exists(timeout = self.wait_time):
                    tmp_result = True
                    ui_steps.click_button(serial = self.serial,
                                          view_to_find = {"resourceId": "android:id/button3", "text": "OK"})()
            else:
                if self.uidevice(resourceId = "com.android.systemui:id/keyguard_message_area",
                                 text = "Wrong Password").wait.exists(timeout = self.wait_time):
                    tmp_result = True
            self.tries -= 1
            if not tmp_result:
                raise Exception("The test result did not achieve the desired results")

    def check_condition(self):
        self.uidevice.press.back()
        return ui_utils.is_device_locked(serial = self.serial)


class enter_correct_password(ui_step, adb_step):
    """
        description:
            enter correct password
    """
    def __init__(self, dut_info = None, dut_password = "test1234", wait_time = 5000, **kwargs):
        ui_step.__init__(self, **kwargs)
        adb_step.__init__(self, **kwargs)
        self.dut_info = dut_info
        self.dut_password = dut_password
        self.wait_time = wait_time

    def do(self):
        if self.dut_info == "bxt":
            adb_steps.swipe(serial = self.serial, sx = 956, sy = 956, ex = 1042, ey = 431)()
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"resourceId": "com.android.systemui:id/passwordEntry"})()
        ui_steps.edit_text(serial = self.serial,
                           view_to_find = {"resourceId":
                                           "com.android.systemui:id/passwordEntry"},
                           value = self.dut_password,
                           is_password = True)()
        self.uidevice.press("enter")
        time.sleep(3)

    def check_condition(self):
        return self.uidevice(resourceId = "com.android.systemui:id/passwordEntry").wait.gone(timeout = self.wait_time) and\
               not self.uidevice(resourceId = "android:id/message").wait.exists(timeout = self.wait_time)


class set_pattern_screen_lock(ui_step):
    """
        description:
            sets screen lock method to Pattern <selected Pattern>
            if already set to Pattern, it will skip
    """
    def __init__(self, the_first_point = [], the_second_point = [], the_third_point = [],
                 the_fourth_point = [], require_pattern_to_start_device = False,
                 wait_time = 5000, **kwargs):
        ui_step.__init__(self, **kwargs)
        self.the_first_point = the_first_point
        self.the_second_point = the_second_point
        self.the_third_point = the_third_point
        self.the_fourth_point = the_fourth_point
        self.require_pattern_to_start_device = require_pattern_to_start_device
        self.wait_time = wait_time

    def do(self):
        if self.uidevice(resourceId = "android:id/summary", text = "Swipe").exists:
            ui_steps.click_button_with_scroll(serial = self.serial,
                                              view_to_find = {"resourceId":
                                                              "android:id/title",
                                                              "text": "Screen lock"})()
            if self.uidevice(text = "Choose screen lock").wait.exists(timeout = self.wait_time):
                ui_steps.click_button(serial = self.serial,
                                      view_to_find = {"resourceId": "android:id/title",
                                                      "text": "Pattern"},
                                      view_to_check = {"text": "Secure start-up"})()
                # if not ui_utils.is_radio_button_enabled(serial = self.serial, instance = 1):
                if self.require_pattern_to_start_device:
                    ui_steps.click_button(serial = self.serial,
                                          view_to_find = {"resourceId":
                                                          "com.android.settings:id/encrypt_require_password",
                                                          "text": "Require pattern to start device"})()
                    if self.uidevice(resourceId = "android:id/alertTitle",
                                     text = "Require password?").wait.exists(timeout = self.wait_time):
                        ui_steps.click_button(serial = self.serial,
                                              view_to_find = {"resourceId": "android:id/button1", "text": "OK"})()
                else:
                    ui_steps.click_button(serial = self.serial,
                                          view_to_find = {"resourceId":
                                                          "com.android.settings:id/encrypt_dont_require_password",
                                                          "text": "No thanks"})()
                ui_steps.click_button(serial = self.serial,
                                      view_to_find = {"resourceId":
                                                      "com.android.settings:id/next_button",
                                                      "text": "Continue"},
                                      view_to_check = {"resourceId":
                                                       "com.android.settings:id/headerText",
                                                       "text": "Draw an unlock pattern"})()
                points = []
                points.append(self.the_first_point)
                points.append(self.the_second_point)
                points.append(self.the_third_point)
                points.append(self.the_fourth_point)
                self.uidevice.swipePoints(points)
                ui_steps.click_button(serial = self.serial,
                                      view_to_find = {"resourceId":
                                                      "com.android.settings:id/footerRightButton",
                                                      "text": "Continue"})()
                self.uidevice.swipePoints(points)
                ui_steps.click_button(serial = self.serial,
                                      view_to_find = {"resourceId":
                                                      "com.android.settings:id/footerRightButton",
                                                      "text": "Confirm"},
                                      view_to_check = {"text": "Notifications"})()
                if not ui_utils.is_radio_button_enabled(serial = self.serial,
                                                        instance = 0):
                    ui_steps.click_button(serial = self.serial,
                                          view_to_find = {"resourceId":
                                                          "com.android.settings:id/show_all",
                                                          "text": "Show all notification content"})()
                ui_steps.click_button(serial = self.serial,
                                      view_to_find = { "resourceId":
                                                       "com.android.settings:id/next_button",
                                                       "text": "Done"})()

    def check_condition(self):
        return self.uidevice(resourceId = "android:id/summary", text = "Pattern").wait.exists(timeout = self.wait_time)


class remove_pattern_screen_lock(ui_step):
    """
        description:
            remove screen lock method to Pattern
    """
    def __init__(self, the_first_point = [], the_second_point = [], the_third_point = [],
                 the_fourth_point = [], wait_time = 5000, **kwargs):
        ui_step.__init__(self, **kwargs)
        self.the_first_point = the_first_point
        self.the_second_point = the_second_point
        self.the_third_point = the_third_point
        self.the_fourth_point = the_fourth_point
        self.wait_time = wait_time

    def do(self):
        if self.uidevice(resourceId = "android:id/summary", text = "Pattern").exists:
            ui_steps.click_button_with_scroll(serial = self.serial,
                                              view_to_find = {"resourceId":
                                                              "android:id/title",
                                                              "text": "Screen lock"})()
            if self.uidevice(resourceId = "com.android.settings:id/headerText",
                             text = "Confirm your pattern").wait.exists(timeout = self.wait_time):
                points = []
                points.append(self.the_first_point)
                points.append(self.the_second_point)
                points.append(self.the_third_point)
                points.append(self.the_fourth_point)
                self.uidevice.swipePoints(points)
                if self.uidevice(text = "Choose screen lock").wait.exists(timeout = self.wait_time):
                    ui_steps.click_button(serial = self.serial,
                                          view_to_find = {"resourceId": "android:id/title",
                                          "text": "Swipe"})()
                    if self.uidevice(resourceId = "android:id/alertTitle",
                                     text = "Remove device protection?").wait.exists(timeout = self.wait_time):
                        ui_steps.click_button(serial = self.serial,
                                              view_to_find = {"resourceId": "android:id/button1",
                                              "text": "Yes, remove"})()

    def check_condition(self):
        return self.uidevice(resourceId = "android:id/summary",
                                text = "Swipe").wait.exists(timeout = self.wait_time)


class enter_incorrect_pattern(ui_step, adb_step):
    """
        description:
            enter incorrect pattern for more than 30 times
    """
    def __init__(self, the_first_point = [], the_second_point = [], the_third_point = [],
                 the_fourth_point = [], dut_info = None, tries = 30, wait_time = 5000, **kwargs):
        ui_step.__init__(self, **kwargs)
        adb_step.__init__(self, **kwargs)
        self.the_first_point = the_first_point
        self.the_second_point = the_second_point
        self.the_third_point = the_third_point
        self.the_fourth_point = the_fourth_point
        self.dut_info = dut_info
        self.tries = tries
        self.tmp_number = tries + 1
        self.wait_time = wait_time

    def do(self):
        # adb_steps.put_device_into_sleep_mode(serial = self.serial)()
        # adb_steps.wake_up_device(serial = self.serial)()
        local_steps.command("adb -s {} shell input keyevent 26 | adb -s {} shell input keyevent 26".format(self.serial, self.serial))()
        time.sleep(3)
        if self.dut_info == "bxt":
            adb_steps.swipe(serial = self.serial, sx = 956, sy = 956, ex = 1042, ey = 431)()
        points = []
        points.append(self.the_first_point)
        points.append(self.the_second_point)
        points.append(self.the_third_point)
        points.append(self.the_fourth_point)
        while self.uidevice(resourceId = "com.android.systemui:id/keyguard_message_area").wait.gone(timeout = 40000) and self.tries > 0:
            tmp_result = False
            self.uidevice.swipePoints(points)
            if (self.tmp_number - 5) == self.tries or (self.tmp_number - 10) >= self.tries:
                if self.uidevice(resourceId = "android:id/message").wait.exists(timeout = self.wait_time):
                    tmp_result = True
                    ui_steps.click_button(serial = self.serial,
                                          view_to_find = {"resourceId": "android:id/button3", "text": "OK"})()
            else:
                if self.uidevice(resourceId = "com.android.systemui:id/keyguard_message_area",
                                 text = "Wrong Pattern").wait.exists(timeout = self.wait_time):
                    tmp_result = True
            self.tries -= 1
            if not tmp_result:
                raise Exception("The test result did not achieve the desired results")

    def check_condition(self):
        self.uidevice.press.back()
        return ui_utils.is_device_locked(serial = self.serial)


class enter_correct_pattern(ui_step, adb_step):
    """
        description:
            enter correct pattern
    """
    def __init__(self, the_first_point = [], the_second_point = [], the_third_point = [],
                 the_fourth_point = [], dut_info = None, wait_time = 5000, **kwargs):
        ui_step.__init__(self, **kwargs)
        adb_step.__init__(self, **kwargs)
        self.the_first_point = the_first_point
        self.the_second_point = the_second_point
        self.the_third_point = the_third_point
        self.the_fourth_point = the_fourth_point
        self.dut_info = dut_info
        self.wait_time = wait_time

    def do(self):
        if self.dut_info == "bxt":
            adb_steps.swipe(serial = self.serial, sx = 956, sy = 956, ex = 1042, ey = 431)()
        points = []
        points.append(self.the_first_point)
        points.append(self.the_second_point)
        points.append(self.the_third_point)
        points.append(self.the_fourth_point)
        self.uidevice.swipePoints(points)

    def check_condition(self):
        return self.uidevice(resourceId = "com.android.systemui:id/lockPatternView").wait.gone(timeout = self.wait_time) and\
               not self.uidevice(resourceId = "android:id/message").wait.exists(timeout = self.wait_time)


class power_button_instantly_locks(ui_step, adb_step):
    """
        description:
            Open and close the power button instantly locks, check the lock screen.
    """
    def __init__(self, dut_info = None, dut_pin = "1234", wait_time = 5000, **kwargs):
        ui_step.__init__(self, **kwargs)
        adb_step.__init__(self, **kwargs)
        self.dut_info = dut_info
        self.dut_pin = dut_pin
        self.wait_time = wait_time

    def do(self):
        # adb_steps.put_device_into_sleep_mode(serial = self.serial)()
        # adb_steps.wake_up_device(serial = self.serial)()
        local_steps.command("adb -s {} shell input keyevent 26 | adb -s {} shell input keyevent 26".format(self.serial, self.serial))()
        time.sleep(3)
        if not ui_utils.is_device_locked(serial = self.serial):
            raise Exception("The test result did not achieve the desired results")
        local_steps.command("adb -s {} reboot".format(self.serial))()
        local_steps.wait_for_adb(timeout = 300, serial = self.serial)()
        time.sleep(100)
        enter_correct_pin(serial = self.serial, dut_info = self.dut_info, dut_pin = self.dut_pin)()
        ui_steps.open_security_settings(serial = self.serial)()
        ui_steps.click_button_with_scroll(serial = self.serial,
                                          view_to_find = {"resourceId": "android:id/title",
                                                          "text": "Power button instantly locks"})()
        # adb_steps.put_device_into_sleep_mode(serial = self.serial)()
        # adb_steps.wake_up_device(serial = self.serial)()
        local_steps.command("adb -s {} shell input keyevent 26 | adb -s {} shell input keyevent 26".format(self.serial, self.serial))()
        time.sleep(3)
        if ui_utils.is_device_locked(serial = self.serial):
            raise Exception("The test result did not achieve the desired results")
        ui_steps.click_button_with_scroll(serial = self.serial,
                                          view_to_find = {"resourceId": "android:id/title",
                                                          "text": "Power button instantly locks"})()

    def check_condition(self):
        return True


class lock_protected_options(ui_step, adb_step):
    """
        description:
            Lock protected options with PIN or password or pattern
    """
    def __init__(self, dut_pin = "1234", wait_time = 5000, **kwargs):
        ui_step.__init__(self, **kwargs)
        adb_step.__init__(self, **kwargs)
        self.dut_pin = dut_pin
        self.wait_time = wait_time

    def do(self):
        # ui_steps.click_button_with_scroll(serial = self.serial,
        #                                   view_to_find = {"resourceId": "android:id/title",
        #                                                   "text": "Smart Lock"},
        #                                   view_to_check = {"resourceId":
        #                                                    "com.android.settings:id/headerText",
        #                                                    "text": "Confirm your PIN"})()
        # ui_steps.edit_text(serial = self.serial,
        #                    view_to_find = {"resourceId": "com.android.settings:id/password_entry"},
        #                    value = self.dut_pin,
        #                    is_password = True)()
        # self.uidevice.press("enter")
        # time.sleep(3)
        # if self.uidevice(resourceId = "com.google.android.gms:id/trust_agent_onboarding_text_title",
        #                  text = "What is Smart Lock?").wait.exists(timeout = self.wait_time):
        #     ui_steps.click_button(serial = self.serial,
        #                           view_to_find = {"resourceId": "com.google.android.gms:id/trust_agent_onboarding_got_it_button",
        #                                           "text": "GOT IT"})()
        # if not self.uidevice(text = "Smart Lock").wait.exists(timeout = self.wait_time):
        #     raise Exception("The test result did not achieve the desired results")
        # # self.uidevice.press.back()
        # ui_steps.open_security_settings(serial = self.serial)()
        # ui_steps.click_button_with_scroll(serial = self.serial,
        #                                   view_to_find = {"resourceId": "android:id/title",
        #                                                   "text": "Screen lock"},
        #                                   view_to_check = {"resourceId":
        #                                                    "com.android.settings:id/headerText",
        #                                                    "text": "Confirm your PIN"})()
        # ui_steps.edit_text(serial = self.serial,
        #                    view_to_find = {"resourceId": "com.android.settings:id/password_entry"},
        #                    value = self.dut_pin,
        #                    is_password = True)()
        # self.uidevice.press("enter")
        # time.sleep(3)
        # if not self.uidevice(text = "Choose screen lock").wait.exists(timeout = self.wait_time):
        #     raise Exception("The test result did not achieve the desired results")
        # self.uidevice.press.back()
        # self.uidevice.press.back()
        ui_steps.open_settings(serial = self.serial)()
        ui_steps.click_button_with_scroll(serial = self.serial,
                                          view_to_find = {"resourceId": "com.android.settings:id/title",
                                                          "text": "About phone"},
                                          view_to_check = {"text": "About phone"})()
        for i in range(10):
            ui_steps.click_button_with_scroll(serial = self.serial,
                                              view_to_find = {"resourceId": "android:id/title",
                                                              "text": "Build number"})()
        self.uidevice.press.back()
        ui_steps.click_button_with_scroll(serial = self.serial,
                                          view_to_find = {"resourceId": "com.android.settings:id/title",
                                                          "text": "Developer options"},
                                          view_to_check = {"text": "Developer options"})()
        ui_steps.click_button_with_scroll(serial = self.serial,
                                          view_to_find = {"resourceId": "android:id/title",
                                                          "text": "OEM unlocking"},
                                          view_to_check = {"resourceId": "com.android.settings:id/headerText",
                                                           "text": "Confirm your PIN"})()
        ui_steps.edit_text(serial = self.serial,
                           view_to_find = {"resourceId": "com.android.settings:id/password_entry"},
                           value = self.dut_pin,
                           is_password = True)()
        self.uidevice.press("enter")
        time.sleep(3)
        if not self.uidevice(resourceId = "android:id/alertTitle",
                             text = "Allow OEM unlocking?").wait.exists(timeout = self.wait_time):
            raise Exception("The test result did not achieve the desired results")
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"resourceId": "android:id/button2", "text": "Cancel"})()
        self.uidevice.press.home()

    def check_condition(self):
        return True


class lock_timer(ui_step, adb_step):
    """
        description:
            Lock timer with PIN or password or pattern
    """
    def __init__(self, dut_info = None, dut_pin = "1234", wait_time = 5000, **kwargs):
        ui_step.__init__(self, **kwargs)
        adb_step.__init__(self, **kwargs)
        self.dut_info = dut_info
        self.dut_pin = dut_pin
        self.wait_time = wait_time

    def do(self):
        # self.uidevice.press.back()
        ui_steps.open_settings(serial = self.serial)()
        ui_steps.click_button_with_scroll(serial = self.serial,
                                          view_to_find = {"resourceId": "com.android.settings:id/title",
                                                          "text": "About phone"},
                                          view_to_check = {"text": "About phone"})()
        for i in range(10):
            ui_steps.click_button_with_scroll(serial = self.serial,
                                              view_to_find = {"resourceId": "android:id/title",
                                                              "text": "Build number"})()
        # self.uidevice.press.back()
        ui_steps.open_settings(serial = self.serial)()
        ui_steps.click_button_with_scroll(serial = self.serial,
                                          view_to_find = {"resourceId": "com.android.settings:id/title",
                                                          "text": "Developer options"},
                                          view_to_check = {"text": "Developer options"})()
        ui_steps.click_switch(serial = self.serial,
                              right_of = True,
                              state = "OFF",
                              view_to_find = {"resourceId": "android:id/title",
                                              "text": "Stay awake"})()
        # self.uidevice.press.back()
        ui_steps.open_security_settings(serial = self.serial)()
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"resourceId": "android:id/title",
                                              "text": "Automatically lock"},
                              view_to_check = {"resourceId": "android:id/alertTitle",
                                               "text": "Automatically lock"})()
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"resourceId": "android:id/text1",
                                              "text": "Immediately"},
                              view_to_check = {"resourceId": "android:id/summary",
                                               "text": "Immediately after sleep, except when kept unlocked by Smart Lock"})()
        # adb_steps.put_device_into_sleep_mode(serial = self.serial)()
        # adb_steps.wake_up_device(serial = self.serial)()
        local_steps.command("adb -s {} shell input keyevent 26 | adb -s {} shell input keyevent 26".format(self.serial, self.serial))()
        time.sleep(10)
        if adb_utils.is_power_state(serial = serial, state = "ON"):
            raise Exception("The test result did not achieve the desired results")
        local_steps.command("adb -s {} shell input keyevent 26 | adb -s {} shell input keyevent 26".format(self.serial, self.serial))()
        enter_correct_pin(serial = self.serial, dut_info = self.dut_info, dut_pin = self.dut_pin)()
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"resourceId": "android:id/title",
                                              "text": "Automatically lock"},
                              view_to_check = {"resourceId": "android:id/alertTitle",
                                               "text": "Automatically lock"})()
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"resourceId": "android:id/text1",
                                              "text": "5 minutes"},
                              view_to_check = {"resourceId": "android:id/summary",
                                               "text": "5 minutes after sleep, except when kept unlocked by Smart Lock"})()
        # adb_steps.put_device_into_sleep_mode(serial = self.serial)()
        # adb_steps.wake_up_device(serial = self.serial)()
        local_steps.command("adb -s {} shell input keyevent 26 | adb -s {} shell input keyevent 26".format(self.serial, self.serial))()
        time.sleep(10)
        if adb_utils.is_power_state(serial = serial, state = "OFF"):
            raise Exception("The test result did not achieve the desired results")
        time.sleep(300)
        if adb_utils.is_power_state(serial = serial, state = "ON"):
            raise Exception("The test result did not achieve the desired results")
        local_steps.command("adb -s {} shell input keyevent 26 | adb -s {} shell input keyevent 26".format(self.serial, self.serial))()
        enter_correct_pin(serial = self.serial, dut_info = self.dut_info, dut_pin = self.dut_pin)()
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"resourceId": "android:id/title",
                                              "text": "Automatically lock"},
                              view_to_check = {"resourceId": "android:id/alertTitle",
                                               "text": "Automatically lock"})()
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"resourceId": "android:id/text1",
                                              "text": "30 minutes"},
                              view_to_check = {"resourceId": "android:id/summary",
                                               "text": "30 minutes after sleep, except when kept unlocked by Smart Lock"})()
        # self.uidevice.press.back()
        ui_steps.open_settings(serial = self.serial)()
        ui_steps.click_button_with_scroll(serial = self.serial,
                                          view_to_find = {"resourceId": "com.android.settings:id/title",
                                                          "text": "Developer options"},
                                          view_to_check = {"text": "Developer options"})()
        ui_steps.click_switch(serial = self.serial,
                              right_of = True,
                              state = "ON",
                              view_to_find = {"resourceId": "android:id/title",
                                              "text": "Stay awake"})()
        # self.uidevice.press.back()
        ui_steps.open_security_settings(serial = self.serial)()

    def check_condition(self):
        return True


class screen_lock_to_no_password(ui_step, adb_step):
    """
        description:
            Screen lock to no password
    """
    def __init__(self, dut_info = None, dut_pin = "1234", tries = 1,
                 wait_time = 5000, **kwargs):
        ui_step.__init__(self, **kwargs)
        adb_step.__init__(self, **kwargs)
        self.dut_info = dut_info
        self.dut_pin = dut_pin
        self.tries = tries
        self.wait_time = wait_time

    def do(self):
        enter_incorrect_pin(serial = self.serial, dut_info = self.dut_info,
                            dut_pin = self.dut_pin, tries = self.tries)()
        local_steps.command("adb -s {} reboot".format(self.serial))()
        local_steps.wait_for_adb(timeout = 300, serial = self.serial)()
        time.sleep(100)
        enter_correct_pin(serial = self.serial, dut_info = self.dut_info, dut_pin = self.dut_pin)()
        ui_steps.open_security_settings(serial = self.serial)()
        if self.uidevice(resourceId = "android:id/summary", text = "PIN").exists:
            ui_steps.click_button_with_scroll(serial = self.serial,
                                              view_to_find = {"resourceId":
                                                              "android:id/title",
                                                              "text": "Screen lock"})()
            if self.uidevice(resourceId = "com.android.settings:id/headerText",
                             text = "Confirm your PIN").wait.exists(timeout = self.wait_time):
                ui_steps.edit_text(serial = self.serial,
                                   view_to_find = {"resourceId":
                                                   "com.android.settings:id/password_entry"},
                                   value = self.dut_pin,
                                   is_password = True)()
                self.uidevice.press("enter")
                time.sleep(3)
                ui_steps.click_button(serial = self.serial,
                                      view_to_find = {"resourceId": "android:id/title",
                                                      "text": "None"})()
                if self.uidevice(resourceId = "android:id/alertTitle",
                                 text = "Remove device protection?").wait.exists(timeout = self.wait_time):
                    ui_steps.click_button(serial = self.serial,
                                          view_to_find = {"resourceId": "android:id/button1",
                                                          "text": "Yes, remove"})()
                    # adb_steps.put_device_into_sleep_mode(serial = self.serial)()
                    # adb_steps.wake_up_device(serial = self.serial)()
                    local_steps.command("adb -s {} shell input keyevent 26 | adb -s {} shell input keyevent 26".format(self.serial, self.serial))()
                    time.sleep(3)
                    if ui_utils.is_device_locked(serial = self.serial):
                        raise Exception("The test result did not achieve the desired results")
        ui_steps.click_button_with_scroll(serial = self.serial,
                                          view_to_find = {"resourceId":
                                                          "android:id/title",
                                                          "text": "Screen lock"})()
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"resourceId": "android:id/title",
                                              "text": "Swipe"})()

    def check_condition(self):
        return True


class screen_lock_update(ui_step, adb_step):
    """
        description:
            Screen lock with PIN or password or pattern update
    """
    def __init__(self, dut_info = None, dut_pin = "1234", dut_password = "test1234",
                 tries = 1, times = 10, wait_time = 5000, **kwargs):
        ui_step.__init__(self, **kwargs)
        adb_step.__init__(self, **kwargs)
        self.dut_info = dut_info
        self.dut_pin = dut_pin
        self.dut_password = dut_password
        self.tries = tries
        self.times = times
        self.wait_time = wait_time

    def do(self):
        while self.times > 0:
            ui_steps.open_security_settings(serial = self.serial)()
            set_pin_screen_lock(serial = self.serial)()
            enter_incorrect_pin(serial = self.serial, dut_info = self.dut_info,
                                dut_pin = self.dut_pin, tries = self.tries)()
            enter_correct_pin(serial = self.serial, dut_info = self.dut_info, dut_pin = self.dut_pin)()
            if self.uidevice(resourceId = "android:id/summary", text = "PIN").exists:
                ui_steps.click_button_with_scroll(serial = self.serial,
                                                  view_to_find = {"resourceId":
                                                                  "android:id/title",
                                                                  "text": "Screen lock"})()
                if self.uidevice(resourceId = "com.android.settings:id/headerText",
                                 text = "Confirm your PIN").wait.exists(timeout = self.wait_time):
                    ui_steps.edit_text(serial = self.serial,
                                       view_to_find = {"resourceId":
                                                       "com.android.settings:id/password_entry"},
                                       value = self.dut_pin,
                                       is_password = True)()
                    self.uidevice.press("enter")
                    time.sleep(3)
                    ui_steps.click_button(serial = self.serial,
                                          view_to_find = {"resourceId": "android:id/title",
                                                          "text": "Swipe"})()
                    if self.uidevice(resourceId = "android:id/alertTitle",
                                     text = "Remove device protection?").wait.exists(timeout = self.wait_time):
                        ui_steps.click_button(serial = self.serial,
                                              view_to_find = {"resourceId": "android:id/button1",
                                                              "text": "Yes, remove"})()
            set_password_screen_lock(serial = self.serial)()
            remove_password_screen_lock(serial = self.serial)()
            local_steps.command("adb -s {} reboot".format(self.serial))()
            local_steps.wait_for_adb(timeout = 300, serial = self.serial)()
            time.sleep(100)
            if self.dut_info == "bxt":
                adb_steps.swipe(serial = self.serial, sx = 956, sy = 956, ex = 1042, ey = 431)()
            self.times -= 1

    def check_condition(self):
        return True


class reboot_system(ui_step, adb_step):
    """
        description:
            Reboot system os
    """
    def __init__(self, **kwargs):
        ui_step.__init__(self, **kwargs)
        adb_step.__init__(self, **kwargs)
        self.platform_name = security_utils.get_platform_name()

    def do(self):
        local_steps.command("adb -s {} reboot".format(self.serial))()
        local_steps.wait_for_adb(timeout = 300, serial = self.serial)()
        time.sleep(100)
        if self.platform_name == "bxtp_abl":
            adb_steps.swipe(serial = self.serial, sx = 956, sy = 956, ex = 1042, ey = 431)()
        if self.platform_name == "gordon_peak":
            if self.uidevice(text = "Drive safely").exists:
                ui_steps.click_button(serial = self.serial,
                                      view_to_find = {"resourceId": "com.android.systemui:id/user_name",
                                                      "text": "Owner"})()

    def check_condition(self):
        return not ui_utils.is_device_locked(serial = self.serial)


class screen_lock_update_with_no_changes(ui_step, adb_step):
    """
        description:
            Screen lock PIN or password or pattern update with no changes
    """
    def __init__(self, dut_info = None, dut_pin = "1234", tries = 1,
                 times = 4, wait_time = 5000, **kwargs):
        ui_step.__init__(self, **kwargs)
        adb_step.__init__(self, **kwargs)
        self.dut_info = dut_info
        self.dut_pin = dut_pin
        self.tries = tries
        self.times = times
        self.wait_time = wait_time

    def do(self):
        while self.times > 0:
            ui_steps.open_security_settings(serial = self.serial)()
            set_pin_screen_lock(serial = self.serial)()
            enter_incorrect_pin(serial = self.serial, dut_info = self.dut_info,
                                dut_pin = self.dut_pin, tries = self.tries)()
            enter_correct_pin(serial = self.serial, dut_info = self.dut_info, dut_pin = self.dut_pin)()
            if self.uidevice(resourceId = "android:id/summary", text = "PIN").exists:
                ui_steps.click_button_with_scroll(serial = self.serial,
                                                  view_to_find = {"resourceId":
                                                                  "android:id/title",
                                                                  "text": "Screen lock"})()
                if self.uidevice(resourceId = "com.android.settings:id/headerText",
                                 text = "Confirm your PIN").wait.exists(timeout = self.wait_time):
                    ui_steps.edit_text(serial = self.serial,
                                       view_to_find = {"resourceId":
                                                       "com.android.settings:id/password_entry"},
                                       value = self.dut_pin,
                                       is_password = True)()
                    self.uidevice.press("enter")
                    time.sleep(3)
                    ui_steps.click_button(serial = self.serial,
                                          view_to_find = {"resourceId": "android:id/title",
                                                          "text": "PIN"},
                                          view_to_check = {"text": "Secure start-up"})()
                    ui_steps.click_button(serial = self.serial,
                                          view_to_find = {"resourceId":
                                                          "com.android.settings:id/encrypt_dont_require_password",
                                                          "text": "No thanks"})()
                    ui_steps.click_button(serial = self.serial,
                                          view_to_find = {"resourceId":
                                                          "com.android.settings:id/next_button",
                                                          "text": "Continue"},
                                          view_to_check = {"resourceId":
                                                           "com.android.settings:id/headerText",
                                                           "text": "PIN must be at least 4 digits"})()
                    ui_steps.edit_text(serial = self.serial,
                                       view_to_find = {"resourceId":
                                                       "com.android.settings:id/password_entry"},
                                       value = self.dut_pin,
                                       is_password = True)()
                    ui_steps.click_button(serial = self.serial,
                                          view_to_find = {"resourceId":
                                                          "com.android.settings:id/next_button",
                                                          "text": "Continue"})()
                    ui_steps.edit_text(serial = self.serial,
                                       view_to_find = {"resourceId":
                                                       "com.android.settings:id/password_entry"},
                                       value = self.dut_pin,
                                       is_password = True)()
                    ui_steps.click_button(serial = self.serial,
                                          view_to_find = {"resourceId":
                                                          "com.android.settings:id/next_button",
                                                          "text": "OK"})()
            remove_pin_screen_lock(serial = self.serial)()
            reboot_system(serial = self.serial)()
            self.times -= 1

    def check_condition(self):
        return True


class disk_encryption_flag_check(ui_step):
    """
        description:
            Disk encryption flag check
    """
    def __init__(self, platform_name = None, wait_time = 5000, **kwargs):
        ui_step.__init__(self, **kwargs)
        self.platform_name = platform_name
        self.wait_time = wait_time

    def do(self):
        if self.platform_name == "gordon_peak":
            ui_steps.click_button_with_scroll(serial = self.serial,
                                              view_to_find = {"resourceId":
                                                              "android:id/title",
                                                              "text": "Encryption & credentials"})()
        ui_steps.scroll_up_to_view(serial = self.serial,
                                   sx = 300, sy = 400, ex = 300, ey = 300, iterations = 100,
                                   view_to_check= {"resourceId": "android:id/summary",
                                                   "text": "Encrypted"})()

    def check_condition(self):
        return self.uidevice(resourceId = "android:id/summary",
                             text = "Encrypted").wait.exists(timeout = self.wait_time)


class hardware_backed_keystore(ui_step):
    """
        description:
            Hardware backed keystore
    """
    def __init__(self, wait_time = 5000, **kwargs):
        ui_step.__init__(self, **kwargs)
        self.wait_time = wait_time

    def do(self):
        ui_steps.scroll_up_to_view(serial = self.serial,
                                   sx = 300, sy = 400, ex = 300, ey = 300, iterations = 100,
                                   view_to_check= {"resourceId": "android:id/summary",
                                                   "text": "Hardware-backed"})()

    def check_condition(self):
        return self.uidevice(resourceId = "android:id/summary",
                             text = "Hardware-backed").wait.exists(timeout = self.wait_time)


class initialize_environment(ui_step):
    """
        description:
            Initialize the test environment
    """
    def __init__(self, wait_time = 5000, **kwargs):
        ui_step.__init__(self, **kwargs)
        self.wait_time = wait_time

    def do(self):
        local_steps.command("adb -s {} reboot".format(self.serial))()
        local_steps.wait_for_adb(timeout = 300, serial = self.serial)()
        time.sleep(100)
        ui_steps.open_security_settings(serial = self.serial)()
        ui_steps.click_button_with_scroll(serial = self.serial,
                                          view_to_find = {"resourceId": "android:id/title",
                                                          "text": "Screen lock"})()
        ui_steps.click_button_with_scroll(serial = self.serial,
                                          view_to_find = {"resourceId": "android:id/title",
                                                          "text": "Swipe"})()
        # self.uidevice.press.back()
        ui_steps.open_settings(serial = self.serial)()
        ui_steps.click_button_with_scroll(serial = self.serial,
                                          view_to_find = {"resourceId": "com.android.settings:id/title",
                                                          "text": "Language & input"})()
        ui_steps.click_button_with_scroll(serial = self.serial,
                                          view_to_find = {"resourceId": "android:id/title",
                                                          "text": "Current Keyboard"})()
        ui_steps.wait_for_view(view_to_find = {"text": "Hardware"}, serial = self.serial)()
        if self.uidevice(text = "Hardware").right(className = "android.widget.Switch").info["text"] != "ON":
            self.uidevice(text = "Hardware").right(className = "android.widget.Switch").click.wait()
        # self.uidevice.press.back()
        self.uidevice.press.home()

    def check_condition(self):
        return True


class initialize_environment_on_ivio(ui_step):
    """
        description:
            Initialize the test environment on IVI-O
    """
    def __init__(self, wait_time = 5000, **kwargs):
        ui_step.__init__(self, **kwargs)
        self.wait_time = wait_time

    def do(self):
        local_steps.command("adb -s {} reboot".format(self.serial))()
        local_steps.wait_for_adb(timeout = 300, serial = self.serial)()
        time.sleep(100)
        if self.uidevice(text = "Drive safely").exists:
            ui_steps.click_button(serial = self.serial,
                                  view_to_find = {"resourceId": "com.android.systemui:id/user_name",
                                                  "text": "Owner"})()
        ui_steps.open_settings(serial = self.serial, settings_check_point = "Suggestions")()
        ui_steps.click_button_with_scroll(serial = self.serial,
                                          view_to_find = {"resourceId": "android:id/title",
                                                          "text": "Display"})()
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"resourceId": "android:id/title",
                                              "text": "Sleep"})()
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"resourceId": "com.android.settings:id/text1",
                                              "text": "30 minutes"})()
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"className": "android.widget.ImageButton"})()
        ui_steps.click_button_with_scroll(serial = self.serial,
                                          view_to_find = {"resourceId": "android:id/title",
                                                          "textContains": "Security & Location"})()
        ui_steps.click_button_with_scroll(serial = self.serial,
                                          view_to_find = {"resourceId": "android:id/title",
                                                          "text": "Screen lock"})()
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"resourceId": "android:id/title",
                                              "text": "Swipe"})()
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"className": "android.widget.ImageButton"})()
        ui_steps.click_button_with_scroll(serial = self.serial,
                                          view_to_find = {"resourceId": "android:id/title",
                                                          "text": "System"})()
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"resourceId": "android:id/title",
                                              "text": "Languages & input"})()
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"resourceId": "android:id/title",
                                              "text": "Physical keyboard"})()
        if self.uidevice(text = "Show virtual keyboard").right(className = "android.widget.Switch").info["text"] != "ON":
            self.uidevice(text = "Show virtual keyboard").right(className = "android.widget.Switch").click.wait()
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"className": "android.widget.ImageButton"})()
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"className": "android.widget.ImageButton"})()
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"resourceId": "android:id/title",
                                              "text": "About phone"})()
        for i in range(10):
            ui_steps.click_button(serial = self.serial,
                                  view_to_find = {"resourceId": "android:id/title",
                                                  "text": "Build number"})()
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"className": "android.widget.ImageButton"})()
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"resourceId": "android:id/title",
                                              "text": "Developer options"})()
        if self.uidevice(text = "Stay awake").right(className = "android.widget.Switch").info["text"] != "ON":
            self.uidevice(text = "Stay awake").right(className = "android.widget.Switch").click.wait()
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"className": "android.widget.ImageButton"})()
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"className": "android.widget.ImageButton"})()

    def check_condition(self):
        return True


class minimium_requirement_check_for_pin(ui_step):
    """
        description:
            minimium requirement check for pin of screen lock
    """
    def __init__(self, dut_pin = "123", require_pin_to_start_device = False,
                 wait_time = 5000, **kwargs):
        ui_step.__init__(self, **kwargs)
        self.dut_pin = dut_pin
        self.require_pin_to_start_device = require_pin_to_start_device
        self.wait_time = wait_time

    def do(self):
        if self.uidevice(resourceId = "android:id/summary", text = "Swipe").exists:
            ui_steps.click_button_with_scroll(serial = self.serial,
                                              view_to_find = {"resourceId":
                                                              "android:id/title",
                                                              "text": "Screen lock"})()
            if self.uidevice(text = "Choose screen lock").wait.exists(timeout = self.wait_time):
                ui_steps.click_button(serial = self.serial,
                                      view_to_find = {"resourceId": "android:id/title",
                                                      "text": "PIN"},
                                      view_to_check = {"text": "Secure start-up"})()
                # if not ui_utils.is_radio_button_enabled(serial = self.serial, instance = 1):
                if self.require_pin_to_start_device:
                    ui_steps.click_button(serial = self.serial,
                                          view_to_find = {"resourceId":
                                                          "com.android.settings:id/encrypt_require_password",
                                                          "text": "Require PIN to start device"})()
                    if self.uidevice(resourceId = "android:id/alertTitle",
                                     text = "Require password?").wait.exists(timeout = self.wait_time):
                        ui_steps.click_button(serial = self.serial,
                                              view_to_find = {"resourceId": "android:id/button1", "text": "OK"})()
                else:
                    ui_steps.click_button(serial = self.serial,
                                          view_to_find = {"resourceId":
                                                          "com.android.settings:id/encrypt_dont_require_password",
                                                          "text": "No thanks"})()
                ui_steps.click_button(serial = self.serial,
                                      view_to_find = {"resourceId":
                                                      "com.android.settings:id/next_button",
                                                      "text": "Continue"},
                                      view_to_check = {"resourceId":
                                                       "com.android.settings:id/headerText",
                                                       "text": "PIN must be at least 4 digits"})()
                ui_steps.edit_text(serial = self.serial,
                                   view_to_find = {"resourceId":
                                                   "com.android.settings:id/password_entry"},
                                   value = self.dut_pin,
                                   is_password = True)()
                tmp_result = ui_utils.is_enabled(serial = self.serial,
                                                 view_to_find = {"resourceId":
                                                                 "com.android.settings:id/next_button",
                                                                 "text": "Continue"})
                if tmp_result:
                    raise Exception("The test result did not achieve the desired results")
                ui_steps.click_button(serial = self.serial,
                                      view_to_find = {"resourceId":
                                                      "com.android.settings:id/cancel_button",
                                                      "text": "Cancel"},
                                      view_to_check = {"resourceId": "android:id/summary",
                                                       "text": "Swipe"})()

    def check_condition(self):
        return True


class minimium_requirement_check_for_password(ui_step):
    """
        description:
            minimium requirement check for password of screen lock
    """
    def __init__(self, dut_password = "tes", require_password_to_start_device = False,
                 wait_time = 5000, **kwargs):
        ui_step.__init__(self, **kwargs)
        self.dut_password = dut_password
        self.require_password_to_start_device = require_password_to_start_device
        self.wait_time = wait_time

    def do(self):
        if self.uidevice(resourceId = "android:id/summary", text = "Swipe").exists:
            ui_steps.click_button_with_scroll(serial = self.serial,
                                              view_to_find = {"resourceId":
                                                              "android:id/title",
                                                              "text": "Screen lock"})()
            if self.uidevice(text = "Choose screen lock").wait.exists(timeout = self.wait_time):
                ui_steps.click_button(serial = self.serial,
                                      view_to_find = {"resourceId": "android:id/title",
                                                      "text": "Password"},
                                      view_to_check = {"text": "Secure start-up"})()
                # if not ui_utils.is_radio_button_enabled(serial = self.serial, instance = 1):
                if self.require_password_to_start_device:
                    ui_steps.click_button(serial = self.serial,
                                          view_to_find = {"resourceId":
                                                          "com.android.settings:id/encrypt_require_password",
                                                          "text": "Require password to start device"})()
                    if self.uidevice(resourceId = "android:id/alertTitle",
                                     text = "Require password?").wait.exists(timeout = self.wait_time):
                        ui_steps.click_button(serial = self.serial,
                                              view_to_find = {"resourceId": "android:id/button1", "text": "OK"})()
                else:
                    ui_steps.click_button(serial = self.serial,
                                          view_to_find = {"resourceId":
                                                          "com.android.settings:id/encrypt_dont_require_password",
                                                          "text": "No thanks"})()
                ui_steps.click_button(serial = self.serial,
                                      view_to_find = {"resourceId":
                                                      "com.android.settings:id/next_button",
                                                      "text": "Continue"},
                                      view_to_check = {"resourceId":
                                                       "com.android.settings:id/headerText",
                                                       "text": "Password must be at least 4 characters"})()
                ui_steps.edit_text(serial = self.serial,
                                   view_to_find = {"resourceId":
                                                   "com.android.settings:id/password_entry"},
                                   value = self.dut_password,
                                   is_password = True)()
                tmp_result = ui_utils.is_enabled(serial = self.serial,
                                                 view_to_find = {"resourceId":
                                                                 "com.android.settings:id/next_button",
                                                                 "text": "Continue"})
                if tmp_result:
                    raise Exception("The test result did not achieve the desired results")
                ui_steps.click_button(serial = self.serial,
                                      view_to_find = {"resourceId":
                                                      "com.android.settings:id/cancel_button",
                                                      "text": "Cancel"},
                                      view_to_check = {"resourceId": "android:id/summary",
                                                       "text": "Swipe"})()

    def check_condition(self):
        return True


class minimium_requirement_check_for_pattern(ui_step):
    """
        description:
            minimium requirement check for pattern of screen lock
    """
    def __init__(self, x = 780, y = 350, require_pattern_to_start_device = False,
                 wait_time = 5000, **kwargs):
        ui_step.__init__(self, **kwargs)
        self.x = x
        self.y = y
        self.require_pattern_to_start_device = require_pattern_to_start_device
        self.wait_time = wait_time

    def do(self):
        if self.uidevice(resourceId = "android:id/summary", text = "Swipe").exists:
            ui_steps.click_button_with_scroll(serial = self.serial,
                                              view_to_find = {"resourceId":
                                                              "android:id/title",
                                                              "text": "Screen lock"})()
            if self.uidevice(text = "Choose screen lock").wait.exists(timeout = self.wait_time):
                ui_steps.click_button(serial = self.serial,
                                      view_to_find = {"resourceId": "android:id/title",
                                                      "text": "Pattern"},
                                      view_to_check = {"text": "Secure start-up"})()
                # if not ui_utils.is_radio_button_enabled(serial = self.serial, instance = 1):
                if self.require_pattern_to_start_device:
                    ui_steps.click_button(serial = self.serial,
                                          view_to_find = {"resourceId":
                                                          "com.android.settings:id/encrypt_require_password",
                                                          "text": "Require pattern to start device"})()
                    if self.uidevice(resourceId = "android:id/alertTitle",
                                     text = "Require password?").wait.exists(timeout = self.wait_time):
                        ui_steps.click_button(serial = self.serial,
                                              view_to_find = {"resourceId": "android:id/button1", "text": "OK"})()
                else:
                    ui_steps.click_button(serial = self.serial,
                                          view_to_find = {"resourceId":
                                                          "com.android.settings:id/encrypt_dont_require_password",
                                                          "text": "No thanks"})()
                ui_steps.click_button(serial = self.serial,
                                      view_to_find = {"resourceId":
                                                      "com.android.settings:id/next_button",
                                                      "text": "Continue"},
                                      view_to_check = {"resourceId":
                                                       "com.android.settings:id/headerText",
                                                       "text": "Draw an unlock pattern"})()
                ui_steps.click_xy(serial = self.serial, x = self.x, y = self.y,
                                   view_to_check = {"resourceId":
                                                   "com.android.settings:id/headerText",
                                                   "text": "Connect at least 4 dots. Try again."})()
                tmp_result = ui_utils.is_enabled(serial = self.serial,
                                                 view_to_find = {"resourceId":
                                                                 "com.android.settings:id/footerRightButton",
                                                                 "text": "Continue"})
                if tmp_result:
                    raise Exception("The test result did not achieve the desired results")
                ui_steps.click_button(serial = self.serial,
                                      view_to_find = {"resourceId":
                                                      "com.android.settings:id/footerLeftButton",
                                                      "text": "Clear"})()
                ui_steps.click_button(serial = self.serial,
                                      view_to_find = {"resourceId":
                                                      "com.android.settings:id/footerLeftButton",
                                                      "text": "Cancel"},
                                      view_to_check = {"resourceId": "android:id/summary",
                                                       "text": "Swipe"})()

    def check_condition(self):
        return True


class minimium_requirement_check_for_pin_on_ivio(ui_step):
    """
        description:
            minimium requirement check for pin of screen lock on IVI-O
    """
    def __init__(self, dut_pin = "123", require_pin_to_start_device = False,
                 wait_time = 5000, **kwargs):
        ui_step.__init__(self, **kwargs)
        self.dut_pin = dut_pin
        self.require_pin_to_start_device = require_pin_to_start_device
        self.wait_time = wait_time

    def do(self):
        time.sleep(3)
        if self.uidevice(resourceId = "android:id/summary", text = "Swipe").exists:
            ui_steps.click_button_with_scroll(serial = self.serial,
                                              view_to_find = {"resourceId":
                                                              "android:id/title",
                                                              "text": "Screen lock"})()
            if self.uidevice(text = "Choose screen lock").wait.exists(timeout = self.wait_time):
                ui_steps.click_button(serial = self.serial,
                                      view_to_find = {"resourceId": "android:id/title",
                                                      "text": "PIN"},
                                      view_to_check = {"text": "Secure start-up"})()
                if self.require_pin_to_start_device:
                    ui_steps.click_button(serial = self.serial,
                                          view_to_find = {"resourceId":
                                                          "com.android.settings:id/encrypt_require_password",
                                                          "text": "YES"},
                                          view_to_check = {"resourceId": "com.android.settings:id/description_text",
                                                           "text": "PIN must be at least 4 digits"})()
                else:
                    ui_steps.click_button(serial = self.serial,
                                          view_to_find = {"resourceId":
                                                          "com.android.settings:id/encrypt_dont_require_password",
                                                          "text": "NO"},
                                          view_to_check = {"resourceId": "com.android.settings:id/description_text",
                                                           "text": "PIN must be at least 4 digits"})()
                ui_steps.edit_text(serial = self.serial,
                                   view_to_find = {"resourceId":
                                                   "com.android.settings:id/password_entry"},
                                   value = self.dut_pin,
                                   is_password = True)()
                tmp_result = ui_utils.is_enabled(serial = self.serial,
                                                 view_to_find = {"resourceId":
                                                                 "com.android.settings:id/next_button",
                                                                 "text": "CONTINUE"})
                if tmp_result:
                    raise Exception("The test result did not achieve the desired results")
                ui_steps.click_button(serial = self.serial,
                                      view_to_find = {"resourceId":
                                                      "com.android.settings:id/cancel_button",
                                                      "text": "CANCEL"})()
                self.uidevice.press.back()

    def check_condition(self):
        return True


class minimium_requirement_check_for_password_on_ivio(ui_step):
    """
        description:
            minimium requirement check for password of screen lock on IVI-O
    """
    def __init__(self, dut_password = "tes", require_password_to_start_device = False,
                 wait_time = 5000, **kwargs):
        ui_step.__init__(self, **kwargs)
        self.dut_password = dut_password
        self.require_password_to_start_device = require_password_to_start_device
        self.wait_time = wait_time

    def do(self):
        time.sleep(3)
        if self.uidevice(resourceId = "android:id/summary", text = "Swipe").exists:
            ui_steps.click_button_with_scroll(serial = self.serial,
                                              view_to_find = {"resourceId":
                                                              "android:id/title",
                                                              "text": "Screen lock"})()
            if self.uidevice(text = "Choose screen lock").wait.exists(timeout = self.wait_time):
                ui_steps.click_button(serial = self.serial,
                                      view_to_find = {"resourceId": "android:id/title",
                                                      "text": "Password"},
                                      view_to_check = {"text": "Secure start-up"})()
                if self.require_password_to_start_device:
                    ui_steps.click_button(serial = self.serial,
                                          view_to_find = {"resourceId":
                                                          "com.android.settings:id/encrypt_require_password",
                                                          "text": "YES"},
                                          view_to_check = {"resourceId": "com.android.settings:id/description_text",
                                                           "text": "Must be at least 4 characters"})()
                else:
                    ui_steps.click_button(serial = self.serial,
                                          view_to_find = {"resourceId":
                                                          "com.android.settings:id/encrypt_dont_require_password",
                                                          "text": "NO"},
                                          view_to_check = {"resourceId": "com.android.settings:id/description_text",
                                                           "text": "Must be at least 4 characters"})()
                ui_steps.edit_text(serial = self.serial,
                                   view_to_find = {"resourceId":
                                                   "com.android.settings:id/password_entry"},
                                   value = self.dut_password,
                                   is_password = True)()
                tmp_result = ui_utils.is_enabled(serial = self.serial,
                                                 view_to_find = {"resourceId":
                                                                 "com.android.settings:id/next_button",
                                                                 "text": "CONTINUE"})
                if tmp_result:
                    raise Exception("The test result did not achieve the desired results")
                ui_steps.click_button(serial = self.serial,
                                      view_to_find = {"resourceId":
                                                      "com.android.settings:id/cancel_button",
                                                      "text": "CANCEL"})()
                self.uidevice.press.back()

    def check_condition(self):
        return True


class minimium_requirement_check_for_pattern_on_ivio(ui_step):
    """
        description:
            minimium requirement check for pattern of screen lock on IVI-O
    """
    def __init__(self, x = 885, y = 540, require_pattern_to_start_device = False,
                 wait_time = 5000, **kwargs):
        ui_step.__init__(self, **kwargs)
        self.x = x
        self.y = y
        self.require_pattern_to_start_device = require_pattern_to_start_device
        self.wait_time = wait_time

    def do(self):
        time.sleep(3)
        if self.uidevice(resourceId = "android:id/summary", text = "Swipe").exists:
            ui_steps.click_button_with_scroll(serial = self.serial,
                                              view_to_find = {"resourceId":
                                                              "android:id/title",
                                                              "text": "Screen lock"})()
            if self.uidevice(text = "Choose screen lock").wait.exists(timeout = self.wait_time):
                ui_steps.click_button(serial = self.serial,
                                      view_to_find = {"resourceId": "android:id/title",
                                                      "text": "Pattern"},
                                      view_to_check = {"text": "Secure start-up"})()
                if self.require_pattern_to_start_device:
                    ui_steps.click_button(serial = self.serial,
                                          view_to_find = {"resourceId":
                                                          "com.android.settings:id/encrypt_require_password",
                                                          "text": "YES"},
                                          view_to_check = {"resourceId": "com.android.settings:id/headerText",
                                                           "text": "Draw an unlock pattern"})()
                else:
                    ui_steps.click_button(serial = self.serial,
                                          view_to_find = {"resourceId":
                                                          "com.android.settings:id/encrypt_dont_require_password",
                                                          "text": "NO"},
                                          view_to_check = {"resourceId": "com.android.settings:id/headerText",
                                                           "text": "Draw an unlock pattern"})()
                ui_steps.click_xy(serial = self.serial, x = self.x, y = self.y,
                                   view_to_check = {"resourceId":
                                                   "com.android.settings:id/headerText",
                                                   "text": "Connect at least 4 dots. Try again."})()
                tmp_result = ui_utils.is_enabled(serial = self.serial,
                                                 view_to_find = {"resourceId":
                                                                 "com.android.settings:id/footerRightButton",
                                                                 "text": "CONTINUE"})
                if tmp_result:
                    raise Exception("The test result did not achieve the desired results")
                ui_steps.click_button(serial = self.serial,
                                      view_to_find = {"resourceId":
                                                      "com.android.settings:id/footerLeftButton",
                                                      "text": "CLEAR"})()
                ui_steps.click_button(serial = self.serial,
                                      view_to_find = {"resourceId":
                                                      "com.android.settings:id/footerLeftButton",
                                                      "text": "CANCEL"})()
                self.uidevice.press.back()

    def check_condition(self):
        return True