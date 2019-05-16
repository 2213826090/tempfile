"""
@summary: Test browser top website
@since: 12/11/2014
@author: Xiaoye Xu(xiaoyex.xu@intel.com)
"""

import os
import time
from testlib.util.common import g_common_obj


class LanguageSettingImpl:
    """
    Implements Language Setting UI actions.

    """

    setting_pkg_name = "com.android.settings"
    setting_activity_name = "com.android.settings.Settings$InputMethodAndLanguageSettingsActivity"

    class Locator(object):
        """
        Helper class to locate UI object on screen
        """

        def __init__(self, device):
            self.d = device

        @property
        def language_title(self):
            """ UI language_title"""
            return self.d(textMatches="Language & input")

        @property
        def settings_title(self):
            """ UI setting_title"""
            return self.d(textMatches="Settings")

        @property
        def btn_change_keyboard(self):
            """ UI setting_title"""
            return self.d(textMatches="Current Keyboard", \
                resourceId="android:id/title")

        def btn_keyboard(self, keyboard):
            """ UI setting_title"""
            return self.d(textContains=keyboard, \
                resourceId="android:id/text1")

        def btn_keyboard_radio(self, keyboard):
            """ UI setting_title"""
            return self.btn_keyboard(keyboard).\
            left(resourceId="android:id/radio")

        def btn_keyboard_summary(self, keyboard):
            """ UI setting_title"""
            return self.d(textContains=keyboard, resourceId="android:id/summary")

        def btn_keyboard_radio_info(self, keyboard):
            """ UI setting_title"""
            return self.btn_keyboard_radio(keyboard).checked

        @property
        def btn_choose_keyboard(self):
            """ UI setting_title"""
            return self.d(textMatches="Choose keyboards", \
                className="android.widget.Button")

        @property
        def btn_language_input(self):
            """ UI setting_title"""
            return self.d(textMatches="Language & input", \
                resourceId="com.android.settings:id/title")

        def btn_ime(self, ime):
            """ UI setting_title"""
            return self.d(textContains=ime, \
                resourceId="android:id/title")

        def btn_ime_switch(self, ime):
            """ UI setting_title"""
            return self.btn_ime(ime).right(resourceId="android:id/switchWidget")

        def btn_ime_info(self, ime):
            """ UI setting_title"""
            return self.btn_ime_switch(ime).checked

        @property
        def btn_language_choose(self):
            """ UI setting_title"""
            return self.d(textMatches="Language", \
                resourceId="android:id/title")


    #------------------------ end of class Locator ----------------------

    def __init__ (self, cfg):
        self.d = g_common_obj.get_device()
        self.cfg = cfg
        self._locator = LanguageSettingImpl.Locator(self.d)

    def set_orientation_n(self):
        """
        @summary: set orientation as n
        """
        self.d.orientation = "n"

    def launch_settings(self):
        """ Launch Setttings from am """
        print "[INFO] Launch Setttings from am"
        cmd = 'am start -a android.settings.SETTINGS; echo $?'
        res_list = g_common_obj.adb_cmd_capture_msg(cmd).split('\r\n')
        msg_fail = "[FAILURE] Launch Setttings fail"
        ret = int(res_list[-1].strip())
        assert ret == 0, msg_fail
        self._locator.settings_title.wait.exists(timeout=3*1000)
        print "[INFO] Launch Setttings success"

    def launch_from_am(self):
        """ Launch Keyboard & input methods Setttings from am """
        print "[INFO] Launch Keyboard & input methods Setttings from am"
        cmd = 'am start -a android.settings.INPUT_METHOD_SETTINGS; echo $?'
        res_list = g_common_obj.adb_cmd_capture_msg(cmd).split('\r\n')
        msg_fail = "[FAILURE] Launch Keyboard & input Setttings fail"
        ret = int(res_list[-1].strip())
        assert ret == 0, msg_fail
        self._locator.language_title.wait.exists(timeout=3*1000)
        print "[INFO] Launch Keyboard & input Setttings success"

    @staticmethod
    def stop_from_am():
        """
            Stop Wi-Fi Setttings from am
        """
        print "[INFO] Stop Wi-Fi Setttings from am"
        g_common_obj.stop_app_am(LanguageSettingImpl.setting_pkg_name)

    def set_language_input(self, ime, keyboard):
        """
        @summary: switch the input method and set the input method as ime
        """
        if not self.change_the_keyboard(keyboard):
            self._locator.btn_choose_keyboard.click()
            self.add_input_ime(ime)
            self.change_the_keyboard(keyboard)
            if not self._locator.btn_keyboard_radio_info(keyboard):
                self._locator.btn_keyboard_radio(keyboard).click()
            self.d.press.back()
        error_mesg = "The keyboard [%s] method is not click" % keyboard
        time.sleep(3)
        assert self._locator.btn_keyboard_summary(keyboard).exists, error_mesg
        g_common_obj.back_home()

    def change_the_keyboard(self, keyboard):
        """
        @summary: change the keyboard
        """
        self.launch_settings()
        self._locator.btn_language_input.click.wait()
        self._locator.btn_change_keyboard.click()
        error_mesg = "[WARNING] The keyboard [%s] method is not exist" % keyboard
        if not self._locator.btn_keyboard(keyboard).exists:
            print error_mesg
            return False
        if not self._locator.btn_keyboard_radio_info(keyboard):
            self._locator.btn_keyboard_radio(keyboard).click()
        else:
            self.d.press.back()
        print "[INFO] The keyboard [%s] method is exist" % keyboard
        return True

    def add_input_ime(self, ime):
        """
        @summary: change the keyboard
        """
        if not self._locator.btn_ime_info(ime):
            self._locator.btn_ime_switch(ime).click()
            time.sleep(2)
        error_mesg = "The input ime is not selected."
        assert self._locator.btn_ime_info(ime), error_mesg
        self.d.press.home()
