# -*- coding:utf-8 -*-

'''
@summary: Android crash reporting test.
@since: 07/01/2016
@author: Lijin Xiong
'''

import os,time
from testlib.util.uiatestbase import UIATestBase
from testlib.util.common import g_common_obj
from testlib.androidframework.common import Settings,SystemPopupsAndDialogs,Gmail,EnvironmentUtils,UiAutomatorUtils, AdbUtils,Func_New
from testlib.util.log import Logger
from testlib.util.config import TestConfig
from testlib.gps.common import GPS_Common

### Settings
SETTINGS_SHORTCUT_NAME = "Settings"
SETTINGS_PACKAGE_NAME = "com.android.settings"
SETTINGS_WIFI_SWITCH_RESID = "com.android.settings:id/switch_text"
SETTINGS_OPTION_RESID = "com.android.settings:id/title"
SETTINGS_DATA_USAGE_TXT = "Data usage"
SETTINGS_NAVIGATE_UP_DESC = "Navigate up"
SETTINGS_DEVELOPER_OPTIONS_TXT = "Developer options"
SETTINGS_DEBUG_GPU_OVERDRAW_TXT = "Debug GPU overdraw"
SETTINGS_OFF_TXT = "Off"
SETTINGS_ON_TXT = "On"
SETTINGS_SHOW_OVERDRAW_AREAS_OPTION_TXT = "Show overdraw areas"
SETTINGS_SHOW_AREAS_FOR_DEUTERANOMALY_OPTION_TXT = "Show areas for Deuteranomaly"
SETTINGS_LANGUAGE_AND_INPUT_OPTIONS_TXT = "Language & input"
SETTINGS_STORAGE_AND_USB_OPTIONS_TXT = "Storage & USB"
SETTINGS_PINYIN_KEYBOARD_OPTIONS_TXT = "Google Pinyin Input"
SETTINGS_KEYBOARD_SUB_OPTION_TXT = "Keyboard"
SETTINGS_KEYBOARD_THEME_SUB_OPTION_TXT = "Keyboard theme"
SETTINGS_KEYBOARD_THEME_MATERIAL_LIGHT_TXT = "Material Light"
SETTINGS_KEYBOARD_THEME_MATERIAL_DARK_TXT = "Material Dark"
SETTINGS_CURRENT_KEYBOARD_OPTION_TXT = "Current Keyboard"
SETTINGS_CHOOSE_KEYBOARDS_OPTION_TXT = "Choose keyboards"
SETTINGS_TAKE_BUG_REPORT_OPTION_TXT = "Take bug report"
SETTINGS_TAKE_BUG_REPORT_POPUP_REPORT_BUTTON_TXT = "Report"
SETTINGS_TAKE_BUG_REPORT_POPUP_REPORT_BUTTON_TXT_N = "REPORT"
SETTINGS_TAKE_BUG_REPORT_NOTIFICATION_TXT = "Bug report captured"
SETTINGS_SHOW_GPU_VIEW_UPDATES_TXT = "Show GPU view updates"

### GMail
GMAIL_APP_NAME = "Gmail"
GMAIL_TAKE_BUG_REPORT_ATTACHEMENT_PREFIX_TXT = "bugreport"
GMAIL_TAKE_BUG_REPORT_ATTACHEMENT_SUBTITLE_RESID = "com.google.android.gm:id/attachment_tile_subtitle"
GMAIL_PACKAGE_NAME = "com.google.android.gm"

LOG = Logger.getlogger(__name__)


class Bug_Report(UIATestBase):

    def setUp(self):
        super(Bug_Report, self).setUp()
        self.__test_name = __name__
        print "Setup: %s" % self.__test_name
        self.d = g_common_obj.get_device()
        self.func = Func_New()
        self.conf = TestConfig().read(section='google_account')
        self.username = self.conf.get("user_name")
        self.passwd = self.conf.get("password")
        UiAutomatorUtils.unlock_screen()

    def test_take_bug_report(self):
        GPS_Common().check_if_wifi_connected()
        ga_st = EnvironmentUtils.check_if_google_account_added()
        if not ga_st:
            self.func.add_google_account_mr1(self.username, self.passwd)
        Settings.turn_on_account_auto_sync()
        Settings.take_bug_report()
        # a popup for taking a bug report should appear
        if EnvironmentUtils.get_android_version() == "N":
            self.assertTrue(self.d(text=SETTINGS_TAKE_BUG_REPORT_POPUP_REPORT_BUTTON_TXT_N).wait.exists(timeout=3000))
            self.d(text=SETTINGS_TAKE_BUG_REPORT_POPUP_REPORT_BUTTON_TXT_N).click()
        else:
            self.assertTrue(self.d(text=SETTINGS_TAKE_BUG_REPORT_POPUP_REPORT_BUTTON_TXT).wait.exists(timeout=3000))
            self.d(text=SETTINGS_TAKE_BUG_REPORT_POPUP_REPORT_BUTTON_TXT).click()
        poll_waiting_tries_for_notification = 30
        found_bug_report_notification = False
        while poll_waiting_tries_for_notification > 0:
            self.d.open.notification()
            time.sleep(10)
            while self.d(textContains="is being generated").exists:
                LOG.info("Waiting for bug report to be generated.")
                time.sleep(10)
#             if self.d(text=SETTINGS_TAKE_BUG_REPORT_NOTIFICATION_TXT).wait.exists(timeout=3000):
            if self.d(textStartsWith="Bug report").wait.exists(timeout=3000):
                # bug report was completed
                found_bug_report_notification = True
                break
            self.d.press.home()  # close notifications
            poll_waiting_tries_for_notification -= 1
        self.assertTrue(found_bug_report_notification,
                        "a notification for the bugreport should have appeared")
        # StatusBar.open_notifications()  # notifications already opened after break in for
#         self.assertTrue(self.d(text=SETTINGS_TAKE_BUG_REPORT_NOTIFICATION_TXT).wait.exists(timeout=3000),
        self.assertTrue(self.d(textStartsWith="Bug report").wait.exists(timeout=3000),
                        "a bug report notification should exist in the notifications bar")
#         self.d(text=SETTINGS_TAKE_BUG_REPORT_NOTIFICATION_TXT).click()
        self.d(textStartsWith="Bug report").click()
        # popup to warn not to share sensitive data with untrusted parties
        SystemPopupsAndDialogs.popup_ok()
        # popup for choosing wich app will handle the bugreport
        # there are different types of popup for different Android versions and devices
        SystemPopupsAndDialogs.open_resource_with(GMAIL_APP_NAME)
        time.sleep(15)
        # gmail application must open with attached bug report information
        self.assertTrue(self.d(packageName=GMAIL_PACKAGE_NAME).wait.exists(timeout=5000),
                        "gmail should have opened displaying the send bug report mail")
        time.sleep(5)
        SystemPopupsAndDialogs.popup_skip()
        # Gmail app setup
        time.sleep(5)
        if self.d(resourceId="com.google.android.gm:id/welcome_tour_skip").exists:
            self.d(resourceId="com.google.android.gm:id/welcome_tour_skip").click.wait()
        if self.d(resourceId="com.google.android.gm:id/action_done").exists:
            self.d(resourceId="com.google.android.gm:id/action_done").click.wait()
            self.d.press.back()

        Gmail.select_sync_account_now()
        time.sleep(20)
        self.assertTrue(self.d(textStartsWith=GMAIL_TAKE_BUG_REPORT_ATTACHEMENT_PREFIX_TXT)
                        .wait.exists(timeout=50000),
                        "mail should have an attachement containing bugreport details")

    def test_crash_reporting_bugreport(self):
        bug_report_filename = "bug_report_test.rpt"
        bugreport_sections = {"UPTIME": 0, "MMC PERF": 0, "MEMORY INFO": 0, "CPU INFO": 0, "PROCRANK": 0,
                              "VIRTUAL MEMORY STATS": 0, "VMALLOC INFO": 0, "SLAB INFO": 0, "ZONEINFO": 0,
                              "PAGETYPEINFO": 0, "BUDDYINFO": 0, "FRAGMENTATION INFO": 0, "KERNEL WAKELOCKS": 0,
                              "KERNEL WAKE SOURCES": 0, "KERNEL CPUFREQ": 0, "KERNEL SYNC": 0, "KERNEL LOG": 0,
                              "SHOW MAP": 0, "PROCESSES": 0, "SYSTEM LOG": 0, "EVENT LOG": 0, "pid": 0,
                              "NETWORK": 0, "QTAGUID": 0, "IP RULES": 0, "ROUTE TABLE": 0,
                              "CACHE": 0, "SYSTEM PROPERTIES": 0, "VOLD DUMP": 0, "BINDER": 0, "DUMPSYS": 0, "APP": 0}

        def manage_bugreport_line(bugreport_line):
            for section in bugreport_sections.keys():
                if section in line:
                    bugreport_sections[section] += 1

        try:
            cmd = "bugreport > " + bug_report_filename
            AdbUtils.run_adb_cmd(cmd, False)
            report_file = open(bug_report_filename)
            statinfo = os.stat(bug_report_filename)
            self.assertTrue(statinfo.st_size > 5000000L)  # magic size number
            for line in report_file:
                if "-----" in line:
                    manage_bugreport_line(line)
        finally:
            os.remove(bug_report_filename)
        missing_section = False
        for key, value in bugreport_sections.iteritems():
            if value == 0:
                missing_section = True
                LOG.info("found missing section " + str(key))
#         self.assertFalse(missing_section)