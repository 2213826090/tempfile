# -*- coding:utf-8 -*-

'''
@summary: Android framework test common module.
@since: 06/30/2016
@author: Lijin Xiong
'''

import time,re,os
from testlib.util.common import g_common_obj
import datetime
from testlib.util.log import Logger
from string import Template
from os import path
from testlib.androidframework.logcat_messaging_gateway import ApiTestsMessagingGateway
from testlib.androidframework.adb_utils import AdbUtils
from testlib.androidframework.shell_utils import ShellUtils
from testlib.util.process import shell_command
from testlib.dut_init.dut_init_impl import Function

LOG = Logger.getlogger(__name__)

#keyevents
screen_unlock_cmd = "input keyevent 82"


### Clock
CLOCK_PACKAGE_NAME = "com.google.android.deskclock"
CLOCK_SHORTCUT_NAME = "Clock"
CLOCK_ADD_ALARM_BUTTON_DESC = "Add alarm"
CLOCK_AM_MARKER_TXT = "AM"
CLOCK_PM_MARKER_TXT = "PM"
CLOCK_ALARM_DESC = "Alarm"
CLOCK_EXPAND_ALARM_DESC = "Expand alarm"
CLOCK_DELETE_ALARM_DESC = "Delete alarm"
CLOCK_DISMISS_ALARM_TXT = "Dismiss"
CLOCK_SNOOZE_ALARM_TXT = "Snooze"
CLOCK_SNOOZING_STATUS_TXT = "Snoozing"

### Gmail
GMAIL_SYNC_NOW_RESID = "com.google.android.gm:id/manual_sync"

### Settings
SETTINGS_SHORTCUT_NAME = "Settings"
SETTINGS_PACKAGE_NAME = "com.android.settings"
SETTINGS_WIFI_SWITCH_RESID = "com.android.settings:id/switch_text"
SETTINGS_OPTION_RESID = "com.android.settings:id/title"
SETTINGS_OPTION_WIFI_TEXT = "Wi‑Fi"
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
SETTINGS_TAKE_BUG_REPORT_NOTIFICATION_TXT = "Bug report captured"
SETTINGS_SHOW_GPU_VIEW_UPDATES_TXT = "Show GPU view updates"
SETTINGS_LOCATION_OPTION_TXT = "Location"

### OS
OS_OPEN_JUST_ONCE_TXT = "Just once"
APP_CRASH_POPUP_BEGIN_TXT = "Unfortunately"
APP_CRASH_POPUP_END_TXT = "has stopped"
VERSION_HISTORY = {
    '21': 'L',
    '22': 'L',
    '23': 'M',
    '24': 'N',
    '25': 'N',
    '26': 'O-MR0',
    '27': 'O-MR1'
}

### Misc UI Elements
APPS_BUTTON_DESC = "Apps"
DISMISS_TASK_RESID = "com.android.systemui:id/dismiss_task"
TITLE_ELEMENT_RESID = "android:id/title"
SUMMARY_ELEMENT_RESID = "android:id/summary"
UI_TRUE = "true"
UI_FALSE = "false"
LAUNCHER_WORKSPACE_SCROLL_TYPE = "horizontal"

### Popups
POPUP_OK = "OK"
POPUP_SKIP = "SKIP"

### Misc Elements
APPS_BUTTON_DESC = "Apps"
POPUP_OK = "OK"
DISMISS_TASK_RESID = "com.android.systemui:id/dismiss_task"

MAX_SWIPES_TO_LIMIT = 15

d = g_common_obj.get_device()

CHROME_PACKAGE_NAME = "com.android.chrome"
CHROME_FRAME_LAYOUT_CLASS = "android.widget.FrameLayout"
CHROME_SHORTCUT_NAME = "Chrome"
CHROME_ACCEPT_TERMS_RESID = "com.android.chrome:id/terms_accept"
CHROME_NEGATIVE_BUTTON_RESID = "com.android.chrome:id/negative_button"
CHROME_OPTION_RESID = "com.android.chrome:id/menu_button"
CHROME_EMPTY_OPTION_RESID = "com.android.chrome:id/empty_menu_button"
CHROME_OPTION_DOCUMENT_RESID = "com.android.chrome:id/document_menu_button"
CHROME_NEW_TAB_OPTION_NAME = "New tab"
CHROME_NEW_INCOGNITO_TAB_OPTION_NAME = "New incognito tab"
CHROME_ADDRESS_BAR_RESID = "com.android.chrome:id/url_bar"
CHROME_GOOGLE_SEARCH_TEXTBOX_RESID = "com.android.chrome:id/search_box_text"
CHROME_REFRESH_BUTTON_RESID = "com.android.chrome:id/refresh_button"

class Func_New(Function):
    def add_google_account_mr1(self, username, password):
    # add google account
        g_common_obj.launch_app_am("com.android.settings", ".Settings")
        if not self.d(text="Accounts").exists:
            self.d(scrollable=True).scroll.vert.to(text="Accounts")
        self.d(text="Accounts").click.wait()
        if self.d(text="Google").exists:
            self.d.press.menu()
            if self.d(resourceId="android:id/checkbox").checked:
                self.d(resourceId="android:id/checkbox").click.wait()
                self.d(text="OK").click.wait()
            else:
                self.d.press.back()
            return
        self.d(text="Add account").click.wait()
        self.d(text="Google").click.wait()
        for i in range(20):
            if self.d(description="Add your account").exists:
                break
            assert not self.d(text="Couldn't sign in").exists
            time.sleep(3)
        assert self.d(description="Add your account").exists
        time.sleep(3)
        y = self.d.displayHeight
        x = self.d.displayWidth
        sc = 20
        while not d(resourceId="identifierId").exists:
            time.sleep(3)
            sc -= 1
        self.d(className="android.widget.EditText").set_text(username)
        time.sleep(3)
        self.d.press.enter()
        st = 20
        while not d(resourceId='password').exists:
            time.sleep(3)
            st -= 1
        self.d(className="android.widget.EditText").set_text(password)
        time.sleep(3)
        self.d.press.enter()
        time.sleep(5)
        if self.d(resourceId='next').exists:
            self.d(resourceId='next').click.wait()
        for i in range(50):
            if self.d(resourceId="com.google.android.gms:id/agree_backup").exists and self.d(
                resourceId="com.google.android.gms:id/agree_backup").checked:
                self.d(resourceId="com.google.android.gms:id/agree_backup").click()
                time.sleep(3)
                self.d(resourceId="com.google.android.gms:id/suw_navbar_next").click()
                assert not self.d(text="Couldn't sign in").exists
                assert not self.d(description="Error").exists
                break
            time.sleep(2)
        if self.d(text="Google").exists:
            return
        no_thk = 'No thanks'
        CONTINUE = 'com.android.vending:id/positive_button'
        for i in range(10):
            if self.d(text=no_thk).exists:
                self.d(text=no_thk).click()
                time.sleep(3)
                self.d(resourceId=CONTINUE).click.wait()
                break
            elif self.d(text=no_thk.upper()).exists:
                self.d(text=no_thk.upper()).click()
                time.sleep(3)
                self.d(resourceId=CONTINUE).click.wait()
                break
            time.sleep(2)
        for i in range(10):
            if self.d(text="SKIP").exists:
                self.d(text="SKIP").click.wait()
            if self.d(text="Remind me later").exists:
                self.d(text="Remind me later").click.wait()
                self.d(text="Next").click.wait()
                break
            time.sleep(2)
        g_common_obj.adb_cmd("am force-stop com.android.settings")

class Chrome(object):
    files_acces_allowed = False

    @staticmethod
    def launch():
        if d(packageName=CHROME_PACKAGE_NAME, className=CHROME_FRAME_LAYOUT_CLASS).exists:
            pass
        else:
#             UiAutomatorUtils.launch_app_from_apps_menu(CHROME_SHORTCUT_NAME)
            g_common_obj.launch_app_am("com.android.chrome", "org.chromium.chrome.browser.ChromeTabbedActivity")
        time.sleep(15)
        if d(resourceId=CHROME_ACCEPT_TERMS_RESID).wait.exists(timeout=3000):
            d(resourceId=CHROME_ACCEPT_TERMS_RESID).click()
        if d(resourceId=CHROME_NEGATIVE_BUTTON_RESID).wait.exists(timeout=3000):
            d(resourceId=CHROME_NEGATIVE_BUTTON_RESID).click()
        elif d(resourceId="com.android.chrome:id/positive_button").exists:
            d(resourceId="com.android.chrome:id/positive_button").click()

    @staticmethod
    def go_to_url(url_address, timeout=None, wait_for_page_to_load=True):
        if not d(resourceId=CHROME_ADDRESS_BAR_RESID).wait.exists(timeout=1000):
            if d(resourceId=CHROME_GOOGLE_SEARCH_TEXTBOX_RESID).wait.exists(timeout=3000):
                d(resourceId=CHROME_GOOGLE_SEARCH_TEXTBOX_RESID).click()
            else:
                ScreenSwiper.swipe_down()
        time.sleep(2)
        UiAutomatorUtils.refresh_uiautomator_view_hierarchy()
        d(resourceId=CHROME_ADDRESS_BAR_RESID).wait.exists(timeout=4000)
        d(resourceId=CHROME_ADDRESS_BAR_RESID).set_text(url_address)
        d.press("enter")
        d.wait.idle()
        if wait_for_page_to_load:
            d(resourceId=CHROME_REFRESH_BUTTON_RESID).wait.exists(timeout=30000)
        if timeout is not None and wait_for_page_to_load:
            time.sleep(timeout)

class MomentOfTime(object):
    def __init__(self, sec, min, hour, day, month, year, time_zone):
        self.sec = int(sec)
        self.min = int(min)
        self.hour = int(hour)
        self.day = int(day)
        self.month = int(month)
        self.year = int(year)
        self.time_zone = time_zone
    def copy(self):
        return MomentOfTime(self.sec, self.min, self.hour, self.day, self.month, self.year, self.time_zone)

    def __str__(self):
        return "timezone: %s, year: %s, month: %s, day: %s, hour: %s, minute: %s, second: %s" %\
               (str(self.time_zone), str(self.year), str(self.month), str(self.day),
                str(self.hour), str(self.min), str(self.sec))

class SystemUtils(object):

    @staticmethod
    def get_display_metrics():
        instrumentation_cmd = "am instrument -e class com.intel.test.apitests.tests.DisplayMetricsTestDriver#" \
            "testDisplayAllFields -w com.intel.test.apitests/com.intel.test.apitests.runners.DisplayMetricsTestRunner"
        AdbUtils.run_adb_cmd(instrumentation_cmd)
        display_metrics_logs_cmd = "logcat -d | grep DisplayMetricsTestDriver"
        output = AdbUtils.run_adb_cmd(display_metrics_logs_cmd, adb_shell=False)
        output_lines = output.splitlines()
        output_lines.reverse()
        usable_lines_index = 0
        for i in range(len(output_lines)):
            if "created DisplayMetricsTestDriver class" in output_lines[i]:
                usable_lines_index = i
                break
        if usable_lines_index != 0:
            output_lines = output_lines[:usable_lines_index]
        prop_finder_regex = r': ([a-zA-Z_\d]+) = ([\d.]+)'
        metrics = {}
        for line in output_lines:
            prop = re.findall(prop_finder_regex, line)
            if len(prop) > 0:
                metrics[prop[0][0]] = prop[0][1]
        return metrics

    @staticmethod
    def get_platform_name():
        get_product_cmd = "getprop | grep ro.product.device"
        product_props = ShellUtils.get_props_from_cmd_output(get_product_cmd)
        return product_props["ro.product.device"]

    @staticmethod
    def set_system_time_slide_seconds(difference_in_seconds):
        instrumentation_args = ApiTestsGenericExtraArgs()
        args = instrumentation_args\
                .get_args_string(secondsAheadOfNow=difference_in_seconds)
        result = SystemApiTestsInterface.run_instrumentation(class_name="CommonTestsDriver",
                                                             method_name="testSetSystemTimeInSecondsFromNow",
                                                             instrumentation_args=args,
                                                             runner_name="GenericArgumentPassingTestRunner")
        time_set = InstrumentationInterface.instrumentation_one_test_pass_output in result
        if not time_set:
            LOG.info("problem setting system time" + str(result))
            return False
        return True

    @staticmethod
    def get_property(property_name, property_filter):
        LOG.info("property name: %s" % property_name + "; " + "property filter: %s" % property_filter)
        get_property_cmd = "getprop | grep " + property_filter
        property_hits = ShellUtils.get_props_from_cmd_output(get_property_cmd)
        LOG.info("property hits: " + str(property_hits))
        if property_name in property_hits:
            return property_hits[property_name]
        return None

    @staticmethod
    def get_screen_size_from_lcd_density_info(lcd_density_info):
        size_regex = r"\d+x\d+"
        size = re.findall(size_regex, lcd_density_info)[0]
        print size
        LOG.info("screen size info: " + str(size))
        return size.split('x')[0], size.split('x')[1]

    @staticmethod
    def wait_for_reminder_notification(reminder_text, wait_steps=10):
        for i in range(wait_steps):
            time.sleep(10)
            d.open.notification()
            if d(textContains=reminder_text).wait.exists(timeout=3000):
                if d(resourceId="com.android.systemui:id/dismiss_text").exists:
                    d(resourceId="com.android.systemui:id/dismiss_text").click()
                return True
            d.press.home()
        return False

    @staticmethod
    def set_system_time(moment_of_time):
        instrumentation_args = ApiTestsGenericExtraArgs()
        if moment_of_time.time_zone is not None:
            args = instrumentation_args\
                .get_args_string(minuteValue=moment_of_time.min,
                                 hourValue=moment_of_time.hour,
                                 dayValue=moment_of_time.day,
                                 monthValue=moment_of_time.month,
                                 yearValue=moment_of_time.year,
                                 timeZoneValue=moment_of_time.time_zone.replace(" ", "^"))
        else:
            args = instrumentation_args\
                .get_args_string(minuteValue=moment_of_time.min,
                                 hourValue=moment_of_time.hour,
                                 dayValue=moment_of_time.day,
                                 monthValue=moment_of_time.month,
                                 yearValue=moment_of_time.year)
        result = SystemApiTestsInterface.run_instrumentation(class_name="CommonTestsDriver",
                                                             method_name="testSetSystemTime",
                                                             instrumentation_args=args,
                                                             runner_name="GenericArgumentPassingTestRunner")
        time_set = InstrumentationInterface.instrumentation_one_test_pass_output in result
        if not time_set:
            LOG.info("problem setting system time" + str(result))
            return False
        return True

    @staticmethod
    def get_system_time():
        result = SystemApiTestsInterface.run_instrumentation(class_name="CommonTestsDriver",
                                                             method_name="testGetSystemTime",
                                                             instrumentation_args=None,
                                                             runner_name="GenericArgumentPassingTestRunner")
        time_acquired = InstrumentationInterface.instrumentation_one_test_pass_output in result
        if not time_acquired:
            LOG.info("problem acquiring time")
            return None
        logcat_msg = ApiTestsMessagingGateway()
        raw_time_info = logcat_msg.get_latest_message_with_id("TimeDateMessagingId").content_lines
        return MomentOfTime(30, raw_time_info[0], raw_time_info[1], raw_time_info[2], raw_time_info[3],
                            raw_time_info[4], raw_time_info[5])

    @staticmethod
    def get_hour_format():
        result = SystemApiTestsInterface.run_instrumentation(class_name="CommonTestsDriver",
                                                             method_name="testGetHourFormat",
                                                             instrumentation_args=None,
                                                             runner_name="GenericArgumentPassingTestRunner")
        time_acquired = InstrumentationInterface.instrumentation_one_test_pass_output in result
        if not time_acquired:
            LOG.info("problem acquiring hour format")
            return None
        logcat_msg = ApiTestsMessagingGateway()
        hour_format = logcat_msg.get_latest_message_with_id("hourFormat").get_content_as_on_line_string()
        return int(hour_format)

    @staticmethod
    def set_hour_format(hour_format):
        instrumentation_args = ApiTestsGenericExtraArgs()
        args = instrumentation_args.get_args_string(hourFormat=str(hour_format))
        result = SystemApiTestsInterface.run_instrumentation(class_name="CommonTestsDriver",
                                                             method_name="testSetHourFormat",
                                                             instrumentation_args=args,
                                                             runner_name="GenericArgumentPassingTestRunner")
        hour_format_set = InstrumentationInterface.instrumentation_one_test_pass_output in result
        if not hour_format_set:
            LOG.info("problem setting hour format" + str(result))
            return False
        return True

class ViewNavigator(object):

    def __init__(self, vertical_scroll_direction=True, scroll_times=10):
        self.vert_scroll = vertical_scroll_direction
        self.scroll_times = scroll_times

    resource_id_marker = ':id/'
    current_view_found = False
    def nagivate_text(self, text_labels):
        for label in text_labels:
            LOG.info("searching for text: " + label)
            if d(text=label).wait.exists(timeout=4000):
                time.sleep(1)
                d(text=label).click()
            elif d(textStartsWith=label).wait.exists(timeout=1000):
                d(textStartsWith=label).click()
            elif d(textContains=label).wait.exists(timeout=1000):
                d(textContains=label).click()
            else:
                self.search_text_in_scroll_view(label)
            time.sleep(1)
        time.sleep(2)

    def search_text_in_scroll_view(self, text):
        if d(scrollable=True).exists:
            self.scroll_to_beginning()
            for i in range(self.scroll_times):
                self.scroll_forward()
                d.wait.idle()
                UiAutomatorUtils.refresh_uiautomator_view_hierarchy()
                if d(text=text).wait.exists(timeout=3000):
                    d(text=text).click()
                    return True
                elif d(textStartsWith=text).wait.exists(timeout=1000):
                    d(textStartsWith=text).click()
                    return True
                elif d(textContains=text).wait.exists(timeout=1000):
                    d(textContains=text).click()
                    return True
        return False

    def scroll_to_beginning(self):
        if self.vert_scroll:
            d(scrollable=True).scroll.vert.toBeginning()
        else:
            d(scrollable=True).scroll.horiz.toBeginning()

    def scroll_forward(self):
        if self.vert_scroll:
            d(scrollable=True).scroll.vert.forward()
        else:
            d(scrollable=True).scroll.horiz.backward()

class ViewUtils():
    @staticmethod
    def view_generator_to_view_list(view_generator):
        view_list = []
        for view in view_generator:
            view_list.append(view)
        return view_list

SETTINGS_ACCOUNTS_OPTION_TXT = "Accounts"
SETTINGS_GOOGLE_ACCOUNT_OPTION_TXT = "Google"
SETTINGS_LOCATION_OPTION_TXT = "Location"
SETTINGS_LOCATION_MODE_TXT = "Mode"
SETTINGS_LOCATION_HIGH_ACCURACY_TXT = "High accuracy"
SETTINGS_SHOW_GPU_VIEW_UPDATES_TXT = "Show GPU view updates"
SETTINGS_SECURITY_TXT = "Security"
SETTINGS_SCREEN_LOCK_TXT = "Screen lock"
SETTINGS_SCREEN_LOCK_PIN_TXT = "PIN"
SETTINGS_SCREEN_LOCK_SWIPE_TXT = "Swipe"
SETTINGS_SCREEN_LOCK_NONE_TXT = "None"
SETTINGS_SECURE_STARTUP_DECLINE_TXT = "No thanks"
SETTINGS_SECURE_STARTUP_DECLINE_CONTINUE_TXT = "Continue"
SETTINGS_ENABLE_PIN_NOTIFICATIONS_TXT = "Show all notification content"
SETTINGS_ENABLE_PIN_DONE_TXT = "Done"
ADB_INPUT_TEXT_CMD = " input text "
TEST_PIN = "1234"
SETTINGS_SMART_LOCK_TXT = "Smart Lock"
SETTINGS_SMART_LOCK_GOTIT_TXT = "GOT IT"
SETTINGS_SMART_LOCK_TRUSTED_PLACES_TXT = "Trusted places"
SETTINGS_SMART_LOCK_ADD_TRUSTED_PLACE_TXT = "Add trusted place"
SETTINGS_SMART_LOCK_SELECT_CURRENT_LOCATION_TXT = "Select this location"
SETTINGS_TRUSTED_PLACE_NAME_RESID = "com.google.android.gms:id/trusted_place_name"
SETTINGS_TRUSTED_PLACES_SEARCH_RESID = "com.google.android.gms:id/places_ui_menu_main_search"
SETTINGS_TRUSTED_PLACE_INFO_RESID = "com.google.android.gms:id/info"
SETTINGS_TRUSTED_PLACE_DELETE_TXT = "Delete"
SETTINGS_TRUSTED_FIRST_TIME_RESID = "com.google.android.gms:id/auth_trust_agent_first_use_notification_button_id"
SETTINGS_TRUSTED_PLACE_SUGGESTION_PRIMARY_RESID = "com.google.android.gms:id/place_autocomplete_prediction_primary_text"
SETTINGS_REMOVE_DEVICE_PROTECTION_TXT = "Yes, remove"
SETTINGS_REMOVE_DEVICE_PROTECTION_TXT_N = "YES, REMOVE"

class Settings(object):

    @staticmethod
    def turn_on_account_auto_sync():
        g_common_obj.launch_app_am("com.android.settings", ".Settings")
        if not d(text="Accounts").exists:
            d(scrollable=True).scroll.vert.to(text="Accounts")
        d(text="Accounts").click.wait()
        d(text="Accounts").right(className="android.widget.ImageButton").click()
        if  d(resourceId="android:id/checkbox").info['checked']:
            LOG.info("Auto-sync data has already been turned on")
            d.press.home()
            return
        time.sleep(3)
        d(resourceId="android:id/checkbox").click()
        time.sleep(5)
        if d(textContains="auto-sync data on").exists:
            d(resourceId="android:id/button1").click()
            d.press.home()
        time.sleep(3)

    @staticmethod
    def enable_location():
        list_of_menu_options = [SETTINGS_LOCATION_OPTION_TXT]
        list_of_location_options = [SETTINGS_LOCATION_MODE_TXT, SETTINGS_LOCATION_HIGH_ACCURACY_TXT]
        Settings.navigate_settings_menu(list_of_menu_options)
        if d(text=SETTINGS_OFF_TXT).wait.exists(timeout=3000):
            d(text=SETTINGS_OFF_TXT).click()
        vn = ViewNavigator()
        vn.nagivate_text(list_of_location_options)

    @staticmethod
    def disable_location():
        list_of_menu_options = [SETTINGS_LOCATION_OPTION_TXT]
        Settings.navigate_settings_menu(list_of_menu_options)
        if d(text=SETTINGS_ON_TXT).wait.exists(timeout=3000):
            d(text=SETTINGS_ON_TXT).click()

    @staticmethod
    def enable_pin():
        list_of_menu_options = [SETTINGS_SECURITY_TXT,SETTINGS_SCREEN_LOCK_TXT, SETTINGS_SCREEN_LOCK_PIN_TXT,
                                SETTINGS_SECURE_STARTUP_DECLINE_TXT, SETTINGS_SECURE_STARTUP_DECLINE_CONTINUE_TXT]
        list_of_pin_options = [SETTINGS_ENABLE_PIN_NOTIFICATIONS_TXT, SETTINGS_ENABLE_PIN_DONE_TXT]
        Settings.navigate_settings_menu(list_of_menu_options)

        # Input the PIN via shell
        # The numpad is not visible in uiautomator
        for i in range(2):
            AdbUtils.input_text(TEST_PIN)
            time.sleep(1)
            d.press.enter()
            time.sleep(1)

        vn = ViewNavigator()
        vn.nagivate_text(list_of_pin_options)

    @staticmethod
    def disable_pin():
        list_of_menu_options = [SETTINGS_SECURITY_TXT, SETTINGS_SCREEN_LOCK_TXT]
        list_of_pin_options = [SETTINGS_SCREEN_LOCK_NONE_TXT, SETTINGS_REMOVE_DEVICE_PROTECTION_TXT]
        list_of_pin_options_n = [SETTINGS_SCREEN_LOCK_NONE_TXT, SETTINGS_REMOVE_DEVICE_PROTECTION_TXT_N]

        Settings.navigate_settings_menu(list_of_menu_options)
        if d(resourceId="android:id/summary").exists:
            LOG.info("PIN unlock has already been disabled.")
            return
        # Input the PIN via shell
        # The numpad is not visible in uiautomator
        AdbUtils.input_text(TEST_PIN)
        time.sleep(1)
        d.press.enter()
        time.sleep(1)

        vn = ViewNavigator()
        if EnvironmentUtils.get_android_version() == "N":
            vn.nagivate_text(list_of_pin_options_n)
        else:
            vn.nagivate_text(list_of_pin_options)

    @staticmethod
    def add_trusted_place(location="current"):
        list_of_menu_options = [SETTINGS_SECURITY_TXT, SETTINGS_SMART_LOCK_TXT]
        list_of_smart_lock_options = [SETTINGS_SMART_LOCK_TRUSTED_PLACES_TXT, SETTINGS_SMART_LOCK_ADD_TRUSTED_PLACE_TXT]

        Settings.navigate_settings_menu(list_of_menu_options)

        # Input the PIN via shell
        # The numpad is not visible in uiautomator
        AdbUtils.input_text(TEST_PIN)
        time.sleep(1)
        d.press.enter()
        time.sleep(3)
        if d(resourceId="com.google.android.gms:id/trust_agent_onboarding_got_it_button").exists:
            d(resourceId="com.google.android.gms:id/trust_agent_onboarding_got_it_button").click()

        vn = ViewNavigator()
        vn.nagivate_text(list_of_smart_lock_options)

        time.sleep(3)
        if location == "current":
            if d(resourceId="android:id/button1").exists:
                d(resourceId="android:id/button1").click()
            for i in range(5,11):
                if not d(resourceId="com.google.android.gms:id/marker_map_my_location_button").exists:
                    time.sleep(i+10)
                else:
                    d(resourceId="com.google.android.gms:id/marker_map_my_location_button").click()
                    time.sleep(5)
                    if not '31.023' in d(resourceId="com.google.android.gms:id/info").info['text']:
                        d(resourceId="com.google.android.gms:id/marker_map_my_location_button").click()
                        time.sleep(i)
                    else:
                        break
            time.sleep(10)
            view_to_click = d(text=SETTINGS_SMART_LOCK_SELECT_CURRENT_LOCATION_TXT)
#             assert view_to_click.wait.exists(timeout=10000), "The view with text={0} did not appear ".format(
#                     SETTINGS_SMART_LOCK_SELECT_CURRENT_LOCATION_TXT)
            t = 20
            while t > 0:
                if view_to_click.exists:
                    view_to_click.click()
                    break
                else:
                    t -= 1
                    time.sleep(10)
            trusted_place_name_view = d(resourceId=SETTINGS_TRUSTED_PLACE_NAME_RESID)
            assert trusted_place_name_view.wait.exists(timeout=10000), "The view with resid={0} did not appear".format(
                    SETTINGS_TRUSTED_PLACE_NAME_RESID)
            # trusted_place_name_view.clear_text()
            UiAutomatorUtils.clear_text_from_edit_text(SETTINGS_TRUSTED_PLACE_NAME_RESID)
            for _ in range(0,10):
                trusted_place_name_view.clear_text()
            trusted_place_name_view.set_text(location)
            UiAutomatorUtils.click_view_with_text(POPUP_OK)
        else:
            place_search = d(resourceId=SETTINGS_TRUSTED_PLACES_SEARCH_RESID)
            assert place_search.wait.exists(timeout=10000), "The view with resid={0} did not appear".format(
                    SETTINGS_TRUSTED_PLACES_SEARCH_RESID)
            place_search.click()
            for _ in range(0,10):
                d(text="Search").clear_text()
            d(text="Search").set_text(location)
            time.sleep(2)
            d.press.enter()
            time.sleep(5)
            # Wait for the first suggestion
            suggestion = d(resourceId=SETTINGS_TRUSTED_PLACE_SUGGESTION_PRIMARY_RESID, text=location.split(",")[0])
            if suggestion.wait.exists(timeout=10000):
                suggestion.click()
            time.sleep(5)
            view_to_click = d(resourceId=SETTINGS_TRUSTED_PLACE_INFO_RESID, text=location)
            assert view_to_click.wait.exists(timeout=10000), "The view with resid={0}, text={1} did not appear".format(
                    SETTINGS_TRUSTED_PLACE_INFO_RESID, location)
            view_to_click.click()
            UiAutomatorUtils.click_view_with_text(POPUP_OK)

    @staticmethod
    def remove_trusted_place(place_name="current"):
        list_of_menu_options = [SETTINGS_SECURITY_TXT, SETTINGS_SMART_LOCK_TXT]
        list_of_smart_lock_options = [SETTINGS_SMART_LOCK_TRUSTED_PLACES_TXT, place_name,
                                      SETTINGS_TRUSTED_PLACE_DELETE_TXT]

        Settings.navigate_settings_menu(list_of_menu_options)

        # Input the PIN via shell
        # The numpad is not visible in uiautomator
        AdbUtils.input_text(TEST_PIN)
        time.sleep(1)
        d.press.enter()
        time.sleep(5)

        vn = ViewNavigator()
        vn.nagivate_text(list_of_smart_lock_options)

    @staticmethod
    def launch():
        if d(packageName=SETTINGS_PACKAGE_NAME).exists:
            pass
        else:
#             UiAutomatorUtils.launch_app_from_apps_menu(SETTINGS_SHORTCUT_NAME)
            g_common_obj.launch_app_am("com.android.settings", "com.android.settings.Settings")

    @staticmethod
    def go_to_main_screen():
        navigate_tries = 0
        max_tries = 7
        while not d(description=SETTINGS_NAVIGATE_UP_DESC).wait.gone(timeout=3000):
            d(description=SETTINGS_NAVIGATE_UP_DESC).click()
            navigate_tries += 1
            if navigate_tries >= max_tries:
                return

    @staticmethod
    def open_data_usage():
        Settings.launch()
        if d(text=SETTINGS_DATA_USAGE_TXT).wait.exists(timeout=2000):
            time.sleep(1)
            d(text=SETTINGS_DATA_USAGE_TXT).click()
        if EnvironmentUtils.get_android_version() == "N":
            d(text="Wi-Fi data usage").click()

    @staticmethod
    def navigate_settings_menu(list_of_menu_options):
        Settings.launch()
        Settings.go_to_main_screen()
        view_navigator = ViewNavigator()
        view_navigator.nagivate_text(list_of_menu_options)

    @staticmethod
    def take_bug_report():
        g_common_obj.launch_app_am('com.android.settings', 'com.android.settings.DevelopmentSettings')
#         list_of_menu_options = [SETTINGS_DEVELOPER_OPTIONS_TXT, SETTINGS_TAKE_BUG_REPORT_OPTION_TXT]
        list_of_menu_options = [SETTINGS_TAKE_BUG_REPORT_OPTION_TXT]
        Settings.navigate_settings_menu(list_of_menu_options)

    @staticmethod
    def get_data_usage_info():
        Settings.open_data_usage()
        data_usage = []
        scroll_ended = False
        scroll_ended_marker = "unknown"
        d(scrollable=True).scroll.vert.toBeginning()
        while not scroll_ended:
            apps = []
            if d(resourceId=TITLE_ELEMENT_RESID).wait.exists(timeout=2000):
                apps = d(resourceId=TITLE_ELEMENT_RESID)
            for app in apps:
                if len(Info.get_text(app)) > 2:
                    data_usage.append(Info.get_text(app))
            if scroll_ended_marker == Info.get_text(apps[0]):
                scroll_ended = True
            scroll_ended_marker = Info.get_text(apps[0])
            d(scrollable=True).scroll.vert.forward()
        return list(set(data_usage))

    @staticmethod
    def get_data_usage_values():
        Settings.open_data_usage()
        data_usage = {}
        scroll_ended = False
        scroll_ended_marker = "unknown"
        d(scrollable=True).scroll.vert.toBeginning()
        while not scroll_ended:
            apps = []
            summaries = []
            if d(resourceId=TITLE_ELEMENT_RESID).wait.exists(timeout=2000):
                apps = d(resourceId=TITLE_ELEMENT_RESID)
            if d(resourceId=SUMMARY_ELEMENT_RESID).wait.exists(timeout=1000):
                summaries = d(resourceId=SUMMARY_ELEMENT_RESID)
            apps = ViewUtils.view_generator_to_view_list(apps)
            summaries = ViewUtils.view_generator_to_view_list(summaries)
            if len(apps) > len(summaries):
                apps = apps[1:]
            for i in range(min(len(apps), len(summaries))):
                data_usage[Info.get_text(apps[i])] = Info.get_text(summaries[i])
            if scroll_ended_marker == Info.get_text(apps[0]):
                scroll_ended = True
            scroll_ended_marker = Info.get_text(apps[0])
            d(scrollable=True).scroll.vert.forward()
        return data_usage

    @staticmethod
    def enable_wifi():
        Settings.launch()
        if d(text=SETTINGS_OPTION_WIFI_TEXT).wait.exists(timeout=2000):
            d(text=SETTINGS_OPTION_WIFI_TEXT)[0].click()
        if d(resourceId=SETTINGS_WIFI_SWITCH_RESID).wait.exists(timeout=2000):
            switch_text = d(resourceId=SETTINGS_WIFI_SWITCH_RESID).info["text"]
            if "Off" in switch_text:
                d(resourceId=SETTINGS_WIFI_SWITCH_RESID).click()
                time.sleep(1)
        d.press.back()

    @staticmethod
    def disable_wifi():
        Settings.launch()
        if d(text=SETTINGS_OPTION_WIFI_TEXT).wait.exists(timeout=2000):
            d(text=SETTINGS_OPTION_WIFI_TEXT)[0].click()
        if d(resourceId=SETTINGS_WIFI_SWITCH_RESID).wait.exists(timeout=2000):
            switch_text = d(resourceId=SETTINGS_WIFI_SWITCH_RESID).info["text"]
            if "On" in switch_text:
                d(resourceId=SETTINGS_WIFI_SWITCH_RESID).click()
                time.sleep(1)
        d.press.back()

    @staticmethod
    def disable_gpu_overdraw():
        list_of_menu_options = [SETTINGS_DEVELOPER_OPTIONS_TXT, SETTINGS_DEBUG_GPU_OVERDRAW_TXT,
                                SETTINGS_OFF_TXT]
        Settings.navigate_settings_menu(list_of_menu_options)

    @staticmethod
    def enable_gpu_show_updates():
        list_of_menu_options = [SETTINGS_DEVELOPER_OPTIONS_TXT, SETTINGS_SHOW_GPU_VIEW_UPDATES_TXT]
        Settings.navigate_settings_menu(list_of_menu_options)

    @staticmethod
    def enable_gpu_overdraw_show_overdraw_areas():
        list_of_menu_options = [SETTINGS_DEVELOPER_OPTIONS_TXT, SETTINGS_DEBUG_GPU_OVERDRAW_TXT,
                                SETTINGS_SHOW_OVERDRAW_AREAS_OPTION_TXT]
        Settings.navigate_settings_menu(list_of_menu_options)

    @staticmethod
    def enable_gpu_overdraw_show_deuteranomaly():
        list_of_menu_options = [SETTINGS_DEVELOPER_OPTIONS_TXT, SETTINGS_DEBUG_GPU_OVERDRAW_TXT,
                                SETTINGS_SHOW_AREAS_FOR_DEUTERANOMALY_OPTION_TXT]
        Settings.navigate_settings_menu(list_of_menu_options)

class ScreenSwiper:
    @staticmethod
    def swipe_up(horizontal_offset=0.5):
        width, height = UiAutomatorUtils.get_screen_dims()
        sx = width * horizontal_offset
        sy = height * 0.7
        ex = width * horizontal_offset
        ey = height * 0.3
        d.swipe(sx, sy, ex, ey)
        time.sleep(2)
    @staticmethod
    def swipe_down(horizontal_offset=0.5):
        width, height = UiAutomatorUtils.get_screen_dims()
        sx = width * horizontal_offset
        sy = height * 0.3
        ex = width * horizontal_offset
        ey = height * 0.7
        d.swipe(sx, sy, ex, ey)
        time.sleep(2)
    @staticmethod
    def swipe_left(horizontal_offset=0.5):
        width, height = UiAutomatorUtils.get_screen_dims()
        sx = width * 0.7
        sy = height * horizontal_offset
        ex = width * 0.3
        ey = height * horizontal_offset
        d.swipe(sx, sy, ex, ey)
        time.sleep(2)
    @staticmethod
    def swipe_right(horizontal_offset=0.5):
        width, height = UiAutomatorUtils.get_screen_dims()
        sx = width * 0.3
        sy = height * horizontal_offset
        ex = width * 0.7
        ey = height * horizontal_offset
        d.swipe(sx, sy, ex, ey)
        time.sleep(2)

class Gmail(object):

    @staticmethod
    def select_sync_account_now():
        if d(resourceId=GMAIL_SYNC_NOW_RESID).wait.exists(timeout=3000):
            d(resourceId=GMAIL_SYNC_NOW_RESID).click()
        if d(resourceId="com.google.android.gm:id/action_done").exists:
            d(resourceId="com.google.android.gm:id/action_done").click()
        time.sleep(90)

class SystemPopupsAndDialogs(object):
    @staticmethod
    def popup_ok():
        if d(text=POPUP_OK).wait.exists(timeout=3000):
            d(text=POPUP_OK).click()

    @staticmethod
    def popup_skip():
        if d(text=POPUP_SKIP).wait.exists(timeout=3000):
            d(text=POPUP_SKIP).click()

    @staticmethod
    def open_resource_with(app_name, just_once_priority=True):
        # there are different types of popup for different Android versions and devices
        if just_once_priority and d(resourceId="android:id/button_once").wait.exists(timeout=3000) and\
                UiAutomatorUtils.is_view_clickable(d(resourceId="android:id/button_once")):
            d(resourceId="android:id/button_once").click()
        elif d(text=app_name).wait.exists(timeout=1000):
            # view with app_name text is not clickable, so find it's layout
            UiAutomatorUtils.click_center_of_ui_object(d(text=app_name))
            time.sleep(2)
            if d(resourceId="android:id/button_once").wait.exists(timeout=3000):
                d(resourceId="android:id/button_once").click()

    @staticmethod
    def is_app_crash_popup_visible(wait_time=3000):
        if d(textContains=APP_CRASH_POPUP_BEGIN_TXT).wait.exists(timeout=wait_time) and\
                d(textContains=APP_CRASH_POPUP_END_TXT):
            return True
        return False

class Clock(object):
    @staticmethod
    def launch():
        if d(packageName=CLOCK_PACKAGE_NAME).exists:
            pass
        else:
#             UiAutomatorUtils.launch_app_from_apps_menu(CLOCK_SHORTCUT_NAME)
            g_common_obj.launch_app_am("com.google.android.deskclock", "com.android.deskclock.DeskClock")

    @staticmethod
    def go_to_alarms_activity():
        Clock.launch()
        if d(description=CLOCK_ALARM_DESC).wait.exists(timeout=3000):
            d(description=CLOCK_ALARM_DESC).click()

    @staticmethod
    def delete_all_alarms():
        Clock.launch()
        Clock.go_to_alarms_activity()
        while d(description=CLOCK_EXPAND_ALARM_DESC).wait.exists(timeout=3000):
            try:
                d(description=CLOCK_EXPAND_ALARM_DESC)[0].click()
                d(description=CLOCK_DELETE_ALARM_DESC).wait.exists(timeout=3000)
                d(description=CLOCK_DELETE_ALARM_DESC)[0].click()
            except:
                LOG.info("some error occured at alarm cleanup")
            time.sleep(2)

    @staticmethod
    def set_new_alarm(hour, minute, offset=0):
        hour = int(hour)
        minute = int(minute)
        alarm_time = datetime.datetime(1, 1, 1, hour, minute, 0) + datetime.timedelta(seconds=120 + offset)
        alarm_minute = alarm_time.minute
        delta_to_five_dividable_minute_value = (5 - alarm_minute % 5) % 5
        alarm_time += datetime.timedelta(seconds=60 * delta_to_five_dividable_minute_value)
        Clock.launch()
        hour = alarm_time.hour
        minute = alarm_time.minute
        if d(description=CLOCK_ALARM_DESC).wait.exists(timeout=3000):
            d(description=CLOCK_ALARM_DESC).click()
        if d(description=CLOCK_ADD_ALARM_BUTTON_DESC).wait.exists(timeout=3000):
            d(description=CLOCK_ADD_ALARM_BUTTON_DESC).click()
        if hour >= 12 and d(resourceId="android:id/pm_label").wait.exists(timeout=2000):
            d(resourceId="android:id/pm_label").click()
            if hour > 12:
                hour -= 12
        elif hour == 0 and d(resourceId="android:id/am_label").wait.exists(timeout=2000):
                d(resourceId="android:id/am_label").click()
                hour = 12
        if d(description=str(hour)).wait.exists(timeout=2000):
            d(description=str(hour)).click()
        if d(description=str(minute)).wait.exists(timeout=2000):
            d(description=str(minute)).click()
        if d(text="OK").wait.exists(timeout=3000):
            d(text="OK").click()
        return alarm_time

class Info(object):
    @staticmethod
    def get_text(ui_object):
        return str(ui_object.info["text"])

    @staticmethod
    def get_description(ui_object):
        return str(ui_object.info["contentDescription"])

    @staticmethod
    def get_clickable(ui_object):
        return ui_object.info["clickable"]

    @staticmethod
    def get_enabled(ui_object):
        return ui_object.info["enabled"]


POLLING_TIMEOUT = 15

class OrientationChanger():
    @staticmethod
    def change_orientation(new_orientation):
        if d.orientation == new_orientation:
            return
        d.orientation = new_orientation
        for i in range(POLLING_TIMEOUT):
            time.sleep(1)
            if d.orientation == new_orientation:
                break

ANDROID_WIDGET_EDIT_TEXT = "android.widget.EditText"
KEYCODE_A = 0x1d
META_ALT = 0x02

class UiAutomatorUtils:

    @staticmethod
    def launch_app_from_apps_menu(app_name):
        UiAutomatorUtils.clear_popups()
        d.screen.on()
        d.press.home()
        time.sleep(3)
        if d(description=APPS_BUTTON_DESC).exists:
            d(description=APPS_BUTTON_DESC).click()
        else:
            d(description="Apps list").click()
#         d(className='android.widget.TextView', index=3).click()
        if d(text=app_name).wait.exists(timeout=3000):
            d(text=app_name).click()
            return
        d(scrollable=True).scroll.vert.backward()
        d(scrollable=True).scroll.vert.toBeginning(max_swipes=MAX_SWIPES_TO_LIMIT)
        chrome = d(text=app_name)
        i = 1
        while not chrome.exists:
            if i == MAX_SWIPES_TO_LIMIT:
                break
            d(scrollable=True).scroll.vert.forward()
            chrome = d(text=app_name)
            i += 1
        chrome.click()

    @staticmethod
    def click_view_with_text(text, view_timeout=3000):
        view_found = d(text=text).wait.exists(timeout=view_timeout)
        if view_found:
            d(text=text).click()
        return view_found

    @staticmethod
    def launch_activity_with_data_uri(uri):
        cmd = 'am start -a android.intent.action.VIEW -d "$URI$"'
        cmd = cmd.replace("$URI$", uri)
        AdbUtils.run_adb_cmd(cmd)
        time.sleep(2)

    @staticmethod
    def close_all_tasks(sleep_after_close=1):
        d.press.home()
        d.press("recent")
        nr_of_dismissed_tasks = 0
        while d(resourceId=DISMISS_TASK_RESID).wait.exists(timeout=5000):
            d(resourceId=DISMISS_TASK_RESID).click()
            time.sleep(sleep_after_close)
            nr_of_dismissed_tasks += 1
        return nr_of_dismissed_tasks

    @staticmethod
    def clear_text_from_edit_text(edit_resit):
        if d(className=ANDROID_WIDGET_EDIT_TEXT, resourceId=edit_resit).wait.exists(timeout=5000):
            d.press(KEYCODE_A, META_ALT)
            d.press.delete()

    @staticmethod
    def get_screen_dims():
        return d.info["displayWidth"], d.info["displayHeight"]

    @staticmethod
    def refresh_uiautomator_view_hierarchy():
        d.dump()

    @staticmethod
    def click_center_of_ui_object(ui_object):
        bounds = Bounds(ui_object.info)
        center_x, center_y = bounds.get_center_coordinates()
        print center_x, center_y
        LOG.info("clicking coordinates: %s, %s" % (center_x, center_y))
        d.click(center_x, center_y)

    @staticmethod
    def is_view_clickable(ui_object):
        return Info.get_clickable(ui_object) and Info.get_enabled(ui_object)

    @staticmethod
    def reboot_device():
        initial_time = datetime.datetime.now()
        AdbUtils.kill_python_uiautomator_rpc_server_on_dut()
        time.sleep(5)
        AdbUtils.reboot_device()
        time.sleep(5)
        AdbUtils.wait_for_main_ui_services()
        time.sleep(5)
        LOG.info("trying to refresh uiautomator device")
        AdbUtils._run_adb_cmd("kill-server", adb_shell=False, add_ticks=False)
#         for i in xrange(50):
        try:
            AdbUtils._run_adb_cmd("root", adb_shell=False, add_ticks=False)
            time.sleep(5)
#                 DutManager().refresh_active_device()
        except:
            LOG.info("UI not yet available")
            time.sleep(5)
        final_time = datetime.datetime.now()
        time_delta = final_time - initial_time
        time.sleep(5)
        return time_delta.seconds

    @staticmethod
    def go_to_apps_beginning():
        UiAutomatorUtils.clear_popups()
        d.screen.on()
        d.press.home()
        d(description=APPS_BUTTON_DESC).click()
        if d(scrollable=True).wait.exists(timeout=3000):
            d(scrollable=True).scroll.horiz.backward()
            d(scrollable=True).scroll.horiz.toBeginning(max_swipes=MAX_SWIPES_TO_LIMIT)

    @staticmethod
    def clear_popups():
        if d(text=POPUP_OK).exists:
            d(text=POPUP_OK).click()

    @staticmethod
    def unlock_screen():
        lock_status_cmd = "dumpsys power | grep 'mHolding'"
        ''' We expect an output like this, if the screen is unlocked:
               mHoldingWakeLockSuspendBlocker=true
               mHoldingDisplaySuspendBlocker=true
        '''
        lock_status = AdbUtils._run_adb_cmd(lock_status_cmd, add_ticks=False)
        blockers_active = re.findall('true', lock_status)
        if len(blockers_active) == 2:
            # screen already unlocked
            LOG.info("screen already unlocked")
            return
        AdbUtils.run_adb_cmd(screen_unlock_cmd)
        time.sleep(1)
        AdbUtils.run_adb_cmd(screen_unlock_cmd)
        time.sleep(3)

class DeviceInfo(object):
    @staticmethod
    def get_device_screen_dimensions():
        device_info = d.info
        return device_info['displayWidth'], device_info['displayHeight']

class Bounds:
    def __init__(self, view_info):
        vis_bounds = view_info["visibleBounds"]
        self.top = vis_bounds["top"]
        self.bottom = vis_bounds["bottom"]
        self.left = vis_bounds["left"]
        self.right = vis_bounds["right"]

    def get_center_coordinates(self):
        x = self.left + (self.right - self.left) / 2
        y = self.top + (self.bottom - self.top) / 2
        return x, y

class InstrumentationInterface(object):
    instrumentation_one_test_pass_output = "OK (1 test)"
    shell_cmd_template = Template('''
            am instrument -e class $classarg $extraargs -w $runnerarg
            ''')

    @staticmethod
    def run_instrumentation(class_arg, extra_args, runner_arg):
        shell_cmd = InstrumentationInterface\
            .shell_cmd_template.substitute(classarg=class_arg, extraargs=extra_args,
                                           runnerarg=runner_arg)
        LOG.info("running instrumentation command: " + shell_cmd)
        # output = subprocess.check_output(shell_cmd, shell=True)
        output = AdbUtils.run_adb_cmd(shell_cmd)
        return output

    @staticmethod
    def was_instrumentation_test_successful(test_output):
        return InstrumentationInterface.instrumentation_one_test_pass_output in test_output

class SystemApiTestsInterface(InstrumentationInterface):
    class_template = Template('com.intel.test.systemapitests.tests.$classname#$method')
    args_template = Template('-e args "$args"')
    runner_template = Template('com.intel.test.systemapitests/com.intel.test.systemapitests.runners.$runner')

    @staticmethod
    def run_instrumentation(class_name="FileSystemTestsDriver", method_name="testCreateFile",
                            instrumentation_args=None, runner_name="GenericArgumentPassingTestRunner"):
        class_string = SystemApiTestsInterface\
            .class_template.substitute(classname=class_name, method=method_name)
        args_string = ""  # pass this to base class if there are no extra args
        if instrumentation_args is not None:
            args_string = SystemApiTestsInterface\
                .args_template.substitute(args=instrumentation_args)
        runner_string = SystemApiTestsInterface\
            .runner_template.substitute(runner=runner_name)
        output = InstrumentationInterface.run_instrumentation(class_string, args_string, runner_string)
        return output

class ApiTestsGenericExtraArgs(object):
    def __init__(self, **initial_args):
        self.args = {}
        for key, value in initial_args.iteritems():
            self.args[key] = value

    def get_args_string(self, **kwargs):
        result = ""
        for key, value in self.args.iteritems():
            result += str(key) + ":" + str(value) + " "
        for key, value in kwargs.iteritems():
            self.args[key] = value
            result += str(key) + ":" + str(value) + " "
        return result[:-1]

class ApiTestsInterface(InstrumentationInterface):
    class_template = Template('com.intel.test.apitests.tests.$classname#$method')
    args_template = Template('-e args "$args"')
    extra_args_template = Template('-e argsExtra "$args"')
    runner_template = Template('com.intel.test.apitests/com.intel.test.apitests.runners.$runner')

    @staticmethod
    def run_instrumentation(class_name="SystemStorageUSBTestsDriver", method_name="testCreateFile",
                            instrumentation_args=None, instrumentation_extra_args=None,
                            runner_name="GenericArgumentPassingTestRunner"):
        class_string = ApiTestsInterface\
            .class_template.substitute(classname=class_name, method=method_name)
        args_string = ""  # pass this to base class if there are no extra args
        if instrumentation_args is not None:
            args_string = ApiTestsInterface\
                .args_template.substitute(args=instrumentation_args)
        if instrumentation_extra_args is not None:
            args_string += " " + ApiTestsInterface\
                .extra_args_template.substitute(args=instrumentation_extra_args)
        runner_string = ApiTestsInterface\
            .runner_template.substitute(runner=runner_name)
        output = InstrumentationInterface.run_instrumentation(class_string, args_string, runner_string)
        return output

class Calendar(object):

    CALENDAR_RIGHT_ARROW_RESID = "com.google.android.calendar:id/right_arrow"
    CALENDAR_DONE_BUTTON_RESID = "com.google.android.calendar:id/done_button"
    CALENDAR_PACKAGE_NAME = "com.google.android.calendar"
    CALENDAR_SHORTCUT_NAME = "Calendar"
    CALENDAR_END_TIME_OPTION_RESID = "com.google.android.calendar:id/end_time"


    @staticmethod
    def get_current_dut_time():
        date = g_common_obj.adb_cmd_capture_msg("date")
        time_val = re.findall("(\d+):(\d+):(\d+)", date)
        hour = time_val[0][0]
        minute = time_val[0][1]
        second = time_val[0][2]
        return hour, minute, second

    def __init__(self):
        self.d = g_common_obj.get_device()
        self.CALENDAR_ADD_ITEM_RESID = "com.google.android.calendar:id/floating_action_button"
        self.CALENDAR_ADD_EVENT_OPTION_TXT = "Event"
        self.CALENDAR_START_TIME_OPTION_RESID = "com.google.android.calendar:id/start_time"
        self.CALENDAR_PM_LABEL_RESID = "android:id/pm_label"
        self.CALENDAR_AM_LABEL_RESID = "android:id/am_label"
        self.CALENDAR_OK_OPTION_TXT = "OK"
        self.CALENDAR_SAVE_EVENT_RESID = "com.google.android.calendar:id/save"

    def launch(self):
        if self.d(packageName="com.google.android.calendar").exists:
            pass
        else:
            g_common_obj.launch_app_am("com.google.android.calendar", "com.android.calendar.AllInOneCalendarActivity")
            for _ in range(0,5):
                if self.d(resourceId="com.google.android.calendar:id/texts_frame").exists:
                    self.d(resourceId="com.google.android.calendar:id/texts_frame").swipe.left()
            if self.d(resourceId="com.google.android.calendar:id/done_button").exists:
                self.d(resourceId="com.google.android.calendar:id/done_button").click()
            if self.d(resourceId="com.google.android.calendar:id/sync_off_notification_text").exists:
                self.d(resourceId="com.google.android.calendar:id/button_dismiss").click()
        time.sleep(3)

    @staticmethod
    def create_event_in_immediate_future():
        hh, mm, ss = Calendar.get_current_dut_time()
        hour = int(hh)
        minute = int(mm)
        alarm_time = datetime.datetime(1, 1, 1, hour, minute, 0) + datetime.timedelta(seconds=60)
        alarm_minute = alarm_time.minute
        delta_to_five_dividable_minute_value = (5 - alarm_minute % 5) % 5
        alarm_time += datetime.timedelta(seconds=60 * delta_to_five_dividable_minute_value)
        hour = alarm_time.hour
        minute = alarm_time.minute
        Calendar.create_event_at_specific_time_multiple_of_5_minutes(hour, minute)
        return alarm_time

    def create_event_at_specific_time_multiple_of_5_minutes(self, hour, minute):
        if self.d(resourceId=self.CALENDAR_ADD_ITEM_RESID).wait.exists(timeout=3000):
            self.d(resourceId=self.CALENDAR_ADD_ITEM_RESID).click()
        if self.d(textContains=self.CALENDAR_ADD_EVENT_OPTION_TXT).wait.exists(timeout=3000):
            self.d(textContains=self.CALENDAR_ADD_EVENT_OPTION_TXT).click()
        if self.d(resourceId=self.CALENDAR_START_TIME_OPTION_RESID).wait.exists(timeout=3000):
            self.d(resourceId=self.CALENDAR_START_TIME_OPTION_RESID).click()
        if hour >= 12 and self.d(resourceId=self.CALENDAR_PM_LABEL_RESID).wait.exists(timeout=3000):
            self.d(resourceId=self.CALENDAR_PM_LABEL_RESID).click()
            hour -= 12
        else:
            self.d(resourceId=self.CALENDAR_AM_LABEL_RESID).click()
        if self.d(description=str(hour)).wait.exists(timeout=2000):
            self.d(description=str(hour)).click()
        if self.d(description=str(minute)).wait.exists(timeout=2000):
            self.d(description=str(minute)).click()
        if self.d(text=self.CALENDAR_OK_OPTION_TXT).wait.exists(timeout=3000):
            self.d(text=self.CALENDAR_OK_OPTION_TXT).click()

    def save_event(self):
        if self.d(resourceId=self.CALENDAR_SAVE_EVENT_RESID).wait.exists(timeout=3000):
            self.d(resourceId=self.CALENDAR_SAVE_EVENT_RESID).click()
        time.sleep(2)

    @staticmethod
    def create_calendar_event(title, description, start_point_seconds, duration_seconds, reminder_minutes=1):
        LOG.info("adding event: %s" % title)
        instrumentation_args = ApiTestsGenericExtraArgs()
        args = instrumentation_args\
            .get_args_string(title=title,
                             description=description,
                             startPointSeconds=start_point_seconds,
                             durationSeconds=duration_seconds,
                             reminderValueMinutes=reminder_minutes)
        result = ApiTestsInterface.run_instrumentation(class_name="CalendarTestsDriver",
                                                       method_name="testInsertEventInCalendar",
                                                       instrumentation_args=args,
                                                       runner_name="GenericArgumentPassingTestRunner")
        return InstrumentationInterface.instrumentation_one_test_pass_output in result

class CommonUtils(object):

    @staticmethod
    def get_current_time_string():
        return datetime.datetime.strftime(datetime.datetime.now(), '%Y_%m_%d_%H_%M_%S')

class EnvironmentUtils(object):
    sd_card_uuid_regex = 'mmcblk1p1.*UUID="([^"]*)"'
    usb_uuid_regex = 'sda\d.*UUID="([^"]*)"'
    ANDROID_VERSION = ""

    @staticmethod
    def check_if_google_account_added():
        g_common_obj.launch_app_am('com.android.settings','com.android.settings.Settings')
        time.sleep(3)
        if not d(textContains="Accounts").exists:
            d(scrollable=True).scroll.vert.to(textContains="Accounts")
            d(textContains="Accounts").click.wait()
        if d(text="Google").exists:
            return True
        else:
            return False

    @staticmethod
    def get_android_version():
        """
        Get Android version:
        adb shell getprop | grep ro.build.version.sdk
        """
        cmd = 'getprop ro.build.version.sdk | grep -o [0-9]*'
        sdk_string = AdbUtils.run_adb_cmd(cmd).strip()
        LOG.info("Android version is: %s" % VERSION_HISTORY[sdk_string])
        return VERSION_HISTORY[sdk_string]

    @staticmethod
    def get_usb_storage_path():
        ANDROID_VERSION = EnvironmentUtils.get_android_version()
        if ANDROID_VERSION is "M" or ANDROID_VERSION is "L":
            return EnvironmentUtils.get_usb_storage_path_for_android_M()
        else:
            return None

    @staticmethod
    def get_sd_card_path():
        ANDROID_VERSION = EnvironmentUtils.get_android_version()
        try:
            if Environment.sd_card_path is not None:
                return Environment.sd_card_path
        except:
            LOG.info("proceding with sdcard initializing")
        if ANDROID_VERSION is "L":
            return "/storage/sdcard1/"
        elif ANDROID_VERSION is "M":
            sd_card_path = EnvironmentUtils.get_sd_card_path_for_android_m()
            LOG.info("Android M SDCARD path is: " + str(sd_card_path))
            return sd_card_path
        elif ANDROID_VERSION in ("N", "O-MR0", "O-MR1"):
            sd_card_path = EnvironmentUtils.get_sd_card_path_for_android_n()
            LOG.info("Android N SDCARD path is: " + str(sd_card_path))
            return sd_card_path
        else:
            LOG.info("Android version not recognized")
            return None

    @staticmethod
    def get_emulated_storage_path():
        ANDROID_VERSION = EnvironmentUtils.get_android_version()
        if ANDROID_VERSION is "L":
            return "/storage/sdcard0/"
        elif ANDROID_VERSION in ("M", "N", "O-MR0", "O-MR1"):
            return "/storage/emulated/0/"
        else:
            LOG.info("Android version not recognized")
            return None

    @staticmethod
    def quick_adb_check_sdcard_mounted():
        cmd = "ls /storage"
        out = AdbUtils.run_adb_cmd(cmd)
        if len(out.splitlines()) > 2 or "-" in out:
            return True
        return False

    @staticmethod
    def get_sd_card_uuid(force_blkid_search=False):
        try:
            if not EnvironmentUtils.quick_adb_check_sdcard_mounted() and not force_blkid_search:
                LOG.info("SDCARD is not mounted")
                return None
            block_device_attributes = AdbUtils.run_adb_cmd("blkid")
            sd_card_uuid = re.findall(EnvironmentUtils.sd_card_uuid_regex, block_device_attributes)[0]
            LOG.info("SD card UUID: " + sd_card_uuid)
            return sd_card_uuid
        except:
            LOG.info("Exception in getting sdcard UUID")
            return None

    @staticmethod
    def get_usb_uuids():
        try:
            block_device_attributes = AdbUtils.run_adb_cmd("blkid")
            usb_uuids = re.findall(EnvironmentUtils.usb_uuid_regex, block_device_attributes)
            LOG.info("USB Storage UUID: " + str(usb_uuids))
            return usb_uuids
        except:
            return None

    @staticmethod
    def get_sd_card_path_for_android_m():
        try:
            sd_card_path = "/storage/" + EnvironmentUtils.get_sd_card_uuid() + "/"
            return sd_card_path
        except:
            LOG.info("Exception in trying to find the SD card UUID")
            return None

    @staticmethod
    def get_sd_card_path_for_android_n():
        # it seems the sdcard path is the same for N and M
        return EnvironmentUtils.get_sd_card_path_for_android_m()

    @staticmethod
    def get_usb_storage_path_for_android_M():
        try:
            usb_uuids = EnvironmentUtils.get_usb_uuids()
            usb_path = "/storage/" + usb_uuids[0] + "/"
            return usb_path
        except:
            LOG.info("Can't find USB storage UUID")
            return None

    @staticmethod
    def get_emulated_mountpoint():
        ANDROID_VERSION = EnvironmentUtils.get_android_version()
        if ANDROID_VERSION is "L":
            return "/mnt/shell/emulated"
        elif ANDROID_VERSION in ("M", "N", "O-MR0", "O-MR1"):
            return "/storage/emulated"
        else:
            LOG.info("Android version is not recognized")

    @staticmethod
    def get_sdcard_folder_symlink():
        ANDROID_VERSION = EnvironmentUtils.get_android_version()
        if ANDROID_VERSION is "L":
            return "/storage/emulated/legacy"
        elif ANDROID_VERSION in ("M", "N", "O-MR0", "O-MR1"):
            return "/storage/self/primary"
        else:
            LOG.info("Android version is not recognized")

    @staticmethod
    def get_portable_sdcard_mountpoint():
        ANDROID_VERSION = EnvironmentUtils.get_android_version()
        if ANDROID_VERSION is "L":
            return "/mnt/media_rw/sdcard1"
        elif ANDROID_VERSION in ("M", "N", "O-MR0", "O-MR1"):
            sd_card_uuid = EnvironmentUtils.get_sd_card_uuid()
            return "/mnt/media_rw/" + sd_card_uuid
        else:
            LOG.info("Android version is not recognized")

    @staticmethod
    def is_sdcard_adopted():
        result = SystemApiTestsInterface\
            .run_instrumentation(class_name="storage.StorageManagerTestsDriver",
                                 method_name="testSDCardIsAdopted",
                                 instrumentation_args=None,
                                 runner_name="GenericArgumentPassingTestRunner")
        LOG.info(result)
        return SystemApiTestsInterface.instrumentation_one_test_pass_output in result

    @staticmethod
    def is_sdcard_portable():
        result = SystemApiTestsInterface\
            .run_instrumentation(class_name="storage.StorageManagerTestsDriver",
                                 method_name="testSDCardIsMounted",
                                 instrumentation_args=None,
                                 runner_name="GenericArgumentPassingTestRunner")
        LOG.info(result)
        return SystemApiTestsInterface.instrumentation_one_test_pass_output in result

    @staticmethod
    def get_gvfs_folder_path():
        # gvfs_mount_info should be something like:
        # "gvfsd-fuse on /run/user/1000/gvfs type fuse.gvfsd-fuse (rw,nosuid,nodev,user=bogdan)"
        gvfs_mount_info = ShellUtils.run_shell_cmd('mount | grep gvfs')
        gvfs_mounts_folder = os.path.normpath(gvfs_mount_info.split()[2])
        return gvfs_mounts_folder

    @staticmethod
    def get_dut_MTP_host_path():
        '''
        # Legacy code
        systemd_user_folder_var_name = "XDG_RUNTIME_DIR"
        if systemd_user_folder_var_name not in os.environ:
            systemd_user_folder = "/run/user/1000"
        else:
            systemd_user_folder = os.environ[systemd_user_folder_var_name]
        mtp_mounts_folder = os.path.join(systemd_user_folder, "gvfs")
        '''
        mtp_mounts_folder = EnvironmentUtils.get_gvfs_folder_path()
        dut_mtp_folder_candidates = [folder for folder in os.listdir(mtp_mounts_folder) if "mtp:" in folder]
        if len(dut_mtp_folder_candidates) == 0:
            return None
        else:
            return os.path.join(mtp_mounts_folder, dut_mtp_folder_candidates[0])

    @staticmethod
    def get_dut_PTP_host_path():
        ptp_mounts_folder = EnvironmentUtils.get_gvfs_folder_path()
        dut_ptp_folder_candidates = [folder for folder in os.listdir(ptp_mounts_folder) if "host" in folder and
                                     "mtp:" not in folder]
        if len(dut_ptp_folder_candidates) == 0:
            return None
        else:
            return os.path.join(ptp_mounts_folder, dut_ptp_folder_candidates[0])

    @staticmethod
    def get_dut_MTP_internal_storage_host_path():
        ANDROID_VERSION = EnvironmentUtils.get_android_version()
        host_mtp_path = EnvironmentUtils.get_dut_MTP_host_path()
        if host_mtp_path is None:
            return None
        dut_storage_mtp_host_paths = os.listdir(host_mtp_path)
        if ANDROID_VERSION is "M" or ANDROID_VERSION is "L":
            for path in dut_storage_mtp_host_paths:
                if "internal" in path.lower():
                    return os.path.join(host_mtp_path, os.path.normpath(path))
        else:
            LOG.info("Android version is not recognized")

    @staticmethod
    def get_dut_MTP_sd_card_host_path():
        ANDROID_VERSION = EnvironmentUtils.get_android_version()
        host_mtp_path = EnvironmentUtils.get_dut_MTP_host_path()
        if host_mtp_path is None:
            return None
        dut_storage_mtp_host_paths = os.listdir(host_mtp_path)
        if ANDROID_VERSION is "M" or ANDROID_VERSION is "L":
            for path in dut_storage_mtp_host_paths:
                if "sd" in path.lower() and "card" in path.lower():
                    return os.path.join(host_mtp_path, os.path.normpath(path))
        else:
            LOG.info("Android version is not recognized")

    @staticmethod
    def get_dut_MTP_adopted_host_path():
        return EnvironmentUtils.get_dut_MTP_sd_card_host_path()

class Environment(object):

    api_tests_data_cache_dir = "/data/data/com.intel.test.apitests/cache/"
    check_permissions_external_cache_dir = "Android/data/com.intel.test.checkpermissions/cache/"
    api_tests_data_cache_dir_sdcard = "Android/data/com.intel.test.apitests/cache/"
    context_tests_data_cache_dir = "Android/data/com.intel.test.apitests/cache"
    context_tests_data_external_dir = "Android/data/com.intel.test.apitests/files/Documents"
    context_tests_obb_dir = "Android/obb/com.intel.test.apitests"
    file_not_found_cmd_output = "No such file or directory"
    sd_card_path = EnvironmentUtils.get_sd_card_path()
    emulated_storage_path = EnvironmentUtils.get_emulated_storage_path()
    dcim_folder_path = emulated_storage_path + "DCIM/"
    tmp_dir_path = path.join(path.dirname(path.dirname(path.abspath(__file__))), "tmp")
    internal_mountpoint = "/data"
    internal_fstype = "ext4"
    fuse_fstype = "fuse"
    fat_fstype = "vfat"
    mtp_internal_storage_path = None
    mtp_sdcard_path = None
    mtp_adopted_path = None
    ptp_path = None
    dut_tmp_dir = "/data/local/tmp/"
    storage_emulated_root = "/storage/emulated"

    @staticmethod
    def initialize_mtp_paths():
        Environment.mtp_internal_storage_path = EnvironmentUtils.get_dut_MTP_internal_storage_host_path()
        LOG.info("MTP internal storage path: " + str(Environment.mtp_internal_storage_path))
        Environment.mtp_sdcard_path = EnvironmentUtils.get_dut_MTP_sd_card_host_path()
        LOG.info("MTP sdcard host path: " + str(Environment.mtp_sdcard_path))
        Environment.mtp_adopted_path = EnvironmentUtils.get_dut_MTP_adopted_host_path()
        LOG.info("MTP adopted host path: " + str(Environment.mtp_adopted_path))

    @staticmethod
    def initialize_ptp_paths():
        Environment.ptp_path = EnvironmentUtils.get_dut_PTP_host_path()

    @staticmethod
    def clean_tmp_dir():
        ShellUtils.clean_local_dir(Environment.tmp_dir_path)

    @staticmethod
    # in case sdcard is adopted or made portable
    def refresh_paths_after_sdcard_operations():
        Environment.sd_card_path = EnvironmentUtils.get_sd_card_path()

### Notifications
NOTIFICATIONS_MAIN_COLUMN_RESID = "android:id/notification_main_column"
NOTIFICATIONS_CLEAR_ALL_RESID = "com.android.systemui:id/dismiss_text"
NOTIFICATIONS_NOTIFICATION_TITLE_RESID = "android:id/title"
NOTIFICATIONS_USE_DEVICE_LOCATION_TXT = "wants to use your device's location"
NOTIFICATIONS_ALLOW_LOCATION_TXT = "Allow"
NOTIFICATIONS_BLOCK_LOCATION_TXT = "Block"
NOTIFICATIONS_NOTIFICATION_TEXT_RESID = "android:id/text"
NOTIFICATIONS_AIRPLANE_ON_DESC = "Airplane mode on."
NOTIFICATIONS_AIRPLANE_OFF_DESC = "Airplane mode off."
NOTIFICATIONS_BATTERY_RESID = "com.android.systemui:id/battery"
DIALER_GLOW_PAD_VIEW_RESID = "com.android.dialer:id/glow_pad_view"
DIALER_MISSED_CALL_TXT = "Missed call"
DIALER_HANG_UP_TXT = "Hang up"
DIALER_ANSWER_TXT = "Answer"
DIALER_DISMISS_TXT = "Dismiss"

class StatusBar(object):
    @staticmethod
    def open_notifications(nr_of_swipes=1):
        width, height = UiAutomatorUtils.get_screen_dims()
        sx = width / 2
        sy = height / 100
        ex = sx
        ey = int(height * 0.6)
        for i in range(nr_of_swipes):
            d.swipe(sx, sy, ex, ey)
#             time.sleep(0.5)
#         time.sleep(2)

    @staticmethod
    def close_notifications():
        # notifications are at most 2 back presses away from closing
        if not d(resourceId=NOTIFICATIONS_BATTERY_RESID).wait.exists(timeout=10000):
            LOG.info('Failed to find %s' % NOTIFICATIONS_BATTERY_RESID)
            return False
        d.press.back()
        if not d(resourceId=NOTIFICATIONS_BATTERY_RESID).wait.gone(timeout=5000):
            d.press.back()
        return d(resourceId=NOTIFICATIONS_BATTERY_RESID).wait.gone(timeout=5000)

    @staticmethod
    def get_nr_of_notifications():
        if d(resourceId=NOTIFICATIONS_MAIN_COLUMN_RESID).wait.exists(timeout=2000):
            return d(resourceId=NOTIFICATIONS_MAIN_COLUMN_RESID).count
        elif d(resourceId=NOTIFICATIONS_NOTIFICATION_TITLE_RESID).wait.exists(timeout=2000):
            return d(resourceId=NOTIFICATIONS_NOTIFICATION_TITLE_RESID).count
        else:
            return 0

    @staticmethod
    def click_on_first_status_element():
        if d(resourceId=NOTIFICATIONS_MAIN_COLUMN_RESID).wait.exists(timeout=2000):
            return d(resourceId=NOTIFICATIONS_MAIN_COLUMN_RESID).click()
        elif d(resourceId=NOTIFICATIONS_NOTIFICATION_TITLE_RESID).wait.exists(timeout=2000):
            return d(resourceId=NOTIFICATIONS_NOTIFICATION_TITLE_RESID).click()
        else:
            return 0

    @staticmethod
    def get_first_status_element():
        if d(resourceId=NOTIFICATIONS_MAIN_COLUMN_RESID).wait.exists(timeout=3000):
            return d(resourceId=NOTIFICATIONS_MAIN_COLUMN_RESID)[0].text
        elif d(resourceId=NOTIFICATIONS_NOTIFICATION_TITLE_RESID).wait.exists(timeout=2000):
            return d(resourceId=NOTIFICATIONS_NOTIFICATION_TITLE_RESID)[0].text
        else:
            return 0

    @staticmethod
    def clear_notifications():
        if d(resourceId=NOTIFICATIONS_CLEAR_ALL_RESID).wait.exists(timeout=2000):
            d(resourceId=NOTIFICATIONS_CLEAR_ALL_RESID).click()

    @staticmethod
    def hang_phone():
        if d(text=DIALER_HANG_UP_TXT).wait.exists(timeout=5000):
            d(text=DIALER_HANG_UP_TXT).click()

    @staticmethod
    def answer_phone():
        if not d(text=DIALER_ANSWER_TXT).wait.exists(timeout=30000):
            LOG.info('Failed to find %s' % DIALER_ANSWER_TXT)
            return False
        time.sleep(1)  # Wait for the UI to sync
        d(text=DIALER_ANSWER_TXT).click()
        if not d(text=DIALER_ANSWER_TXT).wait.gone(timeout=10000):
            LOG.info('Failed to answer.')
            return False
        return True

    @staticmethod
    def dismiss_phone():
        if not d(text=DIALER_DISMISS_TXT).wait.exists(timeout=30000):
            LOG.info('Failed to find %s' % DIALER_DISMISS_TXT)
            return False
        time.sleep(2)
        d(text=DIALER_DISMISS_TXT).click()
        if not d(text=DIALER_DISMISS_TXT).wait.gone(timeout=10000):
            LOG.info('Failed to dismiss.')
            return False
        return True

    @staticmethod
    def toggle_airplane_mode(mode):
        """
        Toggle airplane mode ON/OFF
        :param mode: ON/OFF
        :return: True if success else None
        """
        # TODO: this should be moved outside this function when more
        #       toggles are added to it
        notifications_toggles = {'Airplane mode': {'ON': NOTIFICATIONS_AIRPLANE_ON_DESC,
                                                   'OFF': NOTIFICATIONS_AIRPLANE_OFF_DESC}}

        def negate_mode(mode):
            return 'ON' if mode == 'OFF' else 'OFF'

        if mode not in notifications_toggles['Airplane mode'].keys():
            LOG.info("'%s' not a valid option (ON/OFF)." % mode)
            return None
        if mode == StatusBar.get_airplane_mode():
            LOG.info("Already on mode %s." % mode)
            return True
        else:
            d(description=notifications_toggles['Airplane mode'][negate_mode(mode)]).click.wait()
        # Check if toggle switched states
        return True if mode == StatusBar.get_airplane_mode() else None

    @staticmethod
    def get_airplane_mode():
        """
        Get airplane mode status
        :return: ON/OFF
        """
        if d(description=NOTIFICATIONS_AIRPLANE_ON_DESC).wait.exists(timeout=3000):
            return 'ON'
        elif d(description=NOTIFICATIONS_AIRPLANE_OFF_DESC).wait.exists(timeout=3000):
            return 'OFF'
        else:
            LOG.info('Failed to determine Airplane mode.')
            return None
