from PyUiApi.common.shell_utils import *
from PyUiApi.common.test_utils import *
from PyUiApi.adb_helper.instrumentation_utils import *
from PyUiApi.adb_helper.logcat_messaging_gateway import *
from PyUiApi.common.status_bar import *
from PyUiApi.app_utils.settings_utils import Settings
import re


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
    def get_property(property_name, property_filter):
        LOG.info("property name: " + property_name + " ; property filter: " + property_filter)
        get_property_cmd = "getprop | grep " + property_filter
        property_hits = ShellUtils.get_props_from_cmd_output(get_property_cmd)
        LOG.info("property hits: " + str(property_hits))
        if property_name in property_hits:
            return property_hits[property_name]
        return None

    @staticmethod
    def get_screen_size_from_lcd_density_info(lcd_density_info):
        size_regex = r"(\d+) x (\d+)"
        size = re.findall(size_regex, lcd_density_info)
        LOG.info("screen size info: " + str(size))
        return size[0][0], size[0][1]

    @staticmethod
    def open_homescreen_settings():
        d.press.home()
        UiAutomatorUtils.long_click(d(resourceId=ANDROID_WORKSPACE_RESID), press_time_in_seconds=3)

    @staticmethod
    def is_internet_connection_active():
        result = ApiTestsInterface\
            .run_instrumentation(class_name="WifiTestsDriver",
                                 method_name="testInternetConnectionActive",
                                 instrumentation_args=None,
                                 runner_name="GenericArgumentPassingTestRunner")
        internet_connection_active = InstrumentationInterface.instrumentation_one_test_pass_output in result
        if not internet_connection_active:
            LOG.info("no active internet connection on device")
            return False
        return True

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
    def wait_for_reminder_notification(reminder_text, wait_steps=10):
        for i in range(wait_steps):
            time.sleep(10)
            StatusBar.open_notifications()
            if d(textContains=reminder_text).wait.exists(timeout=3000):
                StatusBar.clear_notifications()
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
    def is_screen_on():
        get_display_state_cmd = "dumpsys power | grep 'Display Power'"
        state_string = AdbUtils.run_adb_cmd(get_display_state_cmd, add_ticks=False)
        if "ON" in state_string:
            return True
        elif "OFF" in state_string:
            return False
        else:
            return None


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
        if just_once_priority and d(text=OS_OPEN_JUST_ONCE_TXT).wait.exists(timeout=3000) and\
                UiAutomatorUtils.is_view_clickable(d(text=OS_OPEN_JUST_ONCE_TXT)):
            d(text=OS_OPEN_JUST_ONCE_TXT).click()
        elif d(text=app_name).wait.exists(timeout=1000):
            # view with app_name text is not clickable, so find it's layout
            UiAutomatorUtils.click_center_of_ui_object(d(text=app_name))
            time.sleep(2)
            if d(text=OS_OPEN_JUST_ONCE_TXT).wait.exists(timeout=3000):
                d(text=OS_OPEN_JUST_ONCE_TXT).click()

    @staticmethod
    def is_app_crash_popup_visible(wait_time=3000):
        if d(textContains=APP_CRASH_POPUP_BEGIN_TXT).wait.exists(timeout=wait_time) and\
                d(textContains=APP_CRASH_POPUP_END_TXT):
            return True
        return False


class ClipboardManager(object):

    @staticmethod
    def set_clipboard_text(text, label=None):
        instrumentation_args = ApiTestsGenericExtraArgs()
        test_args = instrumentation_args\
            .get_args_string(clipLabel=str(label),
                             clipText=str(text))
        result = ApiTestsInterface\
            .run_instrumentation(class_name="ClipboardManagerTestsDriver",
                                 method_name="setClipboardText",
                                 instrumentation_args=test_args,
                                 runner_name="GenericArgumentPassingTestRunner")
        LOG.info("set_clipboard_text result: " + result)

    @staticmethod
    def get_clipboard_text(message_id=None):
        result = ApiTestsInterface\
            .run_instrumentation(class_name="ClipboardManagerTestsDriver",
                                 method_name="testGetClipboardText",
                                 instrumentation_args=None,
                                 runner_name="GenericArgumentPassingTestRunner")
        LOG.info("set_clipboard_text result: " + result)
        api_tests_messages = ApiTestsMessagingGateway()
        if message_id is None:
            clip_message = api_tests_messages.get_latest_message()
            return clip_message.get_content_as_string()
        else:
            clip_message = api_tests_messages.get_latest_message_with_id(message_id)
            return clip_message.get_content_as_string()


class USBChooser(object):
    usb_chooser_activity_string = "com.android.settings/.deviceinfo.UsbModeChooserActivity"

    @staticmethod
    def open_usb_chooser_popup():
        for _ in range(10):
            AdbUtils.run_adb_cmd_ext("am start -S -n %s" % USBChooser.usb_chooser_activity_string)
            ui_exist = True
            ui_exist &= d(packageName=SETTINGS_PACKAGE_NAME).wait.exists(timeout=10000)
            ui_exist &= d(textContains=SETTINGS_USB_CHOOSER_TXT).wait.exists(timeout=10000)
            ui_exist &= d(className='android.widget.CheckedTextView').wait.exists(timeout=10000)
            if ui_exist == True:
                break
        return ui_exist

    @staticmethod
    def get_usb_option_index(option_text):
        idx = None
        USBChooser.open_usb_chooser_popup()
        for i, j in enumerate([item.text for item in d(className='android.widget.CheckedTextView')]):
            if any(x.lower() in j.lower() for x in option_text.split()):
                idx = i
                break
        assert idx is not None, 'Not found item %s' % (option_text)
        return idx

    @staticmethod
    def select_usb_option(opt_idx, retry=10, refresh=False):
        successful = False

        def selected_option():
            USBChooser.open_usb_chooser_popup()
            selected = d(className='android.widget.CheckedTextView', checked='True').text
            d.press.back()
            return selected

        USBChooser.open_usb_chooser_popup()
        target_option = d(className='android.widget.CheckedTextView', instance=opt_idx).text
        if not refresh and selected_option() == target_option:
            LOG.info("usb option already selected : " + str(target_option))
            return True

        i = 0
        while i < retry and not successful:
            i += 1
            try:
                USBChooser.open_usb_chooser_popup()

                if d(className='android.widget.CheckedTextView', instance=opt_idx).wait.exists(timeout=3000):
                    d(className='android.widget.CheckedTextView', instance=opt_idx).click.wait(timeout=60000)
            except Exception, _:
                pass

            if selected_option() == target_option:
                LOG.info("selected usb option: " + str(target_option))
                successful = True
                break
            time.sleep(2)
        time.sleep(5)

        return successful

    @staticmethod
    def select_mtp_option(refresh=True):
        LOG.info("selecting usb option: " + str(SETTINGS_USB_CHOOSER_MTP_OPTION_TXT))
        opt_idx = USBChooser.get_usb_option_index(SETTINGS_USB_CHOOSER_MTP_OPTION_TXT)
        return USBChooser.select_usb_option(opt_idx, refresh=refresh)

    @staticmethod
    def select_ptp_option(refresh=True):
        LOG.info("selecting usb option: " + str(SETTINGS_USB_CHOOSER_PTP_OPTION_TXT))
        opt_idx = USBChooser.get_usb_option_index(SETTINGS_USB_CHOOSER_PTP_OPTION_TXT)
        return USBChooser.select_usb_option(opt_idx, refresh=refresh)

    @staticmethod
    def select_default_option(refresh=True):
        LOG.info("selecting usb option: default")
        opt_idx = 0
        return USBChooser.select_usb_option(opt_idx, refresh=refresh)


class SystemUserUtils(object):

    @staticmethod
    def get_system_users():
        users = []
        users_string = AdbUtils.run_adb_cmd("pm list users")
        users_info = re.findall("UserInfo\{(.+)\}", users_string)
        for info in users_info:
            info_fields = info.split(":")
            users.append(SystemUser(info_fields[1], info_fields[0], info_fields[2]))
        return users

    @staticmethod
    def create_user(user_name):
        AdbUtils.run_adb_cmd("pm create-user " + str(user_name))

    @staticmethod
    def get_user_by_name(user_name):
        users = SystemUserUtils.get_system_users()
        for user in users:
            if user.name == user_name:
                return user
        return None

    @staticmethod
    def delete_user(user_id=None, user_name=None):
        if user_id is not None:
            AdbUtils.run_adb_cmd("pm remove-user " + str(user_id))
            return
        elif user_name is not None:
            user = SystemUserUtils.get_user_by_name(user_name)
            if user is not None:
                AdbUtils.run_adb_cmd("pm remove-user " + str(user.id))
                return
        LOG.info("could not delete user id: %s name: %s" % (str(user_id), str(user_name)))


class SystemUser(object):

    def __init__(self, user_name, user_id, flags):
        self.name = user_name
        self.id = user_id
        self.flags = flags

    def __str__(self):
        return "system user - name: %s, id: %s, flags: %s" % (self.name, self.id, self.flags)


class AccountAdder(object):
    add_account_activity_string = "com.android.settings/com.android.settings.accounts.AddAccountSettings"

    def start_add_account(self):
        AdbUtils.start_activity_from_shell(self.add_account_activity_string)
        if not d(text="Google", packageName=SETTINGS_PACKAGE_NAME).wait.exists(timeout=10000):
            LOG.info("Could not find Add Google Account option")
            return None
        d(text="Google", packageName=SETTINGS_PACKAGE_NAME).click()
        return self

    def enter_email(self, email):
        LOG.info("entering email", self)
        if not d(description=ADD_ACCOUNT_EMAIL_DESC).wait.exists(timeout=30000):
            LOG.info("Could not find Add Account email option")
            return None
        d(description=ADD_ACCOUNT_EMAIL_DESC).set_text(email)
        d.press.enter()
        if not d(description=ADD_ACCOUNT_NEXT_DESC).wait.exists(timeout=5000):
            LOG.info("Could not find Add Account next option")
            return None
        UiAutomatorUtils.wait_for_ui_object_to_be_enabled({"description": ADD_ACCOUNT_NEXT_DESC})
        d(description=ADD_ACCOUNT_NEXT_DESC).click()
        return self

    def enter_pass(self, password):
        LOG.info("entering pass", self)
        if not d(resourceId=ADD_ACCOUNT_PASS_RESID).wait.exists(timeout=30000):
            LOG.info("Could not find Add Account password option")
            return None
        d(resourceId=ADD_ACCOUNT_PASS_RESID).set_text(password)
        d.press.enter()
        if not d(description=ADD_ACCOUNT_NEXT_DESC).wait.exists(timeout=5000):
            LOG.info("Could not find Add Account next option")
            return None
        if d(description=ADD_ACCOUNT_NEXT_DESC).exists:
            UiAutomatorUtils.wait_for_ui_object_to_be_enabled({"description": ADD_ACCOUNT_NEXT_DESC})
            d(description=ADD_ACCOUNT_NEXT_DESC).click()
        return self

    def accept_terms(self):
        LOG.info("accepting terms", self)
        if not d(description=ADD_ACCOUNT_ACCEPT_DESC).wait.exists(timeout=20000):
            LOG.info("Could not find Accept Terms next option")
            return None
        time.sleep(2)
        UiAutomatorUtils.wait_for_ui_object_to_be_enabled({"description": ADD_ACCOUNT_ACCEPT_DESC})
        d(description=ADD_ACCOUNT_ACCEPT_DESC).click()
        return self

    def agree_backup(self):
        LOG.info("agreeing backup", self)
        if not d(resourceId=ADD_ACCOUNT_AGREE_BACKUP_RESID).wait.exists(timeout=30000):
            LOG.info("Could not find Add Account agree backup option")
            return None
        d(resourceId=ADD_ACCOUNT_AGREE_BACKUP_RESID).click()
        if not d(text=ADD_ACCOUNT_NEXT_DESC).wait.exists(timeout=5000):
            LOG.info("Could not find Add Account next option")
            return None
        if d(text=ADD_ACCOUNT_NEXT_DESC).exists:
            UiAutomatorUtils.wait_for_ui_object_to_be_enabled({"text": ADD_ACCOUNT_NEXT_DESC})
        d(text=ADD_ACCOUNT_NEXT_DESC).click()
        return self

    @staticmethod
    def is_google_account_active():
        instrumentation_args = ApiTestsGenericExtraArgs(type="com.google")
        test_args = instrumentation_args.get_args_string()
        result = SystemApiTestsInterface\
            .run_instrumentation(class_name="SystemTestsDriver",
                                 method_name="testIsAccountAvailable",
                                 instrumentation_args=test_args,
                                 runner_name="GenericArgumentPassingTestRunner")
        LOG.info(result)
        return SystemApiTestsInterface.was_instrumentation_test_successful(result)

    def remove_current_google_account(self):
        if AccountAdder.is_google_account_active():
            LOG.info("google account is active, proceding with removal", self)
            account_removed = Settings.remove_google_account()
            LOG.info("account removed status: " + str(account_removed))
            return account_removed
        else:
            LOG.info("google account is NOT active", self)
            return True