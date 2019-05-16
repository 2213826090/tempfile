from PyUiApi.common.uiautomator_utils import *
from PyUiApi.common.uiautomator_extension import UiAutomatorExtended
from PyUiApi.adb_helper.instrumentation_utils import ApiTestsInterface

# Define the device test PIN
TEST_PIN = "1234"

# Define new APN texts
TEST_APN_NAME = "0"
TEST_APN = "Broken_APN"

# Define APN default setting
DEFAULT_APN_NAME = ""


class Settings(object):

    vn = ViewNavigator()
    package_name = 'com.android.settings/com.android.settings.Settings'

    @staticmethod
    def launch():
        if d(packageName=SETTINGS_PACKAGE_NAME).exists:
            pass
        else:
            # UiAutomatorUtils.launch_app_from_apps_menu(SETTINGS_SHORTCUT_NAME)
            AdbUtils.run_adb_cmd("am start -S -n %s" % (Settings.package_name))

    @staticmethod
    def enable_screen_lock(lock_type):
        Settings.launch()
        vn = ViewNavigator()
        vn.nagivate_text(["Security", "Screen lock", lock_type])
        d.press.back()
        time.sleep(2)

    @staticmethod
    def enable_wifi():
        ''' UI deprecated code
        d.press.home()
        Settings.launch()
        if d(resourceId=SETTINGS_OPTION_RESID).wait.exists(timeout=2000):
            d(resourceId=SETTINGS_OPTION_RESID)[0].click()
        if d(resourceId=SETTINGS_WIFI_SWITCH_RESID).wait.exists(timeout=2000):
            switch_text = d(resourceId=SETTINGS_WIFI_SWITCH_RESID).info["text"]
            if "Off" in switch_text:
                d(resourceId=SETTINGS_WIFI_SWITCH_RESID).click()
                time.sleep(1)
        d.press.back()
        '''
        result = ApiTestsInterface\
            .run_instrumentation(class_name="WifiTestsDriver",
                                 method_name="testEnableWifi",
                                 instrumentation_args=None,
                                 runner_name="GenericArgumentPassingTestRunner")
        LOG.info(result)
        return ApiTestsInterface.was_instrumentation_test_successful(result)

    @staticmethod
    def disable_wifi():
        ''' UI deprecated code
        d.press.home()
        Settings.launch()
        if d(resourceId=SETTINGS_OPTION_RESID).wait.exists(timeout=2000):
            d(resourceId=SETTINGS_OPTION_RESID)[0].click()
        if d(resourceId=SETTINGS_WIFI_SWITCH_RESID).wait.exists(timeout=2000):
            switch_text = d(resourceId=SETTINGS_WIFI_SWITCH_RESID).info["text"]
            if "On" in switch_text:
                d(resourceId=SETTINGS_WIFI_SWITCH_RESID).click()
                time.sleep(1)
        d.press.back()
        '''
        result = ApiTestsInterface\
            .run_instrumentation(class_name="WifiTestsDriver",
                                 method_name="testDisableWifi",
                                 instrumentation_args=None,
                                 runner_name="GenericArgumentPassingTestRunner")
        LOG.info(result)
        return ApiTestsInterface.was_instrumentation_test_successful(result)

    @staticmethod
    def enable_airplane_mode():
        Settings.launch()
        if not d(text=SETTINGS_MORE_TXT).wait.exists(timeout=2000):
            LOG.info('Failed to find %s option.' % SETTINGS_MORE_TXT)
            return False
        if not d(text=SETTINGS_MORE_TXT).click.wait(timeout=2000):
            LOG.info('Failed to click on %s option.' % SETTINGS_MORE_TXT)
            return False
        more_switches = dict(UiAutomatorExtended.get_mapping_using_class(ANDROID_WIDGET_SWITCH_CLASSNAME))
        airplane_status = more_switches[SETTINGS_AIRPLANE_MODE_TXT]
        if airplane_status == 'true':
            LOG.info('%s is already ON.' % SETTINGS_AIRPLANE_MODE_TXT)
            d.press.back()
            return True
        LOG.info('Switching %s to ON.' % SETTINGS_AIRPLANE_MODE_TXT)
        UiAutomatorExtended.change_status_for_text(SETTINGS_AIRPLANE_MODE_TXT,
                                                   ANDROID_WIDGET_SWITCH_CLASSNAME)
        # Check that Airplane Mode switched ON
        more_switches = dict(UiAutomatorExtended.get_mapping_using_class(ANDROID_WIDGET_SWITCH_CLASSNAME))
        airplane_status = more_switches[SETTINGS_AIRPLANE_MODE_TXT]
        if airplane_status != 'true':
            LOG.info('%s did not switched ON.' % SETTINGS_AIRPLANE_MODE_TXT)
            d.press.back()
            return False
        d.press.back()
        return True

    @staticmethod
    def disable_airplane_mode():
        Settings.launch()
        if not d(text=SETTINGS_MORE_TXT).wait.exists(timeout=2000):
            LOG.info('Failed to find %s option.' % SETTINGS_MORE_TXT)
            return False
        if not d(text=SETTINGS_MORE_TXT).click.wait(timeout=2000):
            LOG.info('Failed to click on %s option.' % SETTINGS_MORE_TXT)
            return False
        more_switches = dict(UiAutomatorExtended.get_mapping_using_class(ANDROID_WIDGET_SWITCH_CLASSNAME))
        airplane_status = more_switches[SETTINGS_AIRPLANE_MODE_TXT]
        if airplane_status == 'false':
            LOG.info('%s is already OFF.' % SETTINGS_AIRPLANE_MODE_TXT)
            d.press.back()
            return True
        LOG.info('Switching %s to OFF.' % SETTINGS_AIRPLANE_MODE_TXT)
        UiAutomatorExtended.change_status_for_text(SETTINGS_AIRPLANE_MODE_TXT,
                                                   ANDROID_WIDGET_SWITCH_CLASSNAME)
        # Check that Airplane Mode switched OFF
        more_switches = dict(UiAutomatorExtended.get_mapping_using_class(ANDROID_WIDGET_SWITCH_CLASSNAME))
        airplane_status = more_switches[SETTINGS_AIRPLANE_MODE_TXT]
        if airplane_status != 'false':
            LOG.info('%s did not switched OFF.' % SETTINGS_AIRPLANE_MODE_TXT)
            d.press.back()
            return False
        d.press.back()
        return True

    @staticmethod
    def open_data_usage():
        Settings.launch()
        if d(text=SETTINGS_DATA_USAGE_TXT).wait.exists(timeout=2000):
            d(text=SETTINGS_DATA_USAGE_TXT).click()

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
                if len(Info.get_description(app)) > 2:
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
    def go_to_main_screen():
        navigate_tries = 0
        max_tries = 7
        while not d(description=SETTINGS_NAVIGATE_UP_DESC).wait.gone(timeout=3000):
            d(description=SETTINGS_NAVIGATE_UP_DESC).click()
            navigate_tries += 1
            if navigate_tries >= max_tries:
                return

    @staticmethod
    def navigate_settings_menu(list_of_menu_options):
        Settings.launch()
        Settings.go_to_main_screen()
        view_navigator = ViewNavigator()
        view_navigator.nagivate_text(list_of_menu_options)

    @staticmethod
    def sync_app_with_google_account(app_name, nr_of_clicks=2, wait_for_sync_timeout=3):
        list_of_menu_options = [SETTINGS_ACCOUNTS_OPTION_TXT, SETTINGS_GOOGLE_ACCOUNT_OPTION_TXT,
                                app_name]
        Settings.navigate_settings_menu(list_of_menu_options)
        time.sleep(2)
        for i in range(nr_of_clicks - 1):
            if d(textContains=app_name).wait.exists(timeout=2000):
                d(textContains=app_name).click()
                time.sleep(wait_for_sync_timeout)

    @staticmethod
    def remove_google_account():
        list_of_menu_options = [SETTINGS_ACCOUNTS_OPTION_TXT, SETTINGS_GOOGLE_ACCOUNT_OPTION_TXT]
        Settings.navigate_settings_menu(list_of_menu_options)
        d.press.menu()
        if d(text=SETTINGS_GOOGLE_ACCOUNT_REMOVE_TXT).wait.exists(timeout=3000):
            d(text=SETTINGS_GOOGLE_ACCOUNT_REMOVE_TXT).click()
            d(text=SETTINGS_GOOGLE_ACCOUNT_REMOVE_POPUP_TXT).wait.exists(timeout=3000)
            return d(text=SETTINGS_GOOGLE_ACCOUNT_REMOVE_POPUP_TXT).click()
        return False

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

    @staticmethod
    def select_pinyin_material_light_theme():
        list_of_menu_options = [SETTINGS_LANGUAGE_AND_INPUT_OPTIONS_TXT, SETTINGS_PINYIN_KEYBOARD_OPTIONS_TXT,
                                SETTINGS_KEYBOARD_SUB_OPTION_TXT, SETTINGS_KEYBOARD_THEME_SUB_OPTION_TXT,
                                SETTINGS_KEYBOARD_THEME_MATERIAL_LIGHT_TXT]
        Settings.navigate_settings_menu(list_of_menu_options)

    @staticmethod
    def select_pinyin_material_dark_theme():
        list_of_menu_options = [SETTINGS_LANGUAGE_AND_INPUT_OPTIONS_TXT, SETTINGS_PINYIN_KEYBOARD_OPTIONS_TXT,
                                SETTINGS_KEYBOARD_SUB_OPTION_TXT, SETTINGS_KEYBOARD_THEME_SUB_OPTION_TXT,
                                SETTINGS_KEYBOARD_THEME_MATERIAL_DARK_TXT]
        Settings.navigate_settings_menu(list_of_menu_options)

    @staticmethod
    def select_current_keyboard():
        list_of_menu_options = [SETTINGS_LANGUAGE_AND_INPUT_OPTIONS_TXT,
                                SETTINGS_CURRENT_KEYBOARD_OPTION_TXT]
        Settings.navigate_settings_menu(list_of_menu_options)

    @staticmethod
    def open_storage_usb_options():
        list_of_menu_options = [SETTINGS_STORAGE_AND_USB_OPTIONS_TXT]
        Settings.navigate_settings_menu(list_of_menu_options)

    @staticmethod
    def unmount_sdcard():
        Settings.open_storage_usb_options()
        if d(textContains=STORAGE_EJECTED_TXT).wait.exists(timeout=3000):
            LOG.info("SD card is already unmounted")
        else:
            Settings.vn.navigate_views([STORAGE_SDCARD_UNOUNT_ICON_RESID])
            if Settings.vn.current_view_found:
                return
            Settings.vn.navigate_views([STORAGE_SDCARD_TXT])
            time.sleep(2)
            d.press.menu()
            # after pressing first eject, there will be another dialog with a second eject button
            Settings.vn.navigate_views([STORAGE_UNMOUNT_TXT, 2, STORAGE_UNMOUNT_RESID, 2])

    @staticmethod
    def mount_sdcard():
        Settings.open_storage_usb_options()
        if d(textContains=STORAGE_EJECTED_TXT).wait.exists(timeout=3000):
            Settings.vn.navigate_views([STORAGE_SDCARD_TXT, 2, STORAGE_SDCARD_MOUNT_TXT, 2])
        else:
            LOG.info("SD card is already mounted")

    @staticmethod
    def take_bug_report():
        list_of_menu_options = [SETTINGS_DEVELOPER_OPTIONS_TXT, SETTINGS_TAKE_BUG_REPORT_OPTION_TXT]
        Settings.navigate_settings_menu(list_of_menu_options)

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
        list_of_menu_options = [SETTINGS_SECURITY_TXT, SETTINGS_SCREEN_LOCK_TXT, SETTINGS_SCREEN_LOCK_PIN_TXT,
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
        list_of_pin_options = [SETTINGS_SCREEN_LOCK_SWIPE_TXT, SETTINGS_REMOVE_DEVICE_PROTECTION_TXT]

        Settings.navigate_settings_menu(list_of_menu_options)

        # Input the PIN via shell
        # The numpad is not visible in uiautomator
        AdbUtils.input_text(TEST_PIN)
        time.sleep(1)
        d.press.enter()
        time.sleep(1)

        vn = ViewNavigator()
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

        vn = ViewNavigator()
        vn.nagivate_text(list_of_smart_lock_options)

        if location == "current":
            view_to_click = d(text=SETTINGS_SMART_LOCK_SELECT_CURRENT_LOCATION_TXT)
            assert view_to_click.wait.exists(timeout=10000), "The view with text={0} did not appear ".format(
                SETTINGS_SMART_LOCK_SELECT_CURRENT_LOCATION_TXT)
            view_to_click.click()

            trusted_place_name_view = d(resourceId=SETTINGS_TRUSTED_PLACE_NAME_RESID)
            assert trusted_place_name_view.wait.exists(timeout=10000), "The view with resid={0} did not appear".format(
                SETTINGS_TRUSTED_PLACE_NAME_RESID)
            # trusted_place_name_view.clear_text()
            UiAutomatorUtils.clear_text_from_edit_text(SETTINGS_TRUSTED_PLACE_NAME_RESID)
            trusted_place_name_view.set_text(location)
            UiAutomatorUtils.click_view_with_text(POPUP_OK)
        else:
            place_search = d(resourceId=SETTINGS_TRUSTED_PLACES_SEARCH_RESID)
            assert place_search.wait.exists(timeout=10000), "The view with resid={0} did not appear".format(
                SETTINGS_TRUSTED_PLACES_SEARCH_RESID)
            place_search.click()
            d(text="Search").set_text(location)
            time.sleep(2)
            d.press.enter()

            # Wait for the first suggestion
            suggestion = d(resourceId=SETTINGS_TRUSTED_PLACE_SUGGESTION_PRIMARY_RESID, text=location.split(",")[0])
            if suggestion.wait.exists(timeout=10000):
                suggestion.click()

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
    def change_mobile_network(mobile_network):
        mobile_network_types = ["2G", "3G", "4G"]
        assert mobile_network in mobile_network_types, "Mobile network type not in list: " + str(mobile_network_types)
        if mobile_network in mobile_network_types:
            list_of_menu_options = [SETTINGS_MORE_TXT, SETTINGS_MOBILE_NETWORKS_TXT, SETTINGS_NETWORK_TYPE_TXT, mobile_network]
            Settings.navigate_settings_menu(list_of_menu_options)

    @staticmethod
    def add_broken_apn():
        list_of_menu_options = [SETTINGS_MORE_TXT, SETTINGS_MOBILE_NETWORKS_TXT, SETTINGS_ACCESS_POINT_TXT]

        Settings.navigate_settings_menu(list_of_menu_options)
        success = 0
        if d(descriptionContains=SETTINGS_NEW_APN_TXT).wait.exists(timeout=3000):
            d(descriptionContains=SETTINGS_NEW_APN_TXT).click()
            success += 1
        if d(text=SETTINGS_NEW_APN_NAME_TXT).wait.exists(timeout=3000):
            # set_text does not work, it needs to also press OK
            d(text=SETTINGS_NEW_APN_NAME_TXT).click()
            AdbUtils.input_text(TEST_APN_NAME)
            UiAutomatorUtils.click_view_with_text(POPUP_OK)
            success += 1
        if d(text=SETTINGS_NEW_APN_APN_TXT).wait.exists(timeout=3000):
            d(text=SETTINGS_NEW_APN_APN_TXT).click()
            AdbUtils.input_text(TEST_APN)
            UiAutomatorUtils.click_view_with_text(POPUP_OK)
            success += 1
        if d(description=SETTINGS_MORE_OPTIONS_TXT).wait.exists(timeout=3000):
            d(description=SETTINGS_MORE_OPTIONS_TXT).click()
            success += 1
        if d(text=SETTINGS_SAVE_TXT).wait.exists(timeout=3000):
            d(text=SETTINGS_SAVE_TXT).click()
            success += 1
        return success == 5

    @staticmethod
    def select_first_apn():
        if d(resourceId=SETTINGS_APN_RADIO_BUTTON_RESID).wait.exists(timeout=10000):
            d(resourceId=SETTINGS_APN_RADIO_BUTTON_RESID).click()
            return True
        return False

    @staticmethod
    def reset_apns():
        list_of_menu_options = [SETTINGS_MORE_TXT, SETTINGS_MOBILE_NETWORKS_TXT, SETTINGS_ACCESS_POINT_TXT]

        Settings.navigate_settings_menu(list_of_menu_options)
        if d(description=SETTINGS_MORE_OPTIONS_TXT).wait.exists(timeout=3000):
            d(description=SETTINGS_MORE_OPTIONS_TXT).click()
            if d(text=SETTINGS_RESET_DEFAULT_TXT).wait.exists(timeout=3000):
                d(text=SETTINGS_RESET_DEFAULT_TXT).click()
                time.sleep(40)
                # d.press.home()
                return True
        # d.press.home()
        return False

    @staticmethod
    def get_checked_apn():
        list_of_menu_options = [SETTINGS_MORE_TXT, SETTINGS_MOBILE_NETWORKS_TXT, SETTINGS_ACCESS_POINT_TXT]

        Settings.navigate_settings_menu(list_of_menu_options)
        pass

    @staticmethod
    def select_apn(element):
        # if d(element.resourceId).right(resourceId="com.android.settings:id/apn_radiobutton").exists:
        #     d(element.resourceId).right(resourceId="com.android.settings:id/apn_radiobutton").click()
        pass

    @staticmethod
    def is_apn_selected():
        list_of_menu_options = [SETTINGS_MORE_TXT, SETTINGS_MOBILE_NETWORKS_TXT, SETTINGS_ACCESS_POINT_TXT]

        Settings.navigate_settings_menu(list_of_menu_options)
        if d(checked=True).exists:
            d.press.home()
            return True
        d.press.home()
        return False
