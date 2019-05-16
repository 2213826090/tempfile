from PyUiApi.common.system_utils import *
from PyUiApi.common.uiautomator_extension import *
from PyUiApi.common.uiautomator_utils import *
from PyUiApi.common.screenshot_utils import *
from PyUiApi.common.telephony_utils import *
from PyUiApi.app_utils.messenger_utils import *
from PyUiApi.app_utils.settings_utils import *
from PyUiApi.app_utils.dialer_utils import *

from PyUiApi.multi_dut_support.dut_manager import *

DIAL_TIME = 3
FORWARDING_ACTIVATION = 12
FORWARDING_SWITCH_TIME = 20
CALLING_TIME = 50


class CallForwardingTests(unittest.TestCase):
    TAG = "CallForwardingTests"

    def setUp(self):
        UiAutomatorUtils.unlock_screen()
        self.initial_orientation = d.orientation
        # should make a better mechanism for phone devices verification
        try:
            self.phone1_number = dut_manager.acs_config[u'PHONE1'][u'phoneNumber']
            self.phone2_number = dut_manager.acs_config[u'PHONE2'][u'phoneNumber']
            self.phone3_number = dut_manager.acs_config[u'PHONE3'][u'phoneNumber']
        except:
            LOG.info("Phone numbers for devices are not defined in bench config")

    def tearDown(self):
        UiAutomatorUtils.close_all_tasks()

        if d.orientation != self.initial_orientation:
            d.orientation = self.initial_orientation
            d.freeze_rotation(False)

        dut_manager.activate_phone("PHONE1")
        d.press.home()
        if d(text=DIALER_CONFIRMATION).wait.exists(timeout=5000):
            d(text=DIALER_CONFIRMATION).click()
        StatusBar.open_notifications()
        StatusBar.hang_phone()
        StatusBar.clear_notifications()
        Settings.disable_airplane_mode()
        Dialer.launch()
        Dialer.dial_number_using_keyboard("#21#")
        if d(text=DIALER_CONFIRMATION).wait.exists(timeout=5000):
            d(text=DIALER_CONFIRMATION).click()
        UiAutomatorUtils.close_all_tasks()
        Dialer.launch()
        Dialer.dial_number_using_keyboard("#61#")
        if d(text=DIALER_CONFIRMATION).wait.exists(timeout=5000):
            d(text=DIALER_CONFIRMATION).click()
        UiAutomatorUtils.close_all_tasks()

        dut_manager.activate_phone("PHONE2")
        d.press.home()
        StatusBar.open_notifications()
        StatusBar.hang_phone()
        StatusBar.clear_notifications()
        UiAutomatorUtils.close_all_tasks()

        dut_manager.activate_phone("PHONE3")
        d.press.home()
        StatusBar.open_notifications()
        StatusBar.hang_phone()
        StatusBar.clear_notifications()
        UiAutomatorUtils.close_all_tasks()
        dut_manager.activate_phone("PHONE1")

    def test_code21(self):
        """
        ST_TELEPHONY_SS_CF_001
        """
        cf_code = "*21*" + self.phone2_number + "#"
        registration_successful = "Registration was successful"
        cf_status_code = "*#21#"
        deactivate_forwarding_code = "#21#"
        forwarding_disabled = "Service has been disabled"
        activate_forwarding_code = "*21#"
        forwarding_enabled = "Service was enabled"
        erasure_code = "##21#"
        erase_successful = "Erasure was successful"
        not_forwarded = "Not forwarded"

        Dialer.launch()
        Dialer.dial_number_using_keyboard(cf_code)
        d(resourceId=DIALER_MMI_POPUP).wait.exists(timeout=5000)
        self.assertTrue(registration_successful in d(resourceId=DIALER_MMI_POPUP).text,
                        "Registration successful message was not found")
        d(text=DIALER_CONFIRMATION).click()

        UiAutomatorUtils.close_all_tasks()
        Dialer.launch()
        Dialer.dial_number_using_keyboard(cf_status_code)
        d(resourceId=DIALER_MMI_POPUP).wait.exists(timeout=5000)
        self.assertTrue("Call forwarding" in d(resourceId=DIALER_MMI_POPUP).text and self.phone2_number in d(
            resourceId=DIALER_MMI_POPUP).text,
                        "Registration successful message was not found")
        d(text=DIALER_CONFIRMATION).click()

        UiAutomatorUtils.close_all_tasks()
        Dialer.launch()
        Dialer.voice_call("PHONE3", "PHONE1", check_message=False)
        dut_manager.activate_phone("PHONE2")
        StatusBar.open_notifications()
        self.assertTrue(d(text=DIALER_INCOMING_CALL_TXT).wait.exists(timeout=5000),
                        "Phone call not forwarded to phone3.")
        StatusBar.dismiss_phone()
        UiAutomatorUtils.close_all_tasks()
        dut_manager.activate_phone("PHONE1")
        StatusBar.open_notifications()
        self.assertFalse(d(text=DIALER_MISSED_CALL_TXT).wait.exists(timeout=5000),
                         "Phone call should have been fowarded.")
        UiAutomatorUtils.close_all_tasks()

        Dialer.launch()
        Dialer.dial_number_using_keyboard(deactivate_forwarding_code)
        d(resourceId=DIALER_MMI_POPUP).wait.exists(timeout=5000)
        self.assertTrue(forwarding_disabled in d(resourceId=DIALER_MMI_POPUP).text,
                        "Call forwarding disabled message was not found")
        d(text=DIALER_CONFIRMATION).click()

        UiAutomatorUtils.close_all_tasks()

        dut_manager.activate_phone("PHONE3")
        UiAutomatorUtils.close_all_tasks()
        time.sleep(1)

        Dialer.voice_call("PHONE3", "PHONE1")
        dut_manager.activate_phone("PHONE3")
        Dialer.end_call()
        UiAutomatorUtils.close_all_tasks()
        dut_manager.activate_phone("PHONE1")

        UiAutomatorUtils.close_all_tasks()
        Dialer.launch()
        Dialer.dial_number_using_keyboard(activate_forwarding_code)
        time.sleep(2)
        d(resourceId=DIALER_MMI_POPUP).wait.exists(timeout=5000)
        self.assertTrue(forwarding_enabled in d(resourceId=DIALER_MMI_POPUP).text,
                        "Call forwarding enabled message was not found")
        d(text=DIALER_CONFIRMATION).click()

        UiAutomatorUtils.close_all_tasks()
        Dialer.launch()
        Dialer.dial_number_using_keyboard(cf_status_code)
        d(resourceId=DIALER_MMI_POPUP).wait.exists(timeout=5000)
        self.assertTrue("Call forwarding" in d(resourceId=DIALER_MMI_POPUP).text and self.phone2_number in d(
            resourceId=DIALER_MMI_POPUP).text,
                        "Registration successful message was not found")
        d(text=DIALER_CONFIRMATION).click()

        dut_manager.activate_phone("PHONE3")
        UiAutomatorUtils.close_all_tasks()
        time.sleep(3)

        Dialer.voice_call("PHONE3", "PHONE1", check_message=False)
        dut_manager.activate_phone("PHONE2")
        StatusBar.open_notifications()
        self.assertTrue(d(text=DIALER_INCOMING_CALL_TXT).wait.exists(timeout=5000),
                        "Phone call not forwarded to phone3.")
        StatusBar.dismiss_phone()
        UiAutomatorUtils.close_all_tasks()
        dut_manager.activate_phone("PHONE1")
        StatusBar.open_notifications()
        self.assertFalse(d(text=DIALER_MISSED_CALL_TXT).wait.exists(timeout=5000),
                         "Phone call should have been fowarded.")
        UiAutomatorUtils.close_all_tasks()

        dut_manager.activate_phone("PHONE1")
        UiAutomatorUtils.close_all_tasks()
        Dialer.launch()
        Dialer.dial_number_using_keyboard(erasure_code)
        time.sleep(2)
        self.assertTrue(erase_successful in d(resourceId=DIALER_MMI_POPUP).text,
                        "Erasure successful message was not found")
        d(text=DIALER_CONFIRMATION).click()

        UiAutomatorUtils.close_all_tasks()
        Dialer.launch()
        Dialer.dial_number_using_keyboard(cf_status_code)
        d(resourceId=DIALER_MMI_POPUP).wait.exists(timeout=5000)
        self.assertTrue(not_forwarded in d(resourceId=DIALER_MMI_POPUP).text,
                        "Registration successful message was not found")
        d(text=DIALER_CONFIRMATION).click()

        dut_manager.activate_phone("PHONE3")
        UiAutomatorUtils.close_all_tasks()

        Dialer.voice_call("PHONE3", "PHONE1")
        dut_manager.activate_phone("PHONE3")
        StatusBar.open_notifications()
        StatusBar.hang_phone()

    def test_call_forwarding_menu(self):
        """
        ST_TELEPHONY_SS_CF_002
        """
        Dialer.launch()
        view_navigator = ViewNavigator()
        view_navigator.navigate_views(
            [DIALER_MORE_OPTIONS_TXT, DIALER_SETTINGS_TXT, DIALER_CALLS_TXT, DIALER_CALL_FORWARDING])
        time.sleep(FORWARDING_ACTIVATION)
        if d(text=DIALER_ALWAYS_FORWARD_TXT).wait.exists(timeout=10000):
            d(text=DIALER_ALWAYS_FORWARD_TXT).click()
        if d(resourceId=DIALER_EDIT).wait.exists(timeout=5000):
            d(resourceId=DIALER_EDIT).set_text(self.phone2_number)
        if d(text=DIALER_TURN_ON_TXT).wait.exists(timeout=5000):
            d(text=DIALER_TURN_ON_TXT).click()
        time.sleep(FORWARDING_ACTIVATION)
        self.assertTrue(d(textContains="Forwarding all calls to").wait.exists(timeout=5000),
                        "Forwarding status not detected in call forwarding settings")
        Dialer.voice_call("PHONE3", "PHONE1", check_message=False)
        StatusBar.open_notifications()
        self.assertFalse(d(text=DIALER_MISSED_CALL_TXT).wait.exists(timeout=5000),
                         "Phone call should have been forwarded.")
        dut_manager.activate_phone("PHONE2")
        StatusBar.open_notifications()
        self.assertTrue(d(text=DIALER_INCOMING_CALL_TXT).wait.exists(timeout=5000),
                        "Phone call not forwarded to phone3.")
        StatusBar.dismiss_phone()
        dut_manager.activate_phone("PHONE1")
        StatusBar.close_notifications()
        view_navigator.navigate_views([DIALER_ALWAYS_FORWARD_TXT, DIALER_TURN_OFF_TXT])
        time.sleep(FORWARDING_ACTIVATION)
        dut_manager.activate_phone("PHONE3")
        UiAutomatorUtils.close_all_tasks()
        Dialer.voice_call("PHONE3", "PHONE1")
        dut_manager.activate_phone("PHONE3")
        StatusBar.open_notifications()
        StatusBar.hang_phone()

    def test_code62(self):
        """
        ST_TELEPHONY_SS_CF_005
        """

        cf_code = "**62*" + self.phone2_number + "#"
        registration_successful = "Registration was successful"
        cf_status_code = "*#62#"
        deactivate_forwarding_code = "#62#"
        forwarding_disabled = "Service has been disabled"
        activate_forwarding_code = "*62#"
        forwarding_enabled = "Service was enabled"
        erasure_code = "##62#"
        erase_successful = "Erasure was successful"
        not_forwarded = "Not forwarded"

        Dialer.launch()
        Dialer.dial_number_using_keyboard(cf_code)
        d(resourceId=DIALER_MMI_POPUP).wait.exists(timeout=5000)
        self.assertTrue(registration_successful in d(resourceId=DIALER_MMI_POPUP).text,
                        "Registration successful message was not found")
        d(text=DIALER_CONFIRMATION).click()

        UiAutomatorUtils.close_all_tasks()
        Dialer.launch()
        Dialer.dial_number_using_keyboard(cf_status_code)
        d(resourceId=DIALER_MMI_POPUP).wait.exists(timeout=5000)
        self.assertTrue(self.phone2_number in d(resourceId=DIALER_MMI_POPUP).text,
                        "Registration successful message was not found")
        d(text=DIALER_CONFIRMATION).click()

        self.assertTrue(Settings.enable_airplane_mode(), "Airplane mode couldn't be switched on")

        Dialer.voice_call("PHONE3", "PHONE1", check_number=False, check_message=False)
        dut_manager.activate_phone("PHONE1")
        StatusBar.open_notifications()
        self.assertFalse(d(text=DIALER_INCOMING_CALL_TXT).wait.exists(timeout=5000),
                         "Phone call should have been forwarded.")
        UiAutomatorUtils.close_all_tasks()

        dut_manager.activate_phone("PHONE2")
        StatusBar.open_notifications()
        self.assertTrue(d(text=DIALER_DISMISS_TXT).wait.exists(timeout=5000), "Dismiss button not available")
        d(text=DIALER_DISMISS_TXT).click()
        StatusBar.close_notifications()

        dut_manager.activate_phone("PHONE1")
        self.assertTrue(Settings.disable_airplane_mode(), "Airplane mode couldn't be switched off")
        Dialer.launch()
        Dialer.dial_number_using_keyboard(deactivate_forwarding_code)
        d(resourceId=DIALER_MMI_POPUP).wait.exists(timeout=5000)
        self.assertTrue(forwarding_disabled in d(resourceId=DIALER_MMI_POPUP).text,
                        "Call forwarding disabled message was not found")
        d(text=DIALER_CONFIRMATION).click()

        UiAutomatorUtils.close_all_tasks()
        Dialer.launch()
        Dialer.dial_number_using_keyboard(cf_status_code)
        d(resourceId=DIALER_MMI_POPUP).wait.exists(timeout=5000)
        self.assertTrue(not_forwarded in d(resourceId=DIALER_MMI_POPUP).text,
                        "Registration successful message was not found")
        d(text=DIALER_CONFIRMATION).click()

        UiAutomatorUtils.close_all_tasks()
        Dialer.launch()
        Dialer.dial_number_using_keyboard(activate_forwarding_code)
        d(resourceId=DIALER_MMI_POPUP).wait.exists(timeout=5000)
        self.assertTrue(forwarding_enabled in d(resourceId=DIALER_MMI_POPUP).text,
                        "Registration successful message was not found")
        d(text=DIALER_CONFIRMATION).click()

        UiAutomatorUtils.close_all_tasks()
        Dialer.launch()
        Dialer.dial_number_using_keyboard(cf_status_code)
        d(resourceId=DIALER_MMI_POPUP).wait.exists(timeout=5000)
        self.assertTrue(self.phone2_number in d(resourceId=DIALER_MMI_POPUP).text,
                        "Number doesn't appear to be forwarded")
        d(text=DIALER_CONFIRMATION).click()

        self.assertTrue(Settings.enable_airplane_mode(), "Airplane mode couldn't be switched on")
        Dialer.launch()
        dut_manager.activate_phone("PHONE3")
        UiAutomatorUtils.close_all_tasks()
        Dialer.voice_call("PHONE3", "PHONE1", check_number=False, check_message=False)
        dut_manager.activate_phone("PHONE1")
        StatusBar.open_notifications()
        self.assertFalse(d(text=DIALER_INCOMING_CALL_TXT).wait.exists(timeout=5000),
                         "Phone call should have been forwarded.")

        dut_manager.activate_phone("PHONE2")
        StatusBar.open_notifications()
        self.assertTrue(d(text=DIALER_DISMISS_TXT).wait.exists(timeout=5000), "Dismiss button not available")
        d(text=DIALER_DISMISS_TXT).click()
        StatusBar.close_notifications()
        dut_manager.activate_phone("PHONE1")
        self.assertTrue(Settings.disable_airplane_mode(), "Airplane mode couldn't be switched off")

        UiAutomatorUtils.close_all_tasks()
        Dialer.launch()
        Dialer.dial_number_using_keyboard(erasure_code)
        time.sleep(2)
        self.assertTrue(erase_successful in d(resourceId=DIALER_MMI_POPUP).text,
                        "Erasure successful message was not found")
        d(text=DIALER_CONFIRMATION).click()

        UiAutomatorUtils.close_all_tasks()
        Dialer.launch()
        Dialer.dial_number_using_keyboard(cf_status_code)
        d(resourceId=DIALER_MMI_POPUP).wait.exists(timeout=5000)
        self.assertTrue(not_forwarded in d(resourceId=DIALER_MMI_POPUP).text,
                        "Registration successful message was not found")
        d(text=DIALER_CONFIRMATION).click()

    def test_call_forwarding_menu_not_reachable(self):
        """
        ST_TELEPHONY_SS_CF_006
        """
        Dialer.launch()
        view_navigator = ViewNavigator()
        view_navigator.navigate_views(
            [DIALER_MORE_OPTIONS_TXT, DIALER_SETTINGS_TXT, DIALER_CALLS_TXT, DIALER_CALL_FORWARDING])
        time.sleep(FORWARDING_ACTIVATION)
        if d(text=DIALER_WHEN_UNREACHABLE_TXT).wait.exists(timeout=10000):
            d(text=DIALER_WHEN_UNREACHABLE_TXT).click()
        if d(resourceId=DIALER_EDIT).wait.exists(timeout=5000):
            d(resourceId=DIALER_EDIT).set_text(self.phone2_number)
        if d(text=DIALER_TURN_ON_TXT).wait.exists(timeout=5000):
            d(text=DIALER_TURN_ON_TXT).click()
        time.sleep(FORWARDING_ACTIVATION)
        self.assertTrue(d(textContains="Forwarding to").wait.exists(timeout=5000),
                        "Forwarding status not detected in call forwarding settings")
        self.assertTrue(Settings.enable_airplane_mode(), "Airplane mode couldn't be switched on")
        Dialer.voice_call("PHONE3", "PHONE1", check_message=False, check_number=False)
        StatusBar.open_notifications()
        self.assertFalse(d(text=DIALER_MISSED_CALL_TXT).wait.exists(timeout=5000),
                         "Phone call should have been forwarded.")
        StatusBar.close_notifications()
        dut_manager.activate_phone("PHONE2")
        StatusBar.open_notifications()
        self.assertTrue(d(text=DIALER_INCOMING_CALL_TXT).wait.exists(timeout=5000),
                        "Phone call not forwarded to phone2.")
        StatusBar.dismiss_phone()
        StatusBar.close_notifications()
        dut_manager.activate_phone("PHONE1")
        StatusBar.close_notifications()
        self.assertTrue(Settings.disable_airplane_mode(), "Airplane mode couldn't be switched off")
        UiAutomatorUtils.close_all_tasks()
        Dialer.launch()
        view_navigator = ViewNavigator()
        view_navigator.navigate_views(
            [DIALER_MORE_OPTIONS_TXT, DIALER_SETTINGS_TXT, DIALER_CALLS_TXT, DIALER_CALL_FORWARDING])
        time.sleep(FORWARDING_ACTIVATION)
        view_navigator.navigate_views([DIALER_WHEN_UNREACHABLE_TXT, DIALER_TURN_OFF_TXT])

        time.sleep(FORWARDING_ACTIVATION)
        dut_manager.activate_phone("PHONE3")
        UiAutomatorUtils.close_all_tasks()
        Dialer.voice_call("PHONE3", "PHONE1")
        dut_manager.activate_phone("PHONE3")
        StatusBar.open_notifications()
        StatusBar.hang_phone()

    def test_code61(self):
        """
        ST_TELEPHONY_SS_CF_007
        """
        cf_code = "**61*" + self.phone2_number + "#"
        registration_successful = "Registration was successful"
        call_forwarding = "Call forwarding"
        cf_status_code = "*#61#"
        deactivate_forwarding_code = "#61#"
        forwarding_disabled = "Service has been disabled"
        activate_forwarding_code = "*61#"
        forwarding_enabled = "Service was enabled"
        erasure_code = "##61#"
        erase_successful = "Erasure was successful"

        Dialer.launch()
        self.assertTrue(Dialer.mmi_call(cf_code, registration_successful),
                        "Registration successful message was not found")
        UiAutomatorUtils.close_all_tasks()
        Dialer.launch()
        self.assertTrue(Dialer.mmi_call(cf_status_code, call_forwarding),
                        "Registration successful message was not found")
        Dialer.launch()
        Dialer.voice_call("PHONE3", "PHONE1")
        dut_manager.activate_phone("PHONE2")
        time.sleep(FORWARDING_SWITCH_TIME)
        StatusBar.open_notifications()
        Dialer.voice_call_check_number("PHONE3")
        StatusBar.close_notifications()

        dut_manager.activate_phone("PHONE1")
        UiAutomatorUtils.close_all_tasks()
        Dialer.launch()
        self.assertTrue(Dialer.mmi_call(deactivate_forwarding_code, forwarding_disabled),
                        "Call forwarding disabled message was not found")
        UiAutomatorUtils.close_all_tasks()
        Dialer.launch()
        self.assertTrue(Dialer.mmi_call(cf_status_code, call_forwarding),
                        "Registration successful message was not found")
        StatusBar.close_notifications()
        UiAutomatorUtils.close_all_tasks()

        dut_manager.activate_phone("PHONE3")
        UiAutomatorUtils.close_all_tasks()

        dut_manager.activate_phone("PHONE2")
        StatusBar.open_notifications()
        StatusBar.clear_notifications()
        StatusBar.close_notifications()

        Dialer.voice_call("PHONE3", "PHONE1")
        dut_manager.activate_phone("PHONE2")
        StatusBar.open_notifications()
        self.assertFalse(
            d(text=DIALER_INCOMING_CALL_TXT).wait.exists(timeout=5000) and d(text=DIALER_MISSED_CALL_TXT).wait.exists(
                timeout=5000),
            "Phone call appears forwarded to phone2 and it shouldn't")
        StatusBar.close_notifications()
        dut_manager.activate_phone("PHONE1")
        StatusBar.open_notifications()
        time.sleep(CALLING_TIME)
        self.assertTrue(d(text=DIALER_MISSED_CALL_TXT).wait.exists(timeout=7000),
                        "Missed call is not present in status bar of phone1")
        StatusBar.close_notifications()

        Dialer.launch()
        self.assertTrue(Dialer.mmi_call(activate_forwarding_code, forwarding_enabled),
                        "Forward activation not found")
        UiAutomatorUtils.close_all_tasks()
        Dialer.launch()
        self.assertTrue(Dialer.mmi_call(cf_status_code, call_forwarding),
                        "Call forwarding not enabled ")

        dut_manager.activate_phone("PHONE3")
        UiAutomatorUtils.close_all_tasks()

        Dialer.voice_call("PHONE3", "PHONE1")
        dut_manager.activate_phone("PHONE2")
        time.sleep(FORWARDING_SWITCH_TIME)
        StatusBar.open_notifications()
        self.assertTrue(
            d(text=DIALER_INCOMING_CALL_TXT).wait.exists(timeout=5000) or d(text=DIALER_MISSED_CALL_TXT).wait.exists(
                timeout=5000),
            "Phone call isn't forwarded to phone2 ")
        StatusBar.close_notifications()

        dut_manager.activate_phone("PHONE1")
        StatusBar.close_notifications()

        UiAutomatorUtils.close_all_tasks()
        Dialer.launch()
        self.assertTrue(Dialer.mmi_call(erasure_code, erase_successful),
                        "Erasure was not successful ")

        UiAutomatorUtils.close_all_tasks()
        Dialer.launch()
        self.assertTrue(Dialer.mmi_call(cf_status_code, call_forwarding),
                        "Call forwarding not enabled ")

    def test_call_forwarding_menu_no_reply(self):
        """
        ST_TELEPHONY_SS_CF_008
        """
        Dialer.launch()
        view_navigator = ViewNavigator()
        view_navigator.navigate_views(
            [DIALER_MORE_OPTIONS_TXT, DIALER_SETTINGS_TXT, DIALER_CALLS_TXT, DIALER_CALL_FORWARDING])
        time.sleep(FORWARDING_ACTIVATION)
        if d(text=DIALER_WHEN_UNANSWERED_TXT).wait.exists(timeout=10000):
            d(text=DIALER_WHEN_UNANSWERED_TXT).click()
        if d(resourceId=DIALER_EDIT).wait.exists(timeout=5000):
            d(resourceId=DIALER_EDIT).set_text(self.phone2_number)
        if d(text=DIALER_TURN_ON_TXT).wait.exists(timeout=5000):
            d(text=DIALER_TURN_ON_TXT).click()
        time.sleep(FORWARDING_ACTIVATION)
        self.assertTrue(d(textContains="Forwarding to").wait.exists(timeout=5000),
                        "Forwarding status not detected in call forwarding settings")

        Dialer.voice_call("PHONE3", "PHONE1")
        time.sleep(CALLING_TIME)
        StatusBar.open_notifications()
        self.assertTrue(
            d(text=DIALER_INCOMING_CALL_TXT).wait.exists(timeout=5000) or d(text=DIALER_MISSED_CALL_TXT).wait.exists(
                timeout=5000),
            "Notification of missed call doesn't appear on phone1")
        StatusBar.clear_notifications()
        StatusBar.close_notifications()

        dut_manager.activate_phone("PHONE2")
        StatusBar.open_notifications()
        self.assertTrue(d(text=DIALER_MISSED_CALL_TXT).wait.exists(timeout=5000), "Call wasn't forwarded to phone2")

        dut_manager.activate_phone("PHONE1")
        UiAutomatorUtils.close_all_tasks()
        Dialer.launch()
        view_navigator = ViewNavigator()
        view_navigator.navigate_views(
            [DIALER_MORE_OPTIONS_TXT, DIALER_SETTINGS_TXT, DIALER_CALLS_TXT, DIALER_CALL_FORWARDING])
        time.sleep(FORWARDING_ACTIVATION)
        view_navigator.navigate_views([DIALER_WHEN_UNANSWERED_TXT, DIALER_TURN_OFF_TXT])
        self.assertFalse(d(textContains="Forwarding to").wait.exists(timeout=5000),
                         "Forwarding isn't deactivated")
        time.sleep(FORWARDING_ACTIVATION)
        dut_manager.activate_phone("PHONE3")
        UiAutomatorUtils.close_all_tasks()
        Dialer.voice_call("PHONE3", "PHONE1")
        dut_manager.activate_phone("PHONE3")
        StatusBar.open_notifications()
        StatusBar.hang_phone()
