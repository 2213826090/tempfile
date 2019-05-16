# -*- coding: UTF-8 -*-
from PyUiApi.common.system_utils import *
from PyUiApi.common.uiautomator_extension import *
from PyUiApi.common.environment_utils import *
from PyUiApi.common.uiautomator_utils import *
from PyUiApi.common.screenshot_utils import *
from PyUiApi.common.telephony_utils import *
from PyUiApi.dut_info.dut_info import *
from PyUiApi.app_utils.messenger_utils import *
from PyUiApi.app_utils.settings_utils import *
from PyUiApi.app_utils.dialer_utils import *

from PyUiApi.multi_dut_support.dut_manager import *

MESSAGE_DELIVERY_TIME = 20
DIAL_TIME = 3
WAIT_FOR_300s = 300


class CallTests(unittest.TestCase):
    TAG = "CallTests"

    def setUp(self):
        # UiAutomatorUtils.unlock_screen()
        UiAutomatorUtils.close_all_tasks()
        self.initial_orientation = d.orientation
        self.screenshooter = ScreenshotUtils()

        # should make a better mechanism for phone devices verification
        try:
            self.phone1_number = dut_manager.acs_config[u'PHONE1'][u'phoneNumber']
            self.phone2_number = dut_manager.acs_config[u'PHONE2'][u'phoneNumber']
            self.internationalPrefix = dut_manager.acs_config[u'PHONE1'][u'internationalPrefix']
        except:
            LOG.info("Phone numbers for devices are not defined in bench config")

        try:
            self.ACTIVATE_VOICE_CALL = dut_manager.acs_config[u'PHONE1'][u'ACTIVATE_VOICE_CALL']
            self.DEACTIVATE_VOICE_CALL = dut_manager.acs_config[u'PHONE1'][u'DEACTIVATE_VOICE_CALL']
            self.VOICE_CALL_SERVER = dut_manager.acs_config[u'PHONE1'][u'VOICE_CALL_SERVER']
        except:
            LOG.info("SIM MMI numbers for devices are not defined in bench config")

    def tearDown(self):
        '''
        self.log_before_cleanup()
        UiAutomatorUtils.close_all_tasks()

        if d.orientation != self.initial_orientation:
            d.orientation = self.initial_orientation
            d.freeze_rotation(False)
        dut_manager.activate_phone("PHONE1")
        d.press.home()
        StatusBar.open_notifications()
        StatusBar.hang_phone()
        StatusBar.clear_notifications()
        UiAutomatorUtils.close_all_tasks()
        dut_manager.activate_phone("PHONE2")
        d.press.home()
        StatusBar.open_notifications()
        StatusBar.hang_phone()
        StatusBar.clear_notifications()
        UiAutomatorUtils.close_all_tasks()
        dut_manager.activate_phone("PHONE1")
        '''

    def log_before_cleanup(self):
        result = self._resultForDoCleanups
        if not result.wasSuccessful():
            TestUtils.log_screenshot_and_dump()

    def test_call_not_answered(self):
        """
        ST_TELEPHONY_VC_CS_104
        """
        StatusBar.open_notifications()
        StatusBar.clear_notifications()
        time.sleep(2)
        dut_manager.activate_phone("PHONE2")
        Dialer.launch()
        Dialer.dial_number(self.phone1_number)
        dut_manager.activate_phone("PHONE1")
        time.sleep(DIAL_TIME)
        StatusBar.open_notifications()
        detected_call = False
        if d(text=DIALER_INCOMING_CALL_TXT).wait.exists(timeout=5000):
            detected_call = True
        dut_manager.activate_phone("PHONE2")
        Dialer.end_call(5000)
        self.assertTrue(detected_call,
                        "Did not detect the incoming call text in the notification bar")
        dut_manager.activate_phone("PHONE1")
        time.sleep(2)
        self.assertTrue(d(text=DIALER_MISSED_CALL_TXT).wait.exists(timeout=5000),
                        "Did not detect the missing call text in the notification bar")

    def test_call_rejected(self):
        """
        ST_TELEPHONY_VC_CS_103
        """

        # disable VOICE CALL
        Dialer.launch()
        Dialer.dial_number_using_keyboard(self.DEACTIVATE_VOICE_CALL)
        if d(text=DIALER_CONFIRMATION).wait.exists(timeout=10000):
            d(text=DIALER_CONFIRMATION).click()

        Dialer.voice_call("PHONE2", "PHONE1")
        if d(text=DIALER_DISMISS_TXT).wait.exists(timeout=5000):
            d(text=DIALER_DISMISS_TXT).click()
        StatusBar.clear_notifications()

        dut_manager.activate_phone("PHONE2")
        self.assertTrue(d(text="Line busy").wait.exists(timeout=10000),
                        "Did not detect the Line busy text in the reference device")

        self.tearDown()

        # enable VOICE CALL
        dut_manager.activate_phone("PHONE1")
        Dialer.launch()
        Dialer.dial_number_using_keyboard(self.ACTIVATE_VOICE_CALL)
        if d(text=DIALER_CONFIRMATION).wait.exists(timeout=10000):
            d(text=DIALER_CONFIRMATION).click()

        Dialer.voice_call("PHONE2", "PHONE1")
        if d(text=DIALER_DISMISS_TXT).wait.exists(timeout=5000):
            d(text=DIALER_DISMISS_TXT).click()
        StatusBar.clear_notifications()
        dut_manager.activate_phone("PHONE2")
        time.sleep(2)
        self.assertTrue(Dialer.end_call(),
                        "The end call button should still be available while in voice call inbox recording")
        dut_manager.activate_phone("PHONE1")
        time.sleep(12)
        self.assertTrue(Messaging.check_sms_received_in_statusbar(self.VOICE_CALL_SERVER),
                        "The voice call server number was not found in the status bar messages")

    def test_voice_call_dialing_method(self):
        """
        ST_TELEPHONY_VC_CS_004
        """

        Messaging.launch()
        Messaging.remove_all_contacts()
        UiAutomatorUtils.close_all_tasks()

        # case1
        Dialer.launch()
        Dialer.dial_number_using_keyboard(self.phone2_number)
        dut_manager.activate_phone("PHONE2")
        Dialer.answer_call(True, self.phone1_number)
        dut_manager.activate_phone("PHONE1")
        StatusBar.open_notifications()
        StatusBar.hang_phone()
        UiAutomatorUtils.close_all_tasks()

        # case2
        Dialer.launch()
        time.sleep(2)
        if d(description=DIALER_RECENTS).wait.exists(timeout=5000):
            d(description=DIALER_RECENTS).click()
        time.sleep(DIAL_TIME)
        d.dump()
        if d(resourceId=DIALER_CONTACT_NAME).wait.exists(timeout=5000):
            if d(resourceId=DIALER_PRIMARY_ACTION_BUTTON).wait.exists(timeout=5000):
                content_list = d(resourceId=DIALER_PRIMARY_ACTION_BUTTON)
                for index, value in enumerate(content_list):
                    if self.phone2_number in value.description.replace(" ", ""):
                        d(resourceId=DIALER_PRIMARY_ACTION_BUTTON)[index].click()
                        break

        dut_manager.activate_phone("PHONE2")
        Dialer.answer_call(True, self.phone1_number)
        dut_manager.activate_phone("PHONE1")
        StatusBar.open_notifications()
        StatusBar.hang_phone()
        UiAutomatorUtils.close_all_tasks()

        # case3
        contact_name = "voice_method"

        Messaging.launch()
        Messaging.add_contact(contact_name, self.phone2_number)
        UiAutomatorUtils.close_all_tasks()

        Dialer.launch()
        if d(description=DIALER_CONTACTS).wait.exists(timeout=5000):
            d(description=DIALER_CONTACTS).click()
        if d(text=contact_name).wait.exists(timeout=5000):
            d(text=contact_name).click()
        time.sleep(1)
        if d(className=DIALER_RELATIVE_LAYOUT).wait.exists(timeout=5000):
            d(className=DIALER_RELATIVE_LAYOUT)[0].click()
        time.sleep(1)
        self.assertTrue(d(text=contact_name).wait.exists(timeout=5000), "Contact name doesn't appear in the voice call")

        dut_manager.activate_phone("PHONE2")
        Dialer.answer_call(True, self.phone1_number)
        dut_manager.activate_phone("PHONE1")
        StatusBar.open_notifications()
        StatusBar.hang_phone()
        UiAutomatorUtils.close_all_tasks()

        # case4
        international_number = self.internationalPrefix + self.phone2_number
        Dialer.launch()
        Dialer.dial_number_using_keyboard(international_number)
        self.assertTrue(d(text=contact_name).wait.exists(timeout=5000), "Contact name doesn't appear in the voice call")

        dut_manager.activate_phone("PHONE2")
        Dialer.answer_call(True, self.phone1_number)
        dut_manager.activate_phone("PHONE1")
        StatusBar.open_notifications()
        StatusBar.hang_phone()
        UiAutomatorUtils.close_all_tasks()

        # case 5
        Dialer.launch()
        time.sleep(1)
        if d(description=DIALER_RECENTS).wait.exists(timeout=5000):
            d(description=DIALER_RECENTS).click()
        if d(text=contact_name).wait.exists(timeout=5000):
            d(text=contact_name).click()
        d.dump()
        if d(resourceId=DIALER_CALL_DETAILS).wait.exists(timeout=5000):
            d(resourceId=DIALER_CALL_DETAILS).click()
        d.dump()
        d(resourceId=DIALER_CALLER_NAME).wait.exists(timeout=5000)
        self.assertTrue(d(resourceId=DIALER_CALLER_NAME).text == contact_name,
                        "Contact name is not correct. Expected/obtained: " + contact_name + "/" + d(
                            resourceId=DIALER_CALLER_NAME).text)
        self.assertTrue(self.phone2_number in d(resourceId=DIALER_CALLER_NUMBER).text.replace(" ", ""),
                        "Phone number not detected in call details. Expected/obtained: " + self.phone2_number + "/" + d(
                            resourceId=DIALER_CALLER_NUMBER).text.replace(" ", ""))
        if d(resourceId=DIALER_CALL_BACK).wait.exists(timeout=5000):
            d(resourceId=DIALER_CALL_BACK).click()
        dut_manager.activate_phone("PHONE2")
        Dialer.answer_call(True, self.phone1_number)
        dut_manager.activate_phone("PHONE1")
        StatusBar.open_notifications()
        StatusBar.hang_phone()
        UiAutomatorUtils.close_all_tasks()
