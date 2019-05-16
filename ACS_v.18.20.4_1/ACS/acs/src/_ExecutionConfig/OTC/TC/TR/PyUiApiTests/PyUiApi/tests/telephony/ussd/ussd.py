# -*- coding: UTF-8 -*-
from PyUiApi.common.system_utils import *
from PyUiApi.app_utils.messenger_utils import *
from PyUiApi.app_utils.dialer_utils import *
from PyUiApi.app_utils.settings_utils import *
from PyUiApi.multi_dut_support.dut_manager import *

MESSAGE_DELIVERY_TIME = 20
USSD_RESPONSE_TIME = 15

class USSDTests(unittest.TestCase):
    TAG = "USSDTests"
    is_ussd_open = False

    def setUp(self):
        UiAutomatorUtils.unlock_screen()
        UiAutomatorUtils.close_all_tasks()
        self.initial_orientation = d.orientation

        try:
            self.phone1_number = dut_manager.acs_config[u'PHONE1'][u'phoneNumber']
            self.phone2_number = dut_manager.acs_config[u'PHONE2'][u'phoneNumber']
        except:
            LOG.info("Phone numbers for devices are not defined in bench config")
        try:
            self.REQUEST_SUBSCRIPTION_SUMMARY = dut_manager.acs_config[u'PHONE1'][u'REQUEST_SUBSCRIPTION_SUMMARY']
        except:
            LOG.info("REQUEST_SUBSCRIPTION_SUMMARY is not defined in bench config")

    def tearDown(self):
        if self.is_ussd_open:
            Dialer.close_ussd()
            self.is_ussd_open = False
        UiAutomatorUtils.close_all_tasks()
        if d.orientation != self.initial_orientation:
            d.orientation = self.initial_orientation
            d.freeze_rotation(False)
        dut_manager.activate_phone("PHONE1")
        Settings.launch()
        Settings.change_mobile_network("2G")
        UiAutomatorUtils.close_all_tasks()
        dut_manager.activate_phone("PHONE2")
        d.press.home()
        StatusBar.open_notifications()
        StatusBar.hang_phone()
        StatusBar.clear_notifications()
        UiAutomatorUtils.close_all_tasks()

    def test_ussd_request_2g(self):
        """
        ST_TELEPHONY_SS_USSD_001
        """
        StatusBar.open_notifications()
        time.sleep(2)
        StatusBar.click_on_first_status_element()
        # dut_manager.activate_phone("PHONE2")
        Settings.launch()
        Settings.change_mobile_network("2G")
        self.assertTrue(d(text="2G").wait.exists(timeout=5000),
                        "Did not detect changing the cellular network to 2G")
        UiAutomatorUtils.close_all_tasks()
        Dialer.launch()
        Dialer.dial_number(self.REQUEST_SUBSCRIPTION_SUMMARY)
        time.sleep(USSD_RESPONSE_TIME)
        if d(text=DIALER_CONFIRMATION).wait.exists(timeout=5000):
            d(text=DIALER_CONFIRMATION).click()
        d.press.home()
        UiAutomatorUtils.close_all_tasks()
        self.assertTrue(Messaging.check_sms_received_in_statusbar(self.REQUEST_SUBSCRIPTION_SUMMARY),
                        "Did not detect the following sms in messenger app: " + self.REQUEST_SUBSCRIPTION_SUMMARY)

    def test_ussd_request_3g(self):
        """
        ST_TELEPHONY_SS_USSD_002
        """

        StatusBar.open_notifications()
        time.sleep(2)
        StatusBar.click_on_first_status_element()
        # dut_manager.activate_phone("PHONE2")
        Settings.launch()
        Settings.change_mobile_network("3G")
        self.assertTrue(d(text="3G").wait.exists(timeout=5000),
                        "Did not detect changing the cellular network to 3G")
        UiAutomatorUtils.close_all_tasks()
        Dialer.launch()
        Dialer.dial_number(self.REQUEST_SUBSCRIPTION_SUMMARY)
        time.sleep(USSD_RESPONSE_TIME)
        if d(text=DIALER_CONFIRMATION).wait.exists(timeout=5000):
            d(text=DIALER_CONFIRMATION).click()
        d.press.home()
        UiAutomatorUtils.close_all_tasks()
        self.assertTrue(Messaging.check_sms_received_in_statusbar(self.REQUEST_SUBSCRIPTION_SUMMARY),
                        "Did not detect the following sms in messenger app: " + self.REQUEST_SUBSCRIPTION_SUMMARY)

    def test_ussd_sequential(self):
        """ ST_TELEPHONY_SS_USSD_003 """

        Settings.launch()
        Settings.change_mobile_network("3G")
        # Not very happy with this check
        self.assertTrue(d(text="3G").wait.exists(timeout=5000),
                        "Did not detect changing the cellular network to 3G")

        UiAutomatorUtils.close_all_tasks()
        Dialer.launch()
        Dialer.dial_number('*100#')

        LOG.info('Check that USSD popup is open.')
        self.assertTrue(Dialer.is_ussd_open(), 'USSD popup not open.')
        self.is_ussd_open = True

        LOG.info('Get available options.')
        first_response = Dialer.get_ussd_message()
        self.assertIsInstance(first_response, dict, 'Failed to get available options. Found: %s' % first_response)
        LOG.info('Found options: %s' % first_response)

        # Hopefully 'Internet' is going to be an available option
        # regardless of the monthly billing plan or Carrier language
        LOG.info("Look for 'Internet' in available options.")
        internet_option = [key for key in first_response if 'Internet' in first_response[key]]
        self.assertEqual(len(internet_option), 1, 'Expected to find only one Internet option.'
                                                  'Found instead: %s' % internet_option)
        internet_option = internet_option[0]

        LOG.info('Send option %s' % internet_option)
        self.assertTrue(Dialer.send_ussd(internet_option), 'Failed to send option %s' % internet_option)

        LOG.info('Get the response.')
        second_response = Dialer.get_ussd_message()
        self.assertIsNotNone(second_response, 'Got no response.')
        LOG.info('Response: %s' % second_response)

        LOG.info('Close the USSD popup.')
        self.assertTrue(Dialer.close_ussd(), 'Failed to close the USSD popup.')
        self.is_ussd_open = False

        LOG.info('Check that the responses differ.')
        self.assertNotEqual(first_response, second_response, 'Did not received a new response.')

    def clir_verifier(self, clir_cmd, clir_expected_msg):
        Dialer.dial_number_using_keyboard(clir_cmd)
        self.assertTrue(d(resourceId=DIALER_MMI_POPUP).wait.exists(timeout=10000),
                        "Did not detect the mmi reply notification for cmd: " + clir_cmd)
        mmi_reply = UiAutomatorUtils.get_attr_content_matching_text(clir_expected_msg)
        time.sleep(USSD_RESPONSE_TIME)
        if not mmi_reply:
            mmi_reply = ""
        self.assertTrue(clir_expected_msg in mmi_reply,
                        "Did not detect clir message " + clir_expected_msg + "for cmd:" + clir_cmd)
        if d(text=DIALER_CONFIRMATION).wait.exists(timeout=5000):
            d(text=DIALER_CONFIRMATION).click()

    def test_clir(self):
        '''
        ST_TELEPHONY_SS_CLIR_001
        '''

        mmi_enable_clir = "*31#"
        enabled = "Outgoing Caller ID\nService was enabled."
        disable_clir_cmd = "#31#"
        disable_clir_msg = "Outgoing Caller ID\nService has been disabled."
        mmi_not_restricted = "*#31#"
        not_restricted = "Next call: Not restricted"
        restricted = "Next call: Restricted"

        dut_manager.activate_phone("PHONE1")
        Messaging.launch()
        Messaging.remove_all_contacts()
        UiAutomatorUtils.close_all_tasks()

        dut_manager.activate_phone("PHONE2")
        Messaging.launch()
        Messaging.remove_all_contacts()
        UiAutomatorUtils.close_all_tasks()

        dut_manager.activate_phone("PHONE1")
        Dialer.launch()

        self.clir_verifier(disable_clir_cmd, disable_clir_msg)
        UiAutomatorUtils.close_all_tasks()

        Dialer.voice_call("PHONE1", "PHONE2", True)
        dut_manager.activate_phone("PHONE1")

        self.assertTrue(Dialer.end_call(), "Can't find end call button")
        UiAutomatorUtils.close_all_tasks()

        Dialer.launch()
        self.clir_verifier(mmi_not_restricted, not_restricted)
        d.press.back()

        self.clir_verifier(mmi_enable_clir, enabled)
        d.press.back()

        Dialer.launch()
        self.clir_verifier(mmi_not_restricted, restricted)
        d.press.back()

        Dialer.voice_call("PHONE1", "PHONE2", True, True)
        dut_manager.activate_phone("PHONE1")
        self.assertTrue(Dialer.end_call(), "Can't find end call button")
        UiAutomatorUtils.close_all_tasks()

        Dialer.launch()
        self.clir_verifier(disable_clir_cmd, disable_clir_msg)
        d.press.back()

        self.clir_verifier(mmi_not_restricted, not_restricted)

    def test_temporary_clir(self):
        '''
        ST_TELEPHONY_SS_CLIR_003
        '''
        disable_clir_cmd = "#31#"
        tmp = self.phone2_number
        dut_manager.acs_config[u'PHONE2'][u'phoneNumber'] = "#31#" + self.phone2_number
        Dialer.launch()
        Dialer.voice_call("PHONE1", "PHONE2", True, True)
        dut_manager.activate_phone("PHONE1")
        self.assertTrue(Dialer.end_call(), "Can't find end call button")
        dut_manager.acs_config[u'PHONE2'][u'phoneNumber'] = tmp
        UiAutomatorUtils.close_all_tasks()
        Dialer.launch()
        Dialer.voice_call("PHONE1", "PHONE2", True)
        dut_manager.activate_phone("PHONE1")
        self.assertTrue(Dialer.end_call(), "Can't find end call button")

    def test_set_clir_from_settings(self):
        '''
        ST_TELEPHONY_SS_CLIR_002
        '''

        clir_modes = ["default operator settings", "Number hidden", "Number displayed"]
        # dut_manager.activate_phone("PHONE2")

        Dialer.launch()
        time.sleep(1)
        self.assertTrue(any(val in Dialer.get_caller_id_value() for val in clir_modes),
                        "CLIR status not detected from dialer settings")
        Dialer.set_caller_id("Network default")
        self.assertTrue("default operator settings" in Dialer.get_caller_id_value(), "Network default not set")
        UiAutomatorUtils.close_all_tasks()
        Dialer.launch()

        Dialer.voice_call("PHONE1", "PHONE2", True)
        dut_manager.activate_phone("PHONE1")
        self.assertTrue(Dialer.end_call(), "Can't find end call button")

        UiAutomatorUtils.close_all_tasks()
        Dialer.launch()
        Dialer.set_caller_id("Hide number")
        self.assertTrue("Number hidden" in Dialer.get_caller_id_value(), "Number hidden not set")
        UiAutomatorUtils.close_all_tasks()
        Dialer.launch()

        Dialer.voice_call("PHONE1", "PHONE2", True, True)
        dut_manager.activate_phone("PHONE1")
        self.assertTrue(Dialer.end_call(), "Can't find end call button")

        UiAutomatorUtils.close_all_tasks()
        Dialer.launch()
        Dialer.set_caller_id("Show number")
        self.assertTrue("Number displayed" in Dialer.get_caller_id_value(), "Show number not set")
        UiAutomatorUtils.close_all_tasks()
        Dialer.launch()

        Dialer.voice_call("PHONE1", "PHONE2", True)
        dut_manager.activate_phone("PHONE1")
        self.assertTrue(Dialer.end_call(), "Can't find end call button")

    def test_clip(self):
        '''
        ST_TELEPHOY_SS_CLIP_001
        '''
        clip_command = "*#30#"

        Messaging.launch()
        Messaging.remove_all_contacts()
        UiAutomatorUtils.close_all_tasks()

        Dialer.launch()
        Dialer.dial_number_using_keyboard(clip_command)
        time.sleep(7)
        clip_message = "Incoming Caller ID\nService was enabled."
        mmi_reply = UiAutomatorUtils.get_attr_content_matching_text(clip_message)
        self.assertTrue(mmi_reply is not None, "Did not detect mmi response for clip cmd")
        self.assertTrue("Service was enabled." in mmi_reply,
                        "Clip status reply doesn't contain the service enabled message. Reply: " + mmi_reply)
        if d(text=DIALER_CONFIRMATION).wait.exists(timeout=10000):
            d(text=DIALER_CONFIRMATION).click()

        Dialer.voice_call("PHONE2", "PHONE1", True)
        dut_manager.activate_phone("PHONE2")
        self.assertTrue(Dialer.end_call(), "Can't find end call button")