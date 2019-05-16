from PyUiApi.common.test_utils import *
from PyUiApi.app_utils.dialer_utils import Dialer
from PyUiApi.common.status_bar import StatusBar
from PyUiApi.multi_dut_support.dut_manager import *


class CallWaitTests(unittest.TestCase):
    TAG = "CallWaitTests"

    phone1 = 'PHONE1'
    phone2 = 'PHONE2'
    phone3 = 'PHONE3'
    phone1_number = dut_manager.acs_config[phone1].get('phoneNumber')
    phone2_number = dut_manager.acs_config[phone2].get('phoneNumber')
    phone3_number = dut_manager.acs_config[phone3].get('phoneNumber')
    is_call_active2 = False
    is_call_active3 = False
    is_ussd_open = False
    is_call_waiting_disabled = False
    activate_call_waiting_nr = '*43#'
    interrogate_call_waiting_nr = '*#43#'
    deactivate_call_waiting_nr = '#43#'

    def setUp(self):
        # Check TestBed requirements
        # These tests require at least 3 phones with active sim cards
        for phone_nr, phone in zip([self.phone1_number, self.phone2_number, self.phone3_number],
                                   [self.phone1, self.phone2, self.phone3]):
            self.assertIsNotNone(phone_nr, '%s should have an active sim card' % phone)

        self.initial_orientation = d.orientation

        for phone in [self.phone1, self.phone2, self.phone3]:
            LOG.info('Preparing setup for %s.' % phone)
            dut_manager.activate_phone(phone)
            UiAutomatorUtils.unlock_screen()
            UiAutomatorUtils.close_all_tasks()
            StatusBar.open_notifications()
            StatusBar.clear_notifications()
            d.press.home()

    def tearDown(self):
        if d.orientation != self.initial_orientation:
            d.orientation = self.initial_orientation
            d.freeze_rotation(False)

        if self.is_ussd_open:
            dut_manager.activate_phone(self.phone1)
            Dialer.close_ussd()
            self.is_ussd_open = False

        if self.is_call_active2:
            dut_manager.activate_phone(self.phone2)
            self.assertTrue(Dialer.end_call(), 'Failed to end call in cleanup.')
            self.is_call_active2 = False

        if self.is_call_active3:
            dut_manager.activate_phone(self.phone3)
            self.assertTrue(Dialer.end_call(), 'Failed to end call in cleanup.')
            self.is_call_active3 = False

        if self.is_call_waiting_disabled:
            dut_manager.activate_phone(self.phone1)
            Dialer.launch()
            self.assertTrue(Dialer.dialer_launched(), 'Dialer failed to launch.')
            self.assertTrue(Dialer.dial_number(self.activate_call_waiting_nr),
                            'Failed to activate call waiting using code (%s) in cleanup.'
                            % self.activate_call_waiting_nr)
            self.assertTrue(Dialer.is_ussd_open(), 'USSD popup not open.')
            self.assertTrue(Dialer.close_ussd(), 'Failed to close ussd popup.')
            self.is_call_waiting_disabled = False

        for phone in [self.phone1, self.phone2, self.phone3]:
            LOG.info('Cleaning up for %s.' % phone)
            dut_manager.activate_phone(phone)
            UiAutomatorUtils.close_all_tasks()
            StatusBar.open_notifications()
            StatusBar.clear_notifications()
            d.press.home()

    def test_call_wait_using_code_43(self):
        """ ST_TELEPHONY_SS_CW_001 """

        LOG.info('Activate %s' % self.phone1)
        dut_manager.activate_phone(self.phone1)

        LOG.info('Launch Dialer.')
        Dialer.launch()
        self.assertTrue(Dialer.dialer_launched(), 'Dialer failed to launch.')

        LOG.info('Activate call waiting using code (%s).' % self.activate_call_waiting_nr)
        self.assertTrue(Dialer.dial_number(self.activate_call_waiting_nr),
                        'Failed to activate call waiting using code (%s).'
                        % self.activate_call_waiting_nr)

        LOG.info('Check that USSD popup is open.')
        self.assertTrue(Dialer.is_ussd_open(), 'USSD popup not open.')
        self.is_ussd_open = True

        LOG.info('Get USSD message.')
        msg = Dialer.get_ussd_message()
        LOG.info('Found msg: %s' % msg)

        LOG.info('Close USSD popup.')
        self.assertTrue(Dialer.close_ussd(), 'Failed to close ussd popup.')
        self.is_ussd_open = False

        service_enabled_msg = 'Call waiting\nService was enabled.'
        LOG.info('Check that USSD msg indicate that service has been activated.')
        self.assertEqual(msg, service_enabled_msg, 'Found "%s" instead of "%s"' %
                         (msg, service_enabled_msg))

        LOG.info('Interrogate call waiting using code (%s).' % self.interrogate_call_waiting_nr)
        self.assertTrue(Dialer.dial_number(self.interrogate_call_waiting_nr),
                        'Failed to interrogate call waiting using code (%s).'
                        % self.interrogate_call_waiting_nr)

        LOG.info('Check that USSD popup is open.')
        self.assertTrue(Dialer.is_ussd_open(), 'USSD popup not open.')
        self.is_ussd_open = True

        LOG.info('Get USSD message.')
        msg = Dialer.get_ussd_message()
        LOG.info('Found msg: %s' % msg)

        LOG.info('Close USSD popup.')
        self.assertTrue(Dialer.close_ussd(), 'Failed to close ussd popup.')
        self.is_ussd_open = False

        service_interrogate_msg = 'Call waiting\nService was enabled for:\nVoice\nSync'
        LOG.info('Check that USSD msg indicate that service has been activated.')
        self.assertEqual(msg, service_interrogate_msg, 'Found "%s" instead of "%s"' %
                         (msg, service_interrogate_msg))

        LOG.info('Initiate a call from %s to %s.' % (self.phone1, self.phone2))
        self.assertTrue(Dialer.dial_number(self.phone2_number))

        LOG.info('Activate %s' % self.phone2)
        dut_manager.activate_phone(self.phone2)

        LOG.info('Answer incoming call.')
        Dialer.answer_call()  # I wish I could do an assert here
        self.is_call_active2 = True

        LOG.info('Activate %s' % self.phone1)
        dut_manager.activate_phone(self.phone1)

        LOG.info('Check if call is active with %s.' % self.phone2_number)
        self.assertTrue(Dialer.is_call_active(self.phone2_number), 'Call is inactive.')

        LOG.info('Activate %s' % self.phone3)
        dut_manager.activate_phone(self.phone3)

        LOG.info('Launch Dialer.')
        Dialer.launch()
        self.assertTrue(Dialer.dialer_launched(), 'Dialer failed to launch.')

        LOG.info('Initiate a call from %s to %s.' % (self.phone3, self.phone1))
        self.assertTrue(Dialer.dial_number(self.phone1_number), 'Failed to initiate call.')
        self.is_call_active3 = True

        LOG.info('Activate %s' % self.phone1)
        dut_manager.activate_phone(self.phone1)

        LOG.info('Check that there is an incoming call from %s' % self.phone3_number)
        self.assertTrue(Dialer.check_incoming_call(self.phone3_number),
                        'No incoming call from %s.' % self.phone3_number)

        LOG.info('Activate %s' % self.phone3)
        dut_manager.activate_phone(self.phone3)

        LOG.info('End call with %s.' % self.phone1_number)
        self.assertTrue(Dialer.end_call(), 'Failed to end call with %s.' % self.phone1_number)
        self.is_call_active3 = False

        LOG.info('Activate %s' % self.phone2)
        dut_manager.activate_phone(self.phone2)

        LOG.info('End call with %s.' % self.phone1_number)
        self.assertTrue(Dialer.end_call(), 'Failed to end call with %s.' % self.phone1_number)
        self.is_call_active2 = False

        LOG.info('Activate %s' % self.phone1)
        dut_manager.activate_phone(self.phone1)

        LOG.info('Launch Dialer.')
        Dialer.launch()
        self.assertTrue(Dialer.dialer_launched(), 'Dialer failed to launch.')

        LOG.info('Deactivate call waiting using code (%s).' % self.deactivate_call_waiting_nr)
        self.assertTrue(Dialer.dial_number(self.deactivate_call_waiting_nr),
                        'Failed to deactivate call waiting using code (%s).'
                        % self.deactivate_call_waiting_nr)
        self.is_call_waiting_disabled = True

        LOG.info('Check that USSD popup is open.')
        self.assertTrue(Dialer.is_ussd_open(), 'USSD popup not open.')
        self.is_ussd_open = True

        LOG.info('Get USSD message.')
        msg = Dialer.get_ussd_message()
        LOG.info('Found msg: %s' % msg)

        LOG.info('Close USSD popup.')
        self.assertTrue(Dialer.close_ussd(), 'Failed to close ussd popup.')
        self.is_ussd_open = False

        service_disabled_msg = 'Call waiting\nService has been disabled.'
        LOG.info('Check that USSD msg indicate that service has been disabled.')
        self.assertEqual(msg, service_disabled_msg, 'Found "%s" instead of "%s"' %
                         (msg, service_disabled_msg))

        LOG.info('Interrogate call waiting using code (%s).' % self.interrogate_call_waiting_nr)
        self.assertTrue(Dialer.dial_number(self.interrogate_call_waiting_nr),
                        'Failed to interrogate call waiting using code (%s).'
                        % self.interrogate_call_waiting_nr)

        LOG.info('Check that USSD popup is open.')
        self.assertTrue(Dialer.is_ussd_open(), 'USSD popup not open.')
        self.is_ussd_open = True

        LOG.info('Get USSD message.')
        msg = Dialer.get_ussd_message()
        LOG.info('Found msg: %s' % msg)

        LOG.info('Close USSD popup.')
        self.assertTrue(Dialer.close_ussd(), 'Failed to close ussd popup.')
        self.is_ussd_open = False

        service_interrogate_msg = service_disabled_msg
        LOG.info('Check that USSD msg indicate that service has been disabled.')
        self.assertEqual(msg, service_interrogate_msg, 'Found "%s" instead of "%s"' %
                         (msg, service_interrogate_msg))

        LOG.info('Initiate a call from %s to %s.' % (self.phone1, self.phone2))
        self.assertTrue(Dialer.dial_number(self.phone2_number))

        LOG.info('Activate %s' % self.phone2)
        dut_manager.activate_phone(self.phone2)

        LOG.info('Answer incoming call.')
        Dialer.answer_call()  # I wish I could do an assert here
        self.is_call_active2 = True

        LOG.info('Activate %s' % self.phone1)
        dut_manager.activate_phone(self.phone1)

        LOG.info('Check if call is active with %s.' % self.phone2_number)
        self.assertTrue(Dialer.is_call_active(self.phone2_number), 'Call is inactive.')

        LOG.info('Activate %s' % self.phone3)
        dut_manager.activate_phone(self.phone3)

        LOG.info('Launch Dialer.')
        Dialer.launch()
        self.assertTrue(Dialer.dialer_launched(), 'Dialer failed to launch.')

        LOG.info('Initiate a call from %s to %s.' % (self.phone3, self.phone1))
        self.assertTrue(Dialer.dial_number(self.phone1_number), 'Failed to initiate call.')
        self.is_call_active3 = True

        LOG.info('Activate %s' % self.phone1)
        dut_manager.activate_phone(self.phone1)

        LOG.info('Check that there is no incoming call from %s' % self.phone3_number)
        self.assertFalse(Dialer.check_incoming_call(self.phone3_number),
                         'Found incoming call from %s.' % self.phone3_number)

        LOG.info('Activate %s' % self.phone3)
        dut_manager.activate_phone(self.phone3)

        # If voice mail is active, call is redirected to it
        # Otherwise line will be busy
        LOG.info('Check if call is active with %s (redirected to voice mail).' % self.phone1_number)
        if Dialer.is_call_active(self.phone1_number):
            LOG.info('Call was redirected to voice mail.')

            LOG.info('End call with %s.' % self.phone1_number)
            self.assertTrue(Dialer.end_call(), 'Failed to end call with %s.' % self.phone1_number)
        else:
            LOG.info('Line was busy.')
        self.is_call_active3 = False

        LOG.info('Activate %s' % self.phone2)
        dut_manager.activate_phone(self.phone2)

        LOG.info('End call with %s.' % self.phone1_number)
        self.assertTrue(Dialer.end_call(), 'Failed to end call with %s.' % self.phone1_number)
        self.is_call_active2 = False


class CallWaitTests2(unittest.TestCase):
    TAG = "CallWaitTests2"

    phone1 = 'PHONE1'
    phone2 = 'PHONE2'
    phone3 = 'PHONE3'
    phone1_number = dut_manager.acs_config[phone1].get('phoneNumber')
    phone2_number = dut_manager.acs_config[phone2].get('phoneNumber')
    phone3_number = dut_manager.acs_config[phone3].get('phoneNumber')
    is_call_active2 = False
    is_call_active3 = False
    is_call_waiting_disabled = False

    def setUp(self):
        # Check TestBed requirements
        # These tests require at least 3 phones with active sim cards
        for phone_nr, phone in zip([self.phone1_number, self.phone2_number, self.phone3_number],
                                   [self.phone1, self.phone2, self.phone3]):
            self.assertIsNotNone(phone_nr, '%s should have an active sim card' % phone)

        self.initial_orientation = d.orientation

        for phone in [self.phone1, self.phone2, self.phone3]:
            LOG.info('Preparing setup for %s.' % phone)
            dut_manager.activate_phone(phone)
            UiAutomatorUtils.unlock_screen()
            UiAutomatorUtils.close_all_tasks()
            StatusBar.open_notifications()
            StatusBar.clear_notifications()
            d.press.home()

    def tearDown(self):
        if d.orientation != self.initial_orientation:
            d.orientation = self.initial_orientation
            d.freeze_rotation(False)

        if self.is_call_active2:
            dut_manager.activate_phone(self.phone2)
            self.assertTrue(Dialer.end_call(), 'Failed to end call in cleanup.')
            self.is_call_active2 = False

        if self.is_call_active3:
            dut_manager.activate_phone(self.phone3)
            self.assertTrue(Dialer.end_call(), 'Failed to end call in cleanup.')
            self.is_call_active3 = False

        if self.is_call_waiting_disabled:
            dut_manager.activate_phone(self.phone1)
            Dialer.launch()
            self.assertTrue(Dialer.dialer_launched(), 'Dialer failed to launch.')
            self.assertTrue(Dialer.enable_call_waiting(),
                            'Failed to activate call waiting using menu (cleanup).')
            self.is_call_waiting_disabled = False

        for phone in [self.phone1, self.phone2, self.phone3]:
            LOG.info('Cleaning up for %s.' % phone)
            dut_manager.activate_phone(phone)
            UiAutomatorUtils.close_all_tasks()
            StatusBar.open_notifications()
            StatusBar.clear_notifications()
            d.press.home()

    def test_call_wait_using_menu(self):
        """ ST_TELEPHONY_SS_CW_002 """

        LOG.info('Activate %s' % self.phone1)
        dut_manager.activate_phone(self.phone1)

        LOG.info('Launch Dialer.')
        Dialer.launch()
        self.assertTrue(Dialer.dialer_launched(), 'Dialer failed to launch.')

        LOG.info('Activate call waiting using menu.')
        self.assertTrue(Dialer.enable_call_waiting(),
                        'Failed to activate call waiting using menu.')

        LOG.info('Launch Dialer.')
        Dialer.launch()
        self.assertTrue(Dialer.dialer_launched(), 'Dialer failed to launch.')

        LOG.info('Initiate a call from %s to %s.' % (self.phone1, self.phone2))
        self.assertTrue(Dialer.dial_number(self.phone2_number))

        LOG.info('Activate %s' % self.phone2)
        dut_manager.activate_phone(self.phone2)

        LOG.info('Answer incoming call.')
        Dialer.answer_call()  # I wish I could do an assert here
        self.is_call_active2 = True

        LOG.info('Activate %s' % self.phone1)
        dut_manager.activate_phone(self.phone1)

        LOG.info('Check if call is active with %s.' % self.phone2_number)
        self.assertTrue(Dialer.is_call_active(self.phone2_number), 'Call is inactive.')

        LOG.info('Activate %s' % self.phone3)
        dut_manager.activate_phone(self.phone3)

        LOG.info('Launch Dialer.')
        Dialer.launch()
        self.assertTrue(Dialer.dialer_launched(), 'Dialer failed to launch.')

        LOG.info('Initiate a call from %s to %s.' % (self.phone3, self.phone1))
        self.assertTrue(Dialer.dial_number(self.phone1_number), 'Failed to initiate call.')
        self.is_call_active3 = True

        LOG.info('Activate %s' % self.phone1)
        dut_manager.activate_phone(self.phone1)

        LOG.info('Check that there is an incoming call from %s' % self.phone3_number)
        self.assertTrue(Dialer.check_incoming_call(self.phone3_number),
                        'No incoming call from %s.' % self.phone3_number)

        LOG.info('Activate %s' % self.phone3)
        dut_manager.activate_phone(self.phone3)

        LOG.info('End call with %s.' % self.phone1_number)
        self.assertTrue(Dialer.end_call(), 'Failed to end call with %s.' % self.phone1_number)
        self.is_call_active3 = False

        LOG.info('Activate %s' % self.phone2)
        dut_manager.activate_phone(self.phone2)

        LOG.info('End call with %s.' % self.phone1_number)
        self.assertTrue(Dialer.end_call(), 'Failed to end call with %s.' % self.phone1_number)
        self.is_call_active2 = False

        LOG.info('Activate %s' % self.phone1)
        dut_manager.activate_phone(self.phone1)

        LOG.info('Launch Dialer.')
        Dialer.launch()
        self.assertTrue(Dialer.dialer_launched(), 'Dialer failed to launch.')

        LOG.info('Deactivate call waiting using menu.')
        self.assertTrue(Dialer.disable_call_waiting(),
                        'Failed to deactivate call waiting using menu.')

        LOG.info('Launch Dialer.')
        Dialer.launch()
        self.assertTrue(Dialer.dialer_launched(), 'Dialer failed to launch.')
        self.is_call_waiting_disabled = True

        LOG.info('Initiate a call from %s to %s.' % (self.phone1, self.phone2))
        self.assertTrue(Dialer.dial_number(self.phone2_number))

        LOG.info('Activate %s' % self.phone2)
        dut_manager.activate_phone(self.phone2)

        LOG.info('Answer incoming call.')
        Dialer.answer_call()  # I wish I could do an assert here
        self.is_call_active2 = True

        LOG.info('Activate %s' % self.phone1)
        dut_manager.activate_phone(self.phone1)

        LOG.info('Check if call is active with %s.' % self.phone2_number)
        self.assertTrue(Dialer.is_call_active(self.phone2_number), 'Call is inactive.')

        LOG.info('Activate %s' % self.phone3)
        dut_manager.activate_phone(self.phone3)

        LOG.info('Launch Dialer.')
        Dialer.launch()
        self.assertTrue(Dialer.dialer_launched(), 'Dialer failed to launch.')

        LOG.info('Initiate a call from %s to %s.' % (self.phone3, self.phone1))
        self.assertTrue(Dialer.dial_number(self.phone1_number), 'Failed to initiate call.')
        self.is_call_active3 = True

        LOG.info('Activate %s' % self.phone1)
        dut_manager.activate_phone(self.phone1)

        LOG.info('Check that there is no incoming call from %s' % self.phone3_number)
        self.assertFalse(Dialer.check_incoming_call(self.phone3_number),
                         'Found incoming call from %s.' % self.phone3_number)

        LOG.info('Activate %s' % self.phone3)
        dut_manager.activate_phone(self.phone3)

        # If voice mail is active, call is redirected to it
        # Otherwise line will be busy
        LOG.info('Check if call is active with %s (redirected to voice mail).' % self.phone1_number)
        if Dialer.is_call_active(self.phone1_number):
            LOG.info('Call was redirected to voice mail.')

            LOG.info('End call with %s.' % self.phone1_number)
            self.assertTrue(Dialer.end_call(), 'Failed to end call with %s.' % self.phone1_number)
        else:
            LOG.info('Line was busy.')
        self.is_call_active3 = False

        LOG.info('Activate %s' % self.phone2)
        dut_manager.activate_phone(self.phone2)

        LOG.info('End call with %s.' % self.phone1_number)
        self.assertTrue(Dialer.end_call(), 'Failed to end call with %s.' % self.phone1_number)
        self.is_call_active2 = False

    def test_call_wait_incoming_call_ignored(self):
        """ ST_TELEPHONY_SS_CW_004 """

        LOG.info('Activate %s' % self.phone1)
        dut_manager.activate_phone(self.phone1)

        LOG.info('Launch Dialer.')
        Dialer.launch()
        self.assertTrue(Dialer.dialer_launched(), 'Dialer failed to launch.')

        LOG.info('Activate call waiting using menu.')
        self.assertTrue(Dialer.enable_call_waiting(),
                        'Failed to activate call waiting using menu.')

        LOG.info('Launch Dialer.')
        Dialer.launch()
        self.assertTrue(Dialer.dialer_launched(), 'Dialer failed to launch.')

        LOG.info('Initiate a call from %s to %s.' % (self.phone1, self.phone2))
        self.assertTrue(Dialer.dial_number(self.phone2_number))

        LOG.info('Activate %s' % self.phone2)
        dut_manager.activate_phone(self.phone2)

        LOG.info('Answer incoming call.')
        Dialer.answer_call()  # I wish I could do an assert here
        self.is_call_active2 = True

        LOG.info('Activate %s' % self.phone1)
        dut_manager.activate_phone(self.phone1)

        LOG.info('Check if call is active with %s.' % self.phone2_number)
        self.assertTrue(Dialer.is_call_active(self.phone2_number), 'Call is inactive.')

        LOG.info('Activate %s' % self.phone3)
        dut_manager.activate_phone(self.phone3)

        LOG.info('Launch Dialer.')
        Dialer.launch()
        self.assertTrue(Dialer.dialer_launched(), 'Dialer failed to launch.')

        LOG.info('Initiate a call from %s to %s.' % (self.phone3, self.phone1))
        self.assertTrue(Dialer.dial_number(self.phone1_number), 'Failed to initiate call.')
        self.is_call_active3 = True

        LOG.info('Activate %s' % self.phone1)
        dut_manager.activate_phone(self.phone1)

        LOG.info('Dismiss incoming call from %s.' % self.phone3_number)
        self.assertTrue(Dialer.dismiss_incoming_call(self.phone3_number),
                        'Failed to dismiss incoming call from %s' % self.phone3_number)

        LOG.info('Check that there is no indication of incoming call from %s' % self.phone3_number)
        self.assertFalse(Dialer.check_incoming_call(self.phone3_number),
                         'Still found indication of incoming call from %s.' % self.phone3_number)

        LOG.info('Check if call is still active with %s.' % self.phone2_number)
        self.assertTrue(Dialer.is_call_active(self.phone2_number), 'Call is inactive.')

        LOG.info('Activate %s' % self.phone3)
        dut_manager.activate_phone(self.phone3)

        # If voice mail is active, call is redirected to it
        # Otherwise line will be busy
        LOG.info('Check if call is active with %s (redirected to voice mail).' % self.phone1_number)
        if Dialer.is_call_active(self.phone1_number):
            LOG.info('Call was redirected to voice mail.')

            LOG.info('End call with %s.' % self.phone1_number)
            self.assertTrue(Dialer.end_call(), 'Failed to end call with %s.' % self.phone1_number)
        else:
            LOG.info('Line was busy.')
        self.is_call_active3 = False

        LOG.info('Activate %s' % self.phone2)
        dut_manager.activate_phone(self.phone2)

        LOG.info('End call with %s.' % self.phone1_number)
        self.assertTrue(Dialer.end_call(), 'Failed to end call with %s.' % self.phone1_number)
        self.is_call_active2 = False

    def test_call_wait_incoming_call_accepted(self):
        """ ST_TELEPHONY_SS_CW_003 """

        LOG.info('Activate %s' % self.phone1)
        dut_manager.activate_phone(self.phone1)

        LOG.info('Launch Dialer.')
        Dialer.launch()
        self.assertTrue(Dialer.dialer_launched(), 'Dialer failed to launch.')

        LOG.info('Activate call waiting using menu.')
        self.assertTrue(Dialer.enable_call_waiting(),
                        'Failed to activate call waiting using menu.')

        LOG.info('Launch Dialer.')
        Dialer.launch()
        self.assertTrue(Dialer.dialer_launched(), 'Dialer failed to launch.')

        LOG.info('Initiate a call from %s to %s.' % (self.phone1, self.phone2))
        self.assertTrue(Dialer.dial_number(self.phone2_number))

        LOG.info('Activate %s' % self.phone2)
        dut_manager.activate_phone(self.phone2)

        LOG.info('Answer incoming call.')
        Dialer.answer_call()  # I wish I could do an assert here
        self.is_call_active2 = True

        LOG.info('Activate %s' % self.phone1)
        dut_manager.activate_phone(self.phone1)

        LOG.info('Check if call is active with %s.' % self.phone2_number)
        self.assertTrue(Dialer.is_call_active(self.phone2_number), 'Call is inactive.')

        LOG.info('Activate %s' % self.phone3)
        dut_manager.activate_phone(self.phone3)

        LOG.info('Launch Dialer.')
        Dialer.launch()
        self.assertTrue(Dialer.dialer_launched(), 'Dialer failed to launch.')

        LOG.info('Initiate a call from %s to %s.' % (self.phone3, self.phone1))
        self.assertTrue(Dialer.dial_number(self.phone1_number), 'Failed to initiate call.')
        self.is_call_active3 = True

        LOG.info('Activate %s' % self.phone1)
        dut_manager.activate_phone(self.phone1)

        LOG.info('Accept incoming call from %s.' % self.phone3_number)
        self.assertTrue(Dialer.answer_incoming_call(self.phone3_number),
                        'Failed to accept incoming call from %s' % self.phone3_number)

        LOG.info('Check that call with %s is active.' % self.phone3_number)
        self.assertTrue(Dialer.is_call_active(self.phone3_number),
                        'Call with %s is inactive.' % self.phone3_number)

        LOG.info('Check that call with %s is on hold.' % self.phone2_number)
        self.assertTrue(Dialer.is_call_on_hold(self.phone2_number),
                        'Call with %s is not on hold.' % self.phone2_number)

        LOG.info('Activate %s' % self.phone3)
        dut_manager.activate_phone(self.phone3)

        LOG.info('End call with %s.' % self.phone1_number)
        self.assertTrue(Dialer.end_call(), 'Failed to end call with %s.' % self.phone1_number)
        self.is_call_active3 = False

        LOG.info('Activate %s' % self.phone1)
        dut_manager.activate_phone(self.phone1)

        LOG.info('Check that call with %s is still on hold.' % self.phone2_number)
        self.assertTrue(Dialer.is_call_on_hold(self.phone2_number),
                        'Call with %s is not on hold.' % self.phone2_number)

        LOG.info('Resume call with %s.' % self.phone2_number)
        self.assertTrue(Dialer.resume_call(), 'Failed to resume call.')

        LOG.info('End call with %s.' % self.phone2_number)
        self.assertTrue(Dialer.end_call(), 'Failed to end call with %s.' % self.phone2_number)
        self.is_call_active2 = False

    def test_call_wait_back_to_back_incoming_call_accepted(self):
        """ ST_TELEPHONY_SS_CW_008 """

        LOG.info('Activate %s' % self.phone1)
        dut_manager.activate_phone(self.phone1)

        LOG.info('Launch Dialer.')
        Dialer.launch()
        self.assertTrue(Dialer.dialer_launched(), 'Dialer failed to launch.')

        LOG.info('Activate call waiting using menu.')
        self.assertTrue(Dialer.enable_call_waiting(),
                        'Failed to activate call waiting using menu.')

        LOG.info('Launch Dialer.')
        Dialer.launch()
        self.assertTrue(Dialer.dialer_launched(), 'Dialer failed to launch.')

        LOG.info('Initiate a call from %s to %s.' % (self.phone1, self.phone2))
        self.assertTrue(Dialer.dial_number(self.phone2_number))

        LOG.info('Activate %s' % self.phone2)
        dut_manager.activate_phone(self.phone2)

        LOG.info('Answer incoming call.')
        Dialer.answer_call()  # I wish I could do an assert here
        self.is_call_active2 = True

        LOG.info('Activate %s' % self.phone1)
        dut_manager.activate_phone(self.phone1)

        for i in range(5):
            LOG.info('Iteration: %s' % i)

            LOG.info('Check if call is active with %s.' % self.phone2_number)
            self.assertTrue(Dialer.is_call_active(self.phone2_number), 'Call is inactive.')

            LOG.info('Activate %s' % self.phone3)
            dut_manager.activate_phone(self.phone3)

            LOG.info('Launch Dialer.')
            Dialer.launch()
            self.assertTrue(Dialer.dialer_launched(), 'Dialer failed to launch.')

            LOG.info('Initiate a call from %s to %s.' % (self.phone3, self.phone1))
            self.assertTrue(Dialer.dial_number(self.phone1_number), 'Failed to initiate call.')
            self.is_call_active3 = True

            LOG.info('Activate %s' % self.phone1)
            dut_manager.activate_phone(self.phone1)

            LOG.info('Accept incoming call from %s.' % self.phone3_number)
            self.assertTrue(Dialer.answer_incoming_call(self.phone3_number),
                            'Failed to accept incoming call from %s' % self.phone3_number)

            LOG.info('Check that call with %s is active.' % self.phone3_number)
            self.assertTrue(Dialer.is_call_active(self.phone3_number),
                            'Call with %s is inactive.' % self.phone3_number)

            LOG.info('Check that call with %s is on hold.' % self.phone2_number)
            self.assertTrue(Dialer.is_call_on_hold(self.phone2_number),
                            'Call with %s is not on hold.' % self.phone2_number)

            LOG.info('Activate %s' % self.phone3)
            dut_manager.activate_phone(self.phone3)

            LOG.info('End call with %s.' % self.phone1_number)
            self.assertTrue(Dialer.end_call(), 'Failed to end call with %s.' % self.phone1_number)
            self.is_call_active3 = False

            LOG.info('Activate %s' % self.phone1)
            dut_manager.activate_phone(self.phone1)

            LOG.info('Check that call with %s is still on hold.' % self.phone2_number)
            self.assertTrue(Dialer.is_call_on_hold(self.phone2_number),
                            'Call with %s is not on hold.' % self.phone2_number)

            LOG.info('Resume call with %s.' % self.phone2_number)
            self.assertTrue(Dialer.resume_call(), 'Failed to resume call.')

        LOG.info('End call with %s.' % self.phone2_number)
        self.assertTrue(Dialer.end_call(), 'Failed to end call with %s.' % self.phone2_number)
        self.is_call_active2 = False


class CallWaitTests3(unittest.TestCase):
    TAG = "CallWaitTests3"

    phone1 = 'PHONE1'
    phone2 = 'PHONE2'
    phone3 = 'PHONE3'
    phone4 = 'PHONE4'
    phone1_number = dut_manager.acs_config[phone1].get('phoneNumber')
    phone2_number = dut_manager.acs_config[phone2].get('phoneNumber')
    phone3_number = dut_manager.acs_config[phone3].get('phoneNumber')
    phone4_number = dut_manager.acs_config[phone4].get('phoneNumber')
    is_call_active2 = False
    is_call_active3 = False
    is_call_active4 = False

    def setUp(self):
        # Check TestBed requirements
        # These tests require at least 4 phones with active sim cards
        for phone_nr, phone in zip([self.phone1_number, self.phone2_number,
                                    self.phone3_number, self.phone4_number],
                                   [self.phone1, self.phone2,
                                    self.phone3, self.phone4]):
            self.assertIsNotNone(phone_nr, '%s should have an active sim card.' % phone)

        self.initial_orientation = d.orientation

        for phone in [self.phone1, self.phone2, self.phone3, self.phone4]:
            LOG.info('Preparing setup for %s.' % phone)
            dut_manager.activate_phone(phone)
            UiAutomatorUtils.unlock_screen()
            UiAutomatorUtils.close_all_tasks()
            StatusBar.open_notifications()
            StatusBar.clear_notifications()
            d.press.home()

    def tearDown(self):
        if d.orientation != self.initial_orientation:
            d.orientation = self.initial_orientation
            d.freeze_rotation(False)

        if self.is_call_active2:
            dut_manager.activate_phone(self.phone2)
            self.assertTrue(Dialer.end_call(), 'Failed to end call in cleanup.')
            self.is_call_active2 = False

        if self.is_call_active3:
            dut_manager.activate_phone(self.phone3)
            self.assertTrue(Dialer.end_call(), 'Failed to end call in cleanup.')
            self.is_call_active3 = False

        if self.is_call_active4:
            dut_manager.activate_phone(self.phone4)
            self.assertTrue(Dialer.end_call(), 'Failed to end call in cleanup.')
            self.is_call_active3 = False

        for phone in [self.phone1, self.phone2, self.phone3, self.phone4]:
            LOG.info('Cleaning up for %s.' % phone)
            dut_manager.activate_phone(phone)
            UiAutomatorUtils.close_all_tasks()
            StatusBar.open_notifications()
            StatusBar.clear_notifications()
            d.press.home()

    def test_call_wait_third_call_management(self):
        """ ST_TELEPHONY_SS_CW_009 """

        LOG.info('Activate %s' % self.phone1)
        dut_manager.activate_phone(self.phone1)

        LOG.info('Launch Dialer.')
        Dialer.launch()
        self.assertTrue(Dialer.dialer_launched(), 'Dialer failed to launch.')

        LOG.info('Activate call waiting using menu.')
        self.assertTrue(Dialer.enable_call_waiting(),
                        'Failed to activate call waiting using menu.')

        LOG.info('Launch Dialer.')
        Dialer.launch()
        self.assertTrue(Dialer.dialer_launched(), 'Dialer failed to launch.')

        LOG.info('Initiate a call from %s to %s.' % (self.phone1, self.phone2))
        self.assertTrue(Dialer.dial_number(self.phone2_number))

        LOG.info('Activate %s' % self.phone2)
        dut_manager.activate_phone(self.phone2)

        LOG.info('Answer incoming call.')
        Dialer.answer_call()  # I wish I could do an assert here
        self.is_call_active2 = True

        LOG.info('Activate %s' % self.phone1)
        dut_manager.activate_phone(self.phone1)

        LOG.info('Check if call is active with %s.' % self.phone2_number)
        self.assertTrue(Dialer.is_call_active(self.phone2_number), 'Call is inactive.')

        LOG.info('Activate %s' % self.phone3)
        dut_manager.activate_phone(self.phone3)

        LOG.info('Launch Dialer.')
        Dialer.launch()
        self.assertTrue(Dialer.dialer_launched(), 'Dialer failed to launch.')

        LOG.info('Initiate a call from %s to %s.' % (self.phone3, self.phone1))
        self.assertTrue(Dialer.dial_number(self.phone1_number), 'Failed to initiate call.')
        self.is_call_active3 = True

        LOG.info('Activate %s' % self.phone1)
        dut_manager.activate_phone(self.phone1)

        LOG.info('Accept incoming call from %s.' % self.phone3_number)
        self.assertTrue(Dialer.answer_incoming_call(self.phone3_number),
                        'Failed to accept incoming call from %s' % self.phone3_number)

        LOG.info('Check that call with %s is active.' % self.phone3_number)
        self.assertTrue(Dialer.is_call_active(self.phone3_number),
                        'Call with %s is inactive.' % self.phone3_number)

        LOG.info('Check that call with %s is on hold.' % self.phone2_number)
        self.assertTrue(Dialer.is_call_on_hold(self.phone2_number),
                        'Call with %s is not on hold.' % self.phone2_number)

        LOG.info('Activate %s' % self.phone4)
        dut_manager.activate_phone(self.phone4)

        LOG.info('Launch Dialer.')
        Dialer.launch()
        self.assertTrue(Dialer.dialer_launched(), 'Dialer failed to launch.')

        LOG.info('Initiate a call from %s to %s.' % (self.phone4, self.phone1))
        self.assertTrue(Dialer.dial_number(self.phone1_number), 'Failed to initiate call.')
        self.is_call_active4 = True

        LOG.info('Activate %s' % self.phone1)
        dut_manager.activate_phone(self.phone1)

        # Dismissing will redirect to voice mail
        LOG.info('Dismiss incoming call from %s.' % self.phone4_number)
        self.assertTrue(Dialer.dismiss_incoming_call(self.phone4_number),
                        'Failed to dismiss incoming call from %s' % self.phone4_number)

        LOG.info('Check that call with %s is active.' % self.phone3_number)
        self.assertTrue(Dialer.is_call_active(self.phone3_number),
                        'Call with %s is inactive.' % self.phone3_number)

        LOG.info('Check that call with %s is on hold.' % self.phone2_number)
        self.assertTrue(Dialer.is_call_on_hold(self.phone2_number),
                        'Call with %s is not on hold.' % self.phone2_number)

        LOG.info('Activate %s' % self.phone4)
        dut_manager.activate_phone(self.phone4)

        # If voice mail is active, call is redirected to it
        # Otherwise line will be busy
        LOG.info('Check if call is active with %s (redirected to voice mail).' % self.phone1_number)
        if Dialer.is_call_active(self.phone1_number):
            LOG.info('Call was redirected to voice mail.')

            LOG.info('End call with %s.' % self.phone1_number)
            self.assertTrue(Dialer.end_call(), 'Failed to end call with %s.' % self.phone1_number)
        else:
            LOG.info('Line was busy.')
        self.is_call_active4 = False

        LOG.info('Activate %s' % self.phone1)
        dut_manager.activate_phone(self.phone1)

        LOG.info('A message is sent to PHONE1 (missed calls). Will close the notifications.')
        StatusBar.open_notifications()
        self.assertTrue(StatusBar.close_notifications(), 'Failed to close notification.')

        LOG.info('Activate %s' % self.phone4)
        dut_manager.activate_phone(self.phone4)

        LOG.info('Launch Dialer.')
        Dialer.launch()
        self.assertTrue(Dialer.dialer_launched(), 'Dialer failed to launch.')

        LOG.info('Initiate a call from %s to %s.' % (self.phone4, self.phone1))
        self.assertTrue(Dialer.dial_number(self.phone1_number), 'Failed to initiate call.')
        self.is_call_active4 = True

        LOG.info('Activate %s' % self.phone1)
        dut_manager.activate_phone(self.phone1)

        LOG.info('Accept incoming call from %s.' % self.phone4_number)
        # This will end call with PHONE2 and leave PHONE4 still dialing
        self.assertFalse(Dialer.answer_incoming_call(self.phone4_number),
                         'Answered incoming call from %s from the first try.' % self.phone4_number)
        self.is_call_active2 = False
        # This will actually answer incoming call from PHONE4
        self.assertTrue(Dialer.answer_incoming_call(self.phone4_number),
                        'Failed to accept incoming call from %s' % self.phone4_number)

        LOG.info('Check that call with %s is active.' % self.phone4_number)
        self.assertTrue(Dialer.is_call_active(self.phone4_number),
                        'Call with %s is inactive.' % self.phone4_number)

        LOG.info('Check that call with %s is on hold.' % self.phone3_number)
        self.assertTrue(Dialer.is_call_on_hold(self.phone3_number),
                        'Call with %s is not on hold.' % self.phone3_number)

        LOG.info('Activate %s' % self.phone3)
        dut_manager.activate_phone(self.phone3)

        LOG.info('End call with %s.' % self.phone1_number)
        self.assertTrue(Dialer.end_call(), 'Failed to end call with %s.' % self.phone1_number)
        self.is_call_active3 = False

        LOG.info('Activate %s' % self.phone4)
        dut_manager.activate_phone(self.phone4)

        LOG.info('End call with %s.' % self.phone1_number)
        self.assertTrue(Dialer.end_call(), 'Failed to end call with %s.' % self.phone1_number)
        self.is_call_active4 = False
