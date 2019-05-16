from PyUiApi.common.test_utils import *
from PyUiApi.app_utils.dialer_utils import Dialer
from PyUiApi.common.status_bar import StatusBar
from PyUiApi.multi_dut_support.dut_manager import *


class CallHoldTests(unittest.TestCase):
    TAG = "CallHoldTests"

    phone1 = 'PHONE1'
    phone2 = 'PHONE2'
    phone1_number = dut_manager.acs_config[phone1].get('phoneNumber')
    phone2_number = dut_manager.acs_config[phone2].get('phoneNumber')
    is_call_active = False

    def setUp(self):
        # Check TestBed requirements
        # These tests require at least 2 phones with active sim cards
        for phone_nr, phone in zip([self.phone1_number, self.phone2_number],
                                   [self.phone1, self.phone2]):
            self.assertIsNotNone(phone_nr, '%s should have an active sim card' % phone)

        UiAutomatorUtils.unlock_screen()
        self.initial_orientation = d.orientation

        for phone in [self.phone1, self.phone2]:
            dut_manager.activate_phone(phone)
            UiAutomatorUtils.close_all_tasks()
            StatusBar.open_notifications()
            StatusBar.clear_notifications()
            d.press.home()

    def tearDown(self):
        if d.orientation != self.initial_orientation:
            d.orientation = self.initial_orientation
            d.freeze_rotation(False)

        if self.is_call_active:
            dut_manager.activate_phone(self.phone1)
            self.assertTrue(Dialer.end_call(), 'Failed to end call in cleanup.')
            self.is_call_active = False

    def test_call_hold_unhold(self):
        """ ST_TELEPHONY_SS_CH_001 """

        LOG.info('Activate %s' % self.phone1)
        dut_manager.activate_phone(self.phone1)

        LOG.info('Launch Dialer.')
        Dialer.launch()
        self.assertTrue(Dialer.dialer_launched(), 'Dialer failed to launch.')

        LOG.info('Initiate a call from %s to %s.' % (self.phone1, self.phone2))
        self.assertTrue(Dialer.dial_number(self.phone2_number), 'Failed to initiate call.')
        self.is_call_active = True

        LOG.info('Activate %s' % self.phone2)
        dut_manager.activate_phone(self.phone2)

        LOG.info('Answer incoming call.')
        Dialer.answer_call()  # I wish I could do an assert here

        LOG.info('Activate %s' % self.phone1)
        dut_manager.activate_phone(self.phone1)

        LOG.info('Check if call is active.')
        self.assertTrue(Dialer.is_call_active(), 'Call is inactive.')

        LOG.info('Put call on hold.')
        self.assertTrue(Dialer.hold_call(), 'Failed to put call on hold.')

        LOG.info('Check that call is on hold.')
        self.assertTrue(Dialer.is_call_on_hold(), 'Call is not on hold.')

        LOG.info('Resume call.')
        self.assertTrue(Dialer.resume_call(), 'Failed to resume call.')

        LOG.info('Check that call is not on hold anymore.')
        self.assertFalse(Dialer.is_call_on_hold(), 'Call is still on hold.')

        LOG.info('End call.')
        self.assertTrue(Dialer.end_call(), 'Failed to end call.')
        self.is_call_active = False

    def test_back_to_back_voice_call_hold_unhold(self):
        """ ST_TELEPHONY_SS_CH_005 """

        LOG.info('Activate %s' % self.phone1)
        dut_manager.activate_phone(self.phone1)

        LOG.info('Launch Dialer.')
        Dialer.launch()
        self.assertTrue(Dialer.dialer_launched(), 'Dialer failed to launch.')

        LOG.info('Initiate a call from %s to %s.' % (self.phone1, self.phone2))
        self.assertTrue(Dialer.dial_number(self.phone2_number), 'Failed to initiate call.')
        self.is_call_active = True

        LOG.info('Activate %s' % self.phone2)
        dut_manager.activate_phone(self.phone2)

        LOG.info('Answer incoming call.')
        Dialer.answer_call()  # I wish I could do an assert here

        LOG.info('Activate %s' % self.phone1)
        dut_manager.activate_phone(self.phone1)

        LOG.info('Check if call is active.')
        self.assertTrue(Dialer.is_call_active(), 'Call is inactive.')

        LOG.info('Put call hold/unhold 100 times.')
        for i in xrange(100):
            LOG.info('Iteration #%s' % i)

            LOG.info('Put call on hold.')
            self.assertTrue(Dialer.hold_call(), 'Failed to put call on hold.')

            LOG.info('Check that call is on hold.')
            self.assertTrue(Dialer.is_call_on_hold(), 'Call is not on hold.')

            LOG.info('Resume call.')
            self.assertTrue(Dialer.resume_call(), 'Failed to resume call.')

            LOG.info('Check that call is not on hold anymore.')
            self.assertFalse(Dialer.is_call_on_hold(), 'Call is still on hold.')

        LOG.info('End call.')
        self.assertTrue(Dialer.end_call(), 'Failed to end call.')
        self.is_call_active = False


class CallHoldTests2(unittest.TestCase):
    TAG = "CallHoldTests2"

    phone1 = 'PHONE1'
    phone2 = 'PHONE2'
    phone3 = 'PHONE3'
    phone1_number = dut_manager.acs_config[phone1].get('phoneNumber')
    phone2_number = dut_manager.acs_config[phone2].get('phoneNumber')
    phone3_number = dut_manager.acs_config[phone3].get('phoneNumber')
    is_call_active2 = False
    is_call_active3 = False

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

        for phone in [self.phone1, self.phone2, self.phone3]:
            LOG.info('Cleaning up for %s.' % phone)
            dut_manager.activate_phone(phone)
            UiAutomatorUtils.close_all_tasks()
            StatusBar.open_notifications()
            StatusBar.clear_notifications()
            d.press.home()

    def test_resume_call_in_hold_after_releasing_active_call(self):
        """ ST_TELEPHONY_SS_CH_002 """

        LOG.info('Activate %s' % self.phone1)
        dut_manager.activate_phone(self.phone1)

        LOG.info('Launch Dialer.')
        Dialer.launch()
        self.assertTrue(Dialer.dialer_launched(), 'Dialer failed to launch.')

        LOG.info('Initiate a call from %s to %s.' % (self.phone1, self.phone2))
        self.assertTrue(Dialer.dial_number(self.phone2_number), 'Failed to initiate call.')

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

        LOG.info('Answer incoming call from %s.' % self.phone3_number)
        self.assertTrue(Dialer.answer_incoming_call(self.phone3_number),
                        'Failed to answer incoming call from %s' % self.phone3_number)

        LOG.info('Check that call with %s is active.' % self.phone3_number)
        self.assertTrue(Dialer.is_call_active(self.phone3_number),
                        'Call with %s is inactive.' % self.phone3_number)

        LOG.info('Check that call with %s is on hold.' % self.phone2_number)
        self.assertTrue(Dialer.is_call_on_hold(self.phone2_number),
                        'Call with %s is not on hold.' % self.phone2_number)

        LOG.info('End call with %s.' % self.phone3_number)
        self.assertTrue(Dialer.end_call(), 'Failed to end call with %s.' % self.phone3_number)
        self.is_call_active3 = False

        LOG.info('Check that call with %s is unheld and turned active.' % self.phone2_number)
        self.assertTrue(Dialer.is_call_active(self.phone2_number))

        LOG.info('End call with %s.' % self.phone2_number)
        self.assertTrue(Dialer.end_call(), 'Failed to end call with %s.' % self.phone2_number)
        self.is_call_active2 = False





