from PyUiApi.common.system_utils import *
from PyUiApi.common.uiautomator_utils import *
from PyUiApi.common.screenshot_utils import *
from PyUiApi.common.telephony_utils import *
from PyUiApi.app_utils.settings_utils import *
from PyUiApi.app_utils.dialer_utils import *

from PyUiApi.multi_dut_support.dut_manager import *


class MailboxTests(unittest.TestCase):
    TAG = "TelephonyTests"

    def setUp(self):
        # UiAutomatorUtils.unlock_screen()
        UiAutomatorUtils.close_all_tasks()
        self.initial_orientation = d.orientation
        self.screenshooter = ScreenshotUtils()

        # should make a better mechanism for phone devices verification
        try:
            self.phone1_number = dut_manager.acs_config[u'PHONE1'][u'phoneNumber']
            self.phone2_number = dut_manager.acs_config[u'PHONE2'][u'phoneNumber']
        except:
            LOG.info("Phone numbers for devices are not defined in bench config")

    def tearDown(self):
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

    def log_before_cleanup(self):
        result = self._resultForDoCleanups
        if not result.wasSuccessful():
            TestUtils.log_screenshot_and_dump()

    def test_read_mailbox_number(self):
        '''
        ST_TELEPHONY_REG_MBOX_001
        '''
        default_number_value = "<Not set>"
        text = None
        Dialer.launch()
        Dialer.navigate_to_call_settings()
        if d(text=DIALER_VOICEMAIL_TXT).wait.exists(timeout=5000):
            d(text=DIALER_VOICEMAIL_TXT).click()
        if d(text=DIALER_SETUP_TXT).wait.exists(timeout=5000):
            d(text=DIALER_SETUP_TXT).click()
        if d(resourceId=DIALER_SUMMARY).wait.exists(timeout=5000):
            text = d(resourceId=DIALER_SUMMARY).text
        self.assertTrue(text == default_number_value,
                        "Voicemail default number not found. Searched/obtained: " + default_number_value + "/" + text)

    def test_change_mailbox_number(self):
        '''
        ST_TELEPHONY_REG_MBOX_002
        '''
        text = ""
        is_valid = False

        Dialer.launch()
        Dialer.set_voicemail_number(self.phone1_number)

        UiAutomatorUtils.reboot_device()
        SystemUtils.open_homescreen_settings()

        Dialer.launch()
        Dialer.navigate_to_call_settings()
        if d(text=DIALER_VOICEMAIL_TXT).wait.exists(timeout=5000):
            d(text=DIALER_VOICEMAIL_TXT).click()
        if d(text=DIALER_SETUP_TXT).wait.exists(timeout=5000):
            d(text=DIALER_SETUP_TXT).click()
        d(resourceId="Voicemail number").wait.exists(timeout=5000)
        if d(resourceId=DIALER_SUMMARY).wait.exists(timeout=5000):
            text = d(resourceId=DIALER_SUMMARY).text
        if text == self.phone1_number:
            is_valid = True
        Dialer.set_voicemail_number("")

        self.assertTrue(is_valid is True,
                        "Voicemail number does not corespond with value changed. Searched/obtained: " + self.phone1_number + "/" + text)