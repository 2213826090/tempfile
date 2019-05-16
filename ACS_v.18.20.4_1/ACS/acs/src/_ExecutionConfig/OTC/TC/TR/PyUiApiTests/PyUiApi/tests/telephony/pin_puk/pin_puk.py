from PyUiApi.common.system_utils import *
from PyUiApi.common.environment_utils import *
from PyUiApi.common.uiautomator_utils import *
from PyUiApi.common.screenshot_utils import *
from PyUiApi.common.telephony_utils import *
from PyUiApi.dut_info.dut_info import *
from PyUiApi.app_utils.settings_utils import *
from PyUiApi.app_utils.dialer_utils import *

from PyUiApi.multi_dut_support.dut_manager import *


class PINPUKTests(unittest.TestCase):
    TAG = "PINPUKTests"
    REBOOT_ACTIVATE_FDN = False

    def setUp(self):
        PINPUKTests.REBOOT_ACTIVATE_FDN = False

        UiAutomatorUtils.unlock_screen()
        self.initial_orientation = d.orientation
        self.screenshooter = ScreenshotUtils()

        try:
            self.phone_pin_2 = dut_manager.acs_config[u'PHONE1'][u'PIN_2']
            self.phone_puk_2 = dut_manager.acs_config[u'PHONE1'][u'PUK_2']
        except:
            LOG.info("Device's PIN 2 or PUK 2 are not defined in bench config")

        self.assertTrue(Dialer.activate_fdn(self.phone_pin_2),
                        "FDN couldn't be activated")

        UiAutomatorUtils.reboot_device()

    def tearDown(self):
        self.log_before_cleanup()
        UiAutomatorUtils.close_all_tasks()

        if d.orientation != self.initial_orientation:
            d.orientation = self.initial_orientation
            d.freeze_rotation(False)

        if PINPUKTests.REBOOT_ACTIVATE_FDN:
            UiAutomatorUtils.reboot_device()
            self.assertTrue(Dialer.deactivate_fdn(self.phone_pin_2),
                            "FDN couldn't be deactivated")
        UiAutomatorUtils.close_all_tasks()

    def log_before_cleanup(self):
        result = self._resultForDoCleanups
        if not result.wasSuccessful():
            TestUtils.log_screenshot_and_dump()

    def test_unlock_pin_2_using_puk_2(self):
        """
        ST_TELEPHONY_SEC_327
        Unlocking SIM PIN 2 entering the PUK 2
        It requires the FDN option enabled; then, the option is blocked by
        incorrectly entering PIN2 3 times; PUK2 is entered successfully
        """

        self.assertTrue(Dialer.block_pin_2_change_using_puk_2(self.phone_pin_2,
                                                              self.phone_puk_2),
                        "PIN2 couldn't be blocked and changed through PUk2")

        self.assertTrue(TelephonyUtils.is_network_camped(),
                        "DUT didn't camp into a network")

    def test_unlock_pin_2_using_puk_2_MMI(self):
        """
        ST_TELEPHONY_SEC_328
        Unlocking SIM PIN 2 entering the PUK2 through MMI code.
        FDN enabled; Block it by entering PIN2 incorrectly 3 times;
        Enter PUK2 using MMI (**052*PIN2_UNBLOCKING_KEY*NEW_PIN2*NEW_PIN2#);
        Check if FDN is unblocked and DUT is camped in a network
        """
        PINPUKTests.REBOOT_ACTIVATE_FDN = True

        self.assertTrue(Dialer.navigate_to_fdn(), "Could not go to FDN menu")
        self.assertTrue(Dialer.block_fdn(),
                        "FDN couldn't be blocked by incorrect PIN2")

        self.assertTrue(Dialer.change_pin_2_using_puk_2_mmi(self.phone_pin_2,
                                                            self.phone_puk_2),
                        "PIN2 couldn't be changed through MMI code")

        self.assertTrue(TelephonyUtils.is_network_camped(),
                        "DUT didn't camp into a network")
