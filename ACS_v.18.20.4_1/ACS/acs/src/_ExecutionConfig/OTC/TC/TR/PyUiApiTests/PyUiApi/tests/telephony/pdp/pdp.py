from PyUiApi.common.system_utils import *
from PyUiApi.common.environment_utils import *
from PyUiApi.common.uiautomator_utils import *
from PyUiApi.common.screenshot_utils import *
from PyUiApi.common.telephony_utils import *
from PyUiApi.dut_info.dut_info import *
from PyUiApi.app_utils.settings_utils import *
from PyUiApi.app_utils.dialer_utils import *

from PyUiApi.multi_dut_support.dut_manager import *

WAIT_FOR_300s = 300


class PDPTests(unittest.TestCase):
    TAG = "PDPTests"

    def setUp(self):
        UiAutomatorUtils.unlock_screen()
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

    def test_call_after_reboot_pdp_deactivated_2G(self):
        """
        ST_TELEPHONY_LIVE_2G_IDLE_001
        """
        Settings.change_mobile_network("2G")

        TelephonyUtils.disable_mobile_data()

        connected = True
        for i in range(10):
            time.sleep(2)
            if TelephonyUtils.get_data_connection_state() != "2":
                connected = False
                break

        self.assertFalse(connected, "Data is still connected, but should not")

        UiAutomatorUtils.reboot_device()

        good_service = False
        for i in range(30):
            time.sleep(2)
            if TelephonyUtils.get_connected_service() == "2G":
                good_service = True
                break
        self.assertTrue(good_service, "Device has not attached to 2G network")
        dut_manager.activate_phone("PHONE1")
        d.dump()

        time.sleep(WAIT_FOR_300s)

        Dialer.voice_call(phone_originating="PHONE2", phone_receiving="PHONE1",
                        check_number=True, screen_locked=True)
        StatusBar.answer_phone()
        time.sleep(2)
        StatusBar.clear_notifications()
        self.assertTrue(Dialer.end_call(), "Can't find end call button")

    def test_call_after_reboot_pdp_activated_2G(self):
        """
        ST_TELEPHONY_LIVE_2G_IDLE_003
        """
        Settings.change_mobile_network("2G")

        if TelephonyUtils.get_data_connection_state() != "2":
            TelephonyUtils.enable_mobile_data()

        connected = False
        for i in range(10):
            time.sleep(2)
            if TelephonyUtils.get_data_connection_state() == "2":
                connected = True
                break

        self.assertTrue(connected, "Data is not connected")

        UiAutomatorUtils.reboot_device()

        good_service = False
        for i in range(30):
            time.sleep(2)
            if TelephonyUtils.get_connected_service() == "2G":
                good_service = True
                break

        self.assertTrue(good_service, "Device has not attached to 2G network")
        dut_manager.activate_phone("PHONE1")
        d.dump()

        time.sleep(WAIT_FOR_300s)

        Dialer.voice_call(phone_originating="PHONE2", phone_receiving="PHONE1",
                        check_number=True, screen_locked=True)
        StatusBar.answer_phone()
        time.sleep(2)
        StatusBar.clear_notifications()
        self.assertTrue(Dialer.end_call(), "Can't find end call button")

    def test_call_after_reboot_pdp_deactivated_3G(self):
        """
        ST_TELEPHONY_LIVE_3G_IDLE_001
        """
        Settings.change_mobile_network("3G")

        TelephonyUtils.disable_mobile_data()

        connected = True
        for i in range(10):
            time.sleep(2)
            if TelephonyUtils.get_data_connection_state() != "2":
                connected = False
                break
        self.assertFalse(connected, "Data is still connected, but should not")

        UiAutomatorUtils.reboot_device()

        good_service = False
        for i in range(30):
            time.sleep(2)
            if TelephonyUtils.get_connected_service() == "3G":
                good_service = True
                break
        self.assertTrue(good_service, "Device has not attached to 3G network")
        dut_manager.activate_phone("PHONE1")
        d.dump()

        time.sleep(WAIT_FOR_300s)

        Dialer.voice_call(phone_originating="PHONE2", phone_receiving="PHONE1",
                        check_number=True, screen_locked=True)
        StatusBar.answer_phone()
        time.sleep(2)
        StatusBar.clear_notifications()
        self.assertTrue(Dialer.end_call(), "Can't find end call button")

    def test_call_after_reboot_pdp_activated_3G(self):
        """
        ST_TELEPHONY_LIVE_3G_IDLE_003
        """
        Settings.change_mobile_network("3G")

        if TelephonyUtils.get_data_connection_state() != "2":
            TelephonyUtils.enable_mobile_data()

        connected = False
        for i in range(10):
            time.sleep(2)
            if TelephonyUtils.get_data_connection_state() == "2":
                connected = True
                break
        self.assertTrue(connected, "Data is not connected")

        UiAutomatorUtils.reboot_device()

        good_service = False
        for i in range(30):
            time.sleep(2)
            if TelephonyUtils.get_connected_service() == "3G":
                good_service = True
                break
        self.assertTrue(good_service, "Device has not attached to 3G network")
        dut_manager.activate_phone("PHONE1")
        d.dump()

        time.sleep(WAIT_FOR_300s)

        Dialer.voice_call(phone_originating="PHONE2", phone_receiving="PHONE1",
                        check_number=True, screen_locked=True)
        StatusBar.answer_phone()
        time.sleep(2)
        StatusBar.clear_notifications()
        self.assertTrue(Dialer.end_call(), "Can't find end call button")
