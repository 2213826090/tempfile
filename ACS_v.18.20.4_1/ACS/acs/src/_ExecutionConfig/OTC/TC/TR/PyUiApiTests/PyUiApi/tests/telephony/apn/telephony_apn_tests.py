from PyUiApi.common.telephony_utils import *
from PyUiApi.tests.chrome_tests import *
from PyUiApi.app_utils.chrome_utils import *


class TelephonyApnTests(unittest.TestCase):

    data_enable_cmd = "svc data enable"
    data_disable_cmd = "svc data disable"
    wifi_disable_cmd = "svc wifi disable"

    def setUp(self):
        UiAutomatorUtils.close_all_tasks()
        self.initial_orientation = d.orientation
        self.screenshooter = ScreenshotUtils()
        if TelephonyUtils.is_wifi_enabled():
            AdbUtils.run_adb_cmd(TelephonyApnTests.wifi_disable_cmd)
        if TelephonyUtils.get_data_connection_state() != "2":
            AdbUtils.run_adb_cmd(TelephonyApnTests.data_enable_cmd)
        Chrome.launch()
        Chrome.delete_bookmarks()
        SingleMethodRunner.run_single_test(ChromeSupportTests, "test_add_bookmark")
        AdbUtils.run_adb_cmd(TelephonyApnTests.data_disable_cmd)

    def tearDown(self):
        self.assertTrue(Settings.reset_apns(), "The APN list could not be reset to default")
        self.assertTrue(Settings.select_first_apn(), "First APN could not be selected")
        self.log_before_cleanup()
        UiAutomatorUtils.close_all_tasks()

        if d.orientation != self.initial_orientation:
            d.orientation = self.initial_orientation
            d.freeze_rotation(False)

    def log_before_cleanup(self):
        result = self._resultForDoCleanups
        try:
            if not result.wasSuccessful():
                TestUtils.log_screenshot_and_dump()
        except:
            LOG.info("Result Proxy yielded an error")

    def test_internal_pdp_with_apn_in_acl(self):
        """
        ST_TELEPHONY_BA_ACL_009
        Internal PDP using an APN present in ACL
        """
        # Check if an APN is selected; if not, reset to default --> the operator APN will be selected automatically
        if Settings.is_apn_selected() is not True:
            self.assertTrue(Settings.reset_apns(), "The APN list could not be reset to default")

        AdbUtils.run_adb_cmd(TelephonyApnTests.data_enable_cmd)

        time.sleep(3)
        self.assertEqual(TelephonyUtils.get_data_connection_state(), "2", "Data is not connected")

        page_loaded_result = SingleMethodRunner.run_single_test(ChromeSupportTests, "test_page_loaded")
        self.assertTrue(page_loaded_result.wasSuccessful(), "Page could not be loaded")

        AdbUtils.run_adb_cmd(TelephonyApnTests.data_disable_cmd)
        time.sleep(3)
        self.assertNotEqual(TelephonyUtils.get_data_connection_state(), "2", "Data is still connected")

    def test_internal_pdp_with_apn_not_in_acl(self):
        """
        ST_TELEPHONY_BA_ACL_011
        Internal PDP using an APN not present in ACL
        """
        # Create a test APN, that will not work
        self.assertTrue(Settings.add_broken_apn(), "The APN could not be created")
        self.assertTrue(Settings.select_first_apn(), "First APN could not be selected")

        time.sleep(1)
        self.assertEqual(TelephonyUtils.get_data_connection_apn_status(), "apnChanged", "APN has not changed")

        AdbUtils.run_adb_cmd(TelephonyApnTests.data_enable_cmd)

        time.sleep(3)
        self.assertNotEqual(TelephonyUtils.get_data_connection_state(), "2", "Data is connected, but should not")

        page_loaded_result = SingleMethodRunner.run_single_test(ChromeSupportTests, "test_page_loaded")
        self.assertFalse(page_loaded_result.wasSuccessful(), "Page loaded, but should not")

        AdbUtils.run_adb_cmd(TelephonyApnTests.data_disable_cmd)
        time.sleep(3)
        self.assertNotEqual(TelephonyUtils.get_data_connection_state(), "2", "Data is still connected")


if __name__ == "__main__":
    test_result = SingleMethodRunner.run_single_test(TelephonyApnTests, "test_internal_pdp_with_apn_in_acl")
    print test_result.wasSuccessful()
