from PyUiApi.common.storage_usb_utils import *
from PyUiApi.common.system_utils import *


class DeviceTestENV(unittest.TestCase):
    TAG = "DeviceTestENV"

    def setUp(self):
        self.cli_settings = SettingsCLI()
        UiAutomatorUtils.unlock_screen()

    def tearDown(self):
        UiAutomatorUtils.close_all_tasks()

    def setting_device_env(self):
        self.cli_settings.set_key_value('global', 'package_verifier_enable', '0')
        self.cli_settings.set_key_value('system', 'screen_off_timeout', '1800000')
        self.cli_settings.set_key_value('secure', 'install_non_market_apps', '1')

        print AdbUtils.run_adb_cmd_ext("pm grant com.intel.test.apitests android.permission.READ_EXTERNAL_STORAGE")
        print AdbUtils.run_adb_cmd_ext("pm grant com.intel.test.apitests android.permission.WRITE_EXTERNAL_STORAGE")

        print AdbUtils.run_adb_cmd_ext("pm grant com.intel.test.apitests android.permission.CAMERA")
        print AdbUtils.run_adb_cmd_ext("pm grant com.intel.test.apitests android.permission.RECORD_AUDIO")

        print AdbUtils.run_adb_cmd_ext("pm grant com.intel.test.apitests android.permission.READ_CALENDAR")
        print AdbUtils.run_adb_cmd_ext("pm grant com.intel.test.apitests android.permission.WRITE_CALENDAR")

if __name__ == "__main__":
    test_result = SingleMethodRunner.run_single_test(DeviceTestENV, "setting_device_env")
    print test_result.wasSuccessful()
