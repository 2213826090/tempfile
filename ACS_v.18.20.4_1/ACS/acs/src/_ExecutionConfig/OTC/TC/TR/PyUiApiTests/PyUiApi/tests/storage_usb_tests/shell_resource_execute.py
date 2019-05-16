from PyUiApi.common.storage_usb_utils import *
from PyUiApi.common.system_utils import *
import os

class ShellResourceExecuter(unittest.TestCase):
    TAG = "ShellResourceExecute"

    def setUp(self):
        UiAutomatorUtils.unlock_screen()

    def tearDown(self):
        UiAutomatorUtils.close_all_tasks()

    def excute_shell_file(self):
        os.environ['ANDROID_SERIAL'] = dut_manager.active_uiautomator_device_serial
        _, output = run_cmd('bash %s' % self.shell_file)
        self.assertTrue('Pass' in output, output)

