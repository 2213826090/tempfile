from PyUiApi.common.system_utils import *
from PyUiApi.linux_utils.mtp_utils import *
from subprocess import Popen, PIPE
import subprocess

class StorageUSBMTPMiscTests(unittest.TestCase):
    def setUp(self):
        self.initial_orientation = d.orientation
        LOG.info("Test started")

    def tearDown(self):
        if d.orientation != self.initial_orientation:
            d.orientation = self.initial_orientation
            d.freeze_rotation(False)
        LOG.info("Test finished")

    def test_mtp_ptp_device_serial_number(self):
        self.assertTrue(len(dut_manager.devices_detected) > 1, "There must be at least 2 DUTS connected for this test")
        initial_detected_devices = dut_manager.devices_detected
        for serial in initial_detected_devices:
            AdbUtils.run_adb_cmd('root', adb_shell=False, dut_serial=serial)
            time.sleep(3)
            dut_manager.activate_dut(serial)
            UiAutomatorUtils.unlock_screen()
            self.assertTrue(USBChooser.select_mtp_option(), "could not select MTP mode for device " + str(serial))
        final_detected_devices = dut_manager.get_available_dut_serials()
        LOG.info("initial detected: " + str(initial_detected_devices) + " final detected: " + str(final_detected_devices))
        self.assertTrue(len(initial_detected_devices) == len(final_detected_devices), "initial devices are not the same as final devices")

    def test_adb_shell_connect_two_or_more_device(self):
        self.assertTrue(len(dut_manager.devices_detected) > 1,"There must be at least 2 DUTS connected for this test")
        initial_detected_devices = dut_manager.devices_detected
        LOG.info("Multiple Devices: " + str(initial_detected_devices))
        output = Popen(['adb', 'shell'], stdin=PIPE,stdout=PIPE, stderr=PIPE)
        out, err = output.communicate()
        if 'more than one device and emulator' in err:
            LOG.info("Error Found: "+str(err))
        else:
            LOG.info(str(out+err))
            self.assertFalse("Expected Error for multiple device adb shell command not found")


