from PyUiApi.common.system_utils import *
from PyUiApi.common.environment_utils import *
from PyUiApi.common.acs_utils import *
from threading import Thread
import subprocess
import psutil


class AdbTests(unittest.TestCase):
    TAG = "AdbTests"

    def test_logcat_b(self):
        output = AdbUtils.run_adb_cmd("logcat -d -b radio")
        expected_strings = ["TelephonyManager"]
        for s in expected_strings:
            self.assertTrue(s in output, "logcat is missing expected strings")
        self.assertTrue(len(output) > 5000, "logcat size is too small")  # a fair amount of characters

    def test_connection_unable_to_enter_sleep(self):
        LOG.info("unlocking screen")
        UiAutomatorUtils.unlock_screen()
        LOG.info("pressing power button")
        d.press.power()
        LOG.info("wait for screen to turn off")
        while SystemUtils.is_screen_on():
            time.sleep(3)
        # wait a while
        time.sleep(30)
        output = AdbUtils.run_adb_cmd("ls")
        self.assertTrue("error" not in output, "adb connection is not working after screen off")
        # unlock screen
        UiAutomatorUtils.unlock_screen()

    def test_adb_shell_coexist_one_quit(self):
        # use exec so that the adb shell process will replace the unix shell created with Popen
        adb_shell_cmd = "exec adb -s %s shell" % dut_manager.active_uiautomator_device_serial
        shell_1 = subprocess.Popen(adb_shell_cmd, shell=True, stdout=subprocess.PIPE,
                                   stdin=subprocess.PIPE, stderr=subprocess.PIPE, preexec_fn=os.setsid)
        shell_2 = subprocess.Popen(adb_shell_cmd, shell=True,  stdout=subprocess.PIPE,
                                   stdin=subprocess.PIPE, stderr=subprocess.PIPE, preexec_fn=os.setsid)
        try:
            time.sleep(5)
            self.assertTrue(shell_1.returncode is None, "adb shell 1 ended prematurely")
            self.assertTrue(shell_2.returncode is None, "adb shell 2 ended prematurely")
            shell_1.stdin.write("exit\n".encode())
            time.sleep(5)
            shell_1_process = psutil.Process(shell_1.pid)
            shell_2_process = psutil.Process(shell_2.pid)
            LOG.info("shell 1 status: " + str(shell_1_process.status()))
            self.assertTrue(shell_1_process.status() == psutil.STATUS_ZOMBIE, "adb shell did not exit properly")
            LOG.info("shell 2 status: " + str(shell_2_process.status()))
            self.assertTrue(shell_2_process.status() != psutil.STATUS_ZOMBIE, "adb shell exited prematurely")
        finally:
            os.killpg(os.getpgid(shell_1.pid), signal.SIGTERM)
            os.killpg(os.getpgid(shell_2.pid), signal.SIGTERM)
            shell_1.wait()
            shell_2.wait()

    def test_usb_debugging_not_selected(self):
        initial_connected_devices_serials = AdbUtils.get_adb_devices_serials()
        LOG.info("initial devices connected: " + str(initial_connected_devices_serials))
        try:
            AdbUtils.disable_enable_adb_debugging(disable_enable_timeout=10000)
            # wait for adb to shut down
            time.sleep(4)
            final_connected_devices_serials = AdbUtils.get_adb_devices_serials()
            LOG.info("devices connected after adb disable: " + str(final_connected_devices_serials))
            self.assertTrue(len(initial_connected_devices_serials) > len(final_connected_devices_serials),
                            "adb reports same device serials as before adb debugging disable")
        finally:
            # wait for adb debugging to be switched on again
            time.sleep(10)

    def test_install_r(self):
        apk_path = AcsUtils.find_file_path_in_acs_downloaded_artifacts("AnTuTu-5.6.apk")
        install_r_verdict = AdbUtils.run_adb_cmd("install -r " + apk_path, adb_shell=False)
        LOG.info("install -r vedict: " + install_r_verdict)
        self.assertTrue("Success" in install_r_verdict,
                        "adb install -r was not successful")

    def test_install(self):
        apk_path = AcsUtils.find_file_path_in_acs_downloaded_artifacts("AnTuTu-5.6.apk")
        install_r_verdict = AdbUtils.run_adb_cmd("install " + apk_path, adb_shell=False)
        LOG.info("install vedict: " + install_r_verdict)
        self.assertTrue("Success" in install_r_verdict,
                        "adb install was not successful")

    def test_uninstall(self):
        self.test_install_r()
        uninstall_verdict = AdbUtils.run_adb_cmd("uninstall com.antutu.ABenchMark", adb_shell=False)
        self.assertTrue("Success" in uninstall_verdict,
                        "adb uninstall was not successful")

    def test_logcat_bugreport(self):
        tmp_location="/tmp/bugreport.tmp"
        try:
            # Run the adb bugreport command
            AdbUtils.run_adb_cmd("bugreport > {0}".format(tmp_location), adb_shell=False, add_ticks=False)

            # Get the last but one line in the bugreport
            result = ShellUtils.run_shell_cmd("tail -2 {0} | head -1".format(tmp_location))
            self.assertTrue("done" in result, "The adb bugreport command does not have 'done' in output")
        finally:
            ShellUtils.delete_file_or_folder(tmp_location)

    def test_get_serialno(self):
        serialno = AdbUtils.run_adb_cmd("get-serialno", adb_shell=False, add_ticks=False).strip()
        adb_devices = ShellUtils.run_shell_cmd("adb devices")
        self.assertTrue(serialno in str(adb_devices), "The serialno is not in the devices list")

    def test_pull_fake_host(self):
        test_file = "/sdcard/test_file"
        fake_host = "fake_host_dir/"
        pull_dir = os.path.join(Environment.tmp_dir_path, fake_host)
        ShellUtils.delete_file_or_folder(pull_dir)
        expected_output = "cannot create '{0}': Is a directory".format(pull_dir)
        LOG.info("Expected output = {0}".format(expected_output))

        try:
            # Create the test file on the device
            AdbUtils.run_adb_cmd("touch {0}".format(test_file))

            cmd = "adb -s {0} pull {1} {2}".format(dut_manager.active_uiautomator_device_serial,
                                                   test_file, pull_dir)
            process = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            output, err = process.communicate()

            LOG.info("Running command {0}".format(cmd))
            LOG.info("stderr = {0}".format(err.strip()))

            self.assertTrue(expected_output == err.strip(), "The actual output is not the same as the expected one")
        finally:
            ShellUtils.delete_file_or_folder(pull_dir)
            AdbUtils.delete_files(test_file)

    def test_stop_uiautomator_on_dut(self):
        self.assertTrue(AdbUtils.kill_python_uiautomator_rpc_server_on_dut(),
                        "python uiautomator services are still running on dut")


class AdbPTPTests(unittest.TestCase):

    def setUp(self):
        UiAutomatorUtils.unlock_screen()
        self.assertTrue(USBChooser.select_ptp_option(), "could not select PTP mode for DUT")
        for i in range(10):  # give PTP some time to start
            time.sleep(1)
            self.ptp_path = EnvironmentUtils.get_dut_PTP_host_path()
            if self.ptp_path is not None:
                break
        self.assertIsNotNone(self.ptp_path, "ptp path could not be found")
        self.ptp_DCIM_path = os.path.join(self.ptp_path, "DCIM")
        self.ptp_transfer_file_name = "ptp_transfer.file"
        self.cleanup_files = []
        self.cleanup_dut_files = []

    def tearDown(self):
        UiAutomatorUtils.close_all_tasks()
        self.do_cleanup()
        LOG.info("Test finished")

    def add_file_for_cleanup(self, file_path):
        self.cleanup_files.append(file_path)

    def add_dut_file_for_cleanup(self, dut_file_path):
        self.cleanup_dut_files.append(dut_file_path)

    def do_cleanup(self):
        for file_path in self.cleanup_files:
            ShellUtils.delete_file_or_folder(file_path)
        for dut_file_path in self.cleanup_dut_files:
            AdbUtils.delete_files(dut_file_path)

    def create_file_for_ptp_transfer(self):
        file_path = os.path.join(Environment.tmp_dir_path, self.ptp_transfer_file_name)
        ShellUtils.fallocate_file("500M", file_path)
        self.add_file_for_cleanup(file_path)
        return file_path

    @staticmethod
    def copy_file_through_ptp(source_path, destination_path):
        ShellUtils.copy_file(source_path, destination_path)

    def prepare_file_transfer(self):
        ptp_transfer_file_path = self.create_file_for_ptp_transfer()
        dut_file_path = os.path.join("/sdcard/DCIM/", self.ptp_transfer_file_name)
        self.add_dut_file_for_cleanup(dut_file_path)

        def transfer_file():
            AdbPTPTests.copy_file_through_ptp(ptp_transfer_file_path, self.ptp_DCIM_path)

        return Thread(target=transfer_file)

    def test_adb_shell_ptp_transfer(self):
        transfer_thread = self.prepare_file_transfer()
        transfer_thread.start()
        time.sleep(3)
        self.assertTrue(AdbUtils.wait_for_adb(), "adb is not connected")
        file_found_with_adb = False
        for i in range(10):
            time.sleep(1)
            dut_dcim_content = AdbUtils.run_adb_cmd("ls /sdcard/DCIM/")
            if self.ptp_transfer_file_name in dut_dcim_content:
                file_found_with_adb = True
                break
        self.assertTrue(file_found_with_adb, "could not find the file that is currently transfering")
        transfer_thread.join()

    def test_adb_logcat_ptp_transfer(self):
        transfer_thread = self.prepare_file_transfer()
        transfer_thread.start()
        time.sleep(3)
        self.assertTrue(AdbUtils.wait_for_adb(), "adb is not connected")
        logcat_content = AdbUtils.run_adb_cmd("logcat -d")
        self.assertTrue(len(logcat_content) > 5000,
                        "logcat size is too small")
        transfer_thread.join()
