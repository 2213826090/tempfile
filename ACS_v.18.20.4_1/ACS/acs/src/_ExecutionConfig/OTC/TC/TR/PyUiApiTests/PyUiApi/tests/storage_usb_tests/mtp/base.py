"""
These tests work on Ubuntu 14.04 out of the box
If you have previous versions of Ubuntu, you need to:
sudo add-apt-repository ppa:langdalepl/gvfs-mtp
and update
"""

from PyUiApi.common.storage_usb_utils import *
from PyUiApi.common.system_utils import *
from PyUiApi.linux_utils.mtp_utils import *
from threading import Thread


class StorageUSBMTPTests(unittest.TestCase):
    create_folder_name = "mtp_test_folder"
    create_file_name = "mtp_test_file.txt"
    test_content = "this_is_some_random_content"
    test_MTP_internal_storage_path = True

    def setUp(self):
        UiAutomatorUtils.unlock_screen()
        self.initial_orientation = d.orientation
        USBChooser.select_mtp_option(refresh=False)
        self.cleanup_paths = []
        if StorageUSBMTPTests.test_MTP_internal_storage_path:
            Environment.initialize_mtp_paths()
            self.ensure_mtp_is_selected()
            self.mtp_dir = EnvironmentUtils.get_dut_MTP_host_path()
            LOG.info("mtp dir: %s" % self.mtp_dir)
            self.assertIsNotNone(self.mtp_dir, "could not find a valid mtp path on host")
            self.ensure_mtp_gvfs_monitor_is_functioning()
            self.init_mtp_internal_storage_test_paths()
        LOG.info("Test started")

    def init_devices_usb_mode(self, devices):
        dut_serial = dut_manager.active_uiautomator_device_serial
        if not devices:
            return

        for serial in devices:
            AdbUtils.run_adb_cmd('root', adb_shell=False, dut_serial=serial)
            time.sleep(3)
            dut_manager.activate_dut(serial)
            UiAutomatorUtils.unlock_screen()
            USBChooser.select_default_option(refresh=False)

        dut_manager.activate_dut(dut_serial)

    def ensure_mtp_is_selected(self):
        if Environment.mtp_internal_storage_path is None and Environment.mtp_sdcard_path is None:
            d.press.home()
            self.assertTrue(USBChooser.select_mtp_option(), "could not select MTP mode for device")
            time.sleep(2)
            Environment.initialize_mtp_paths()

    def ensure_mtp_gvfs_monitor_is_functioning(self):
        if Environment.mtp_internal_storage_path is None and Environment.mtp_sdcard_path is None:
            LOG.info("something wrong with mtp_gvf_monitor ... restarting it")
            MTPUtils.restart_mtp_monitor()
            Environment.initialize_mtp_paths()

    def init_mtp_internal_storage_test_paths(self):
        self.cleanup_paths = []
        self.create_file_path = os.path.join(Environment.mtp_internal_storage_path,
                                             StorageUSBMTPTests.create_file_name)
        self.dut_created_file_path = os.path.join(Environment.emulated_storage_path,
                                                  StorageUSBMTPTests.create_file_name)
        self.create_folder_path = os.path.join(Environment.mtp_internal_storage_path,
                                               StorageUSBMTPTests.create_folder_name)
        self.dut_created_folder_path = os.path.join(Environment.emulated_storage_path,
                                                    StorageUSBMTPTests.create_folder_name)

    def tearDown(self):
        self.cleanup()
        if d.orientation != self.initial_orientation:
            d.orientation = self.initial_orientation
            d.freeze_rotation(False)
        # UiAutomatorUtils.close_all_tasks()
        LOG.info("Test finished")

    def cleanup(self):
        for del_file in self.cleanup_paths:
            LOG.info("deleting: " + del_file)
            self.mtp_delete_file_or_folder(del_file)

    def add_paths_for_cleanup(self, *dut_paths):
        for file_path in dut_paths:
            self.cleanup_paths.append(file_path)

    def create_file_with_content(self, file_path, content):
        LOG.info("creating file through mtp: " + file_path)
        f = open(file_path, 'w')
        f.write(content)
        f.close()

    def mtp_delete_file_or_folder(self, mtp_path):
        if os.path.isfile(mtp_path):
            os.remove(mtp_path)
        elif os.path.isdir(mtp_path):
            shutil.rmtree(mtp_path)

    def create_folder(self, folder_path):
        os.makedirs(folder_path)

    def copy_file(self, source_path, destination_path):
        shutil.copy(source_path, destination_path)

    def check_folder_through_adb(self, dut_folder_path, negative_test=False):
        folder_ls_output = AdbUtils.ls_dir(dut_folder_path)
        if not negative_test:
            self.assertTrue(Environment.file_not_found_cmd_output not in folder_ls_output)
        else:
            self.assertTrue(Environment.file_not_found_cmd_output in folder_ls_output)

    def check_file_through_adb(self, dut_file_path, reference_content=None, negative_test=False):
        adb_file_content = AdbUtils.cat_file(dut_file_path)
        LOG.info("got following file content through adb: " + adb_file_content)
        if reference_content is not None:
            LOG.info("comparing with reference content: " + reference_content)
            self.assertTrue(reference_content.lower().strip() == adb_file_content.lower().strip(),
                            "reference content does not match actual file content")
        elif not negative_test:
            self.assertTrue(Environment.file_not_found_cmd_output not in adb_file_content,
                            "file not found")
        elif negative_test:
            self.assertTrue(Environment.file_not_found_cmd_output in adb_file_content,
                            "file should not exist")

    def check_host_file(self, host_file_path, reference_content=None):
        self.assertTrue(os.path.exists(host_file_path), "file " + str(host_file_path) + " does not exist")
        if reference_content is not None:
            with open(host_file_path) as host_file:
                file_content = host_file.read()
                LOG.info("got following HOST file content: " + file_content)
                LOG.info("comparing with reference content: " + reference_content)
                self.assertTrue(file_content.strip().lower() == reference_content.strip().lower())

    def test_usb_chooser_options(self):
        self.assertTrue(USBChooser.get_usb_option_index(SETTINGS_USB_CHOOSER_MTP_OPTION_TXT),
                        "MTP option not found")
        self.assertTrue(USBChooser.get_usb_option_index(SETTINGS_USB_CHOOSER_PTP_OPTION_TXT),
                        "PTP option not found")

    def test_select_MTP_usage(self):
        d.press.home()
        self.assertTrue(USBChooser.select_mtp_option(), "could not select MTP mode for device")

    def test_create_file(self):
        self.add_paths_for_cleanup(self.create_file_path)
        self.create_file_with_content(self.create_file_path, StorageUSBMTPTests.test_content)
        self.check_file_through_adb(self.dut_created_file_path, StorageUSBMTPTests.test_content)

    def test_create_folder(self):
        self.add_paths_for_cleanup(self.create_folder_path)
        self.create_folder(self.create_folder_path)
        self.check_folder_through_adb(self.dut_created_folder_path)

    def test_delete_folder(self):
        self.create_folder(self.create_folder_path)
        self.check_folder_through_adb(self.dut_created_folder_path)
        self.mtp_delete_file_or_folder(self.create_folder_path)
        self.check_folder_through_adb(self.dut_created_folder_path, negative_test=True)

    def test_delete_file(self):
        self.add_paths_for_cleanup(self.create_file_path)
        self.create_file_with_content(self.create_file_path, StorageUSBMTPTests.test_content)
        self.check_file_through_adb(self.dut_created_file_path, StorageUSBMTPTests.test_content)
        self.mtp_delete_file_or_folder(self.create_file_path)
        self.check_file_through_adb(self.dut_created_file_path, negative_test=True)

    def test_create_file_255_chars(self):
        StorageUSBMTPTests.create_file_name = reduce(lambda x, y: x + y, ["a" for i in range(250)]) + ".txt"
        self.init_mtp_internal_storage_test_paths()
        self.test_create_file()

    def test_create_folder_255_chars(self):
        StorageUSBMTPTests.create_folder_name = reduce(lambda x, y: x + y,
                                                       ["a" for i in range(254 - len(Environment.mtp_internal_storage_path))])
        self.init_mtp_internal_storage_test_paths()
        self.test_create_folder()

    def test_copy_file_from_host(self):
        host_file_path = os.path.join(Environment.tmp_dir_path, StorageUSBMTPTests.create_file_name)
        self.create_file_with_content(host_file_path, StorageUSBMTPTests.test_content)
        self.add_paths_for_cleanup(host_file_path, self.create_file_path)
        ShellUtils.copy_file(host_file_path, self.create_file_path)
        self.check_file_through_adb(self.dut_created_file_path, StorageUSBMTPTests.test_content)

    def test_copy_file_to_host(self):
        self.test_create_file()
        host_file_path = os.path.join(Environment.tmp_dir_path, StorageUSBMTPTests.create_file_name)
        self.add_paths_for_cleanup(host_file_path)
        ShellUtils.copy_file(self.create_file_path, host_file_path)
        self.check_host_file(host_file_path, StorageUSBMTPTests.test_content)

    def test_mtp_transfer_while_adb_pull_push(self):
        adb_file_name = "adb_file.file"
        adb_file_path = os.path.join(Environment.tmp_dir_path, adb_file_name)
        mtp_file_name = "mtp_file.file"
        mtp_file_path = os.path.join(Environment.tmp_dir_path, mtp_file_name)
        adb_dut_file_path = os.path.join(Environment.emulated_storage_path, adb_file_name)
        mtp_dut_file_path = os.path.join(Environment.emulated_storage_path, mtp_file_name)
        try:
            ShellUtils.fallocate_file("200M", adb_file_path)
            ShellUtils.fallocate_file("200M", mtp_file_path)
            mtp_copy_dir = os.path.normpath(Environment.mtp_internal_storage_path)

            success_status = {"adb thread": False, "mtp thread": False}

            def adb_push_pull():
                LOG.info("adb pushing file: " + str(adb_file_path) + " to: " + str(Environment.emulated_storage_path))
                AdbUtils.push(adb_file_path, Environment.emulated_storage_path)
                time.sleep(1)
                LOG.info("adb pulling file from: " + str(adb_dut_file_path) + " to: " + str(Environment.tmp_dir_path))
                AdbUtils.pull(adb_dut_file_path, Environment.tmp_dir_path)
                success_status["adb thread"] = True

            def mtp_copy():
                LOG.info("mtp copy file: " + str(mtp_file_path) + " to: " + str(mtp_copy_dir))
                ShellUtils.gvfs_copy_file(mtp_file_path, mtp_copy_dir)
                success_status["mtp thread"] = True

            adb_thread = Thread(target=adb_push_pull)
            mtp_thread = Thread(target=mtp_copy)
            adb_thread.start()
            time.sleep(5)
            mtp_thread.start()
            adb_thread.join()
            mtp_thread.join()

            self.assertTrue(success_status["adb thread"], "adb push pull was not successful")
            self.assertTrue(success_status["mtp thread"], "mtp copy was not successful")
            self.assertTrue(Environment.file_not_found_cmd_output not in AdbUtils.ls_dir(adb_dut_file_path),
                            "adb file not present on dut")
            self.assertTrue(Environment.file_not_found_cmd_output not in AdbUtils.ls_dir(mtp_dut_file_path),
                            "mtp file not present on dut")

        finally:
            ShellUtils.delete_file_or_folder(adb_file_path)
            ShellUtils.delete_file_or_folder(mtp_file_path)
            AdbUtils.delete_files(adb_dut_file_path, mtp_dut_file_path)


class USBChooseMTP(StorageUSBMTPTests):

    def setUp(self):
        UiAutomatorUtils.unlock_screen()
        self.initial_orientation = d.orientation

    def prereq_select_MTP(self):
        dut_serial = dut_manager.active_uiautomator_device_serial
        other_devices = dut_manager.get_available_dut_serials()
        other_devices.remove(dut_serial)

        self.init_devices_usb_mode(other_devices)
        self.test_select_MTP_usage()

    def tearDown(self):
        if d.orientation != self.initial_orientation:
            d.orientation = self.initial_orientation
            d.freeze_rotation(False)
