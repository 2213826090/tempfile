"""
These tests work on Ubuntu 14.04 out of the box
If you have previous versions of Ubuntu, you need to:
sudo add-apt-repository ppa:langdalepl/gvfs-mtp
and update
"""

from PyUiApi.tests.storage_usb_tests.mtp.base import *
from PyUiApi.common.acs_utils import AcsUtils


class StorageUSBMTPTestsPortable(StorageUSBMTPTests):
    test_MTP_sd_card_path = False

    def setUp(self):
        super(StorageUSBMTPTestsPortable, self).setUp()
        self.assertTrue(EnvironmentUtils.is_sdcard_portable())
        UiAutomatorUtils.unlock_screen()
        self.initial_orientation = d.orientation
        if StorageUSBMTPTestsPortable.test_MTP_sd_card_path:
            self.init_mtp_sdcard_test_paths()
        LOG.info("Test started")

    def tearDown(self):
        if d.orientation != self.initial_orientation:
            d.orientation = self.initial_orientation
            d.freeze_rotation(False)
        # UiAutomatorUtils.close_all_tasks()
        super(StorageUSBMTPTestsPortable, self).tearDown()
        LOG.info("Test finished")

    def test_mtp_mounts(self):
        dut_mtp_mounts = os.listdir(self.mtp_dir)
        # sdcard is portable, so one must have the internal storage and the sdcard accesible from mtp
        self.assertTrue(len(dut_mtp_mounts) >= 2)
        for mountpoint in dut_mtp_mounts:
            mountpoint_path = os.path.join(self.mtp_dir, mountpoint)
            LOG.info("found mtp mountpoint: " + mountpoint_path)
            # only dirs must be available inside the mtp folder
            self.assertTrue(os.path.isdir(mountpoint_path))

    def init_mtp_sdcard_test_paths(self):
        self.cleanup_paths = []
        self.create_file_path = os.path.join(Environment.mtp_sdcard_path,
                                             StorageUSBMTPTests.create_file_name)
        self.dut_created_file_path = os.path.join(Environment.sd_card_path,
                                                  StorageUSBMTPTests.create_file_name)
        self.create_folder_path = os.path.join(Environment.mtp_sdcard_path,
                                               StorageUSBMTPTests.create_folder_name)
        self.dut_created_folder_path = os.path.join(Environment.sd_card_path,
                                                    StorageUSBMTPTests.create_folder_name)

    def test_create_file_255_chars(self):
        StorageUSBMTPTests.create_file_name = reduce(lambda x, y: x+y, ["a" for i in range(250)]) + ".txt"
        if StorageUSBMTPTestsPortable.test_MTP_sd_card_path:
            self.init_mtp_sdcard_test_paths()
        else:
            self.init_mtp_internal_storage_test_paths()
        self.test_create_file()

    def test_create_file_255_chars_through_adb(self):
        file_name = reduce(lambda x, y: x+y, ["a" for i in range(250)]) + ".txt"
        if StorageUSBMTPTestsPortable.test_MTP_sd_card_path:
            dir_path = EnvironmentUtils.get_sd_card_path()
        else:
            dir_path = EnvironmentUtils.get_emulated_storage_path()

        dut_created_file_path = dir_path + file_name
        AdbUtils.create_file(file_name, dir_path, StorageUSBMTPTests.test_content)
        self.check_file_through_adb(dut_created_file_path, StorageUSBMTPTests.test_content)
        AdbUtils.run_adb_cmd(DEL_CMD + dut_created_file_path)

    def test_create_folder_255_chars(self):
        StorageUSBMTPTests.create_folder_name = reduce(lambda x, y: x+y,
                                                       ["a" for i in range(254 - len(Environment.mtp_internal_storage_path))])
        if StorageUSBMTPTestsPortable.test_MTP_sd_card_path:
            self.init_mtp_sdcard_test_paths()
        else:
            self.init_mtp_internal_storage_test_paths()
        self.test_create_folder()

    def test_copy_file_internal_to_sdcard(self):
        self.init_mtp_internal_storage_test_paths()
        self.test_create_file()
        mtp_sdcard_file_path = os.path.join(Environment.mtp_sdcard_path, StorageUSBMTPTests.create_file_name)
        self.add_paths_for_cleanup(mtp_sdcard_file_path)
        ShellUtils.copy_file(self.create_file_path, mtp_sdcard_file_path)
        self.check_host_file(mtp_sdcard_file_path, StorageUSBMTPTests.test_content)

    def test_copy_file_sdcard_to_internal(self):
        self.init_mtp_sdcard_test_paths()
        self.test_create_file()
        mtp_internal_storage_file_path = os.path.join(Environment.mtp_internal_storage_path,
                                                      StorageUSBMTPTests.create_file_name)
        self.add_paths_for_cleanup(mtp_internal_storage_file_path)
        ShellUtils.copy_file(self.create_file_path, mtp_internal_storage_file_path)
        self.check_host_file(mtp_internal_storage_file_path, StorageUSBMTPTests.test_content)

    def test_delete_from_MTP_to_create_free_space_for_app(self):
        fill_dir_path = os.path.join("/sdcard/", "fill_mem_dir")
        try:
            # fill memory
            StorageUsbUtils.fill_emulated_memory(10, fill_dir_path, leave_free_space_files=1)
            time.sleep(5)
            install_apk_path = AcsUtils.find_file_path_in_acs_downloaded_artifacts("*3DImersiveGame*")
            self.assertIsNotNone(install_apk_path, "could not find 3DImersiveGame.apk file locally")
            LOG.info("path to apk to be installed: " + str(install_apk_path))
            output = AdbUtils.run_adb_cmd("install -r " + install_apk_path, adb_shell=False, add_ticks=False)
            LOG.info("install command output: " + output)
            self.assertTrue("Failure" in output, "apk was installed regardless of insufficient memory")
            AdbUtils.delete_files(fill_dir_path)
            time.sleep(2)
            output = AdbUtils.run_adb_cmd("install -r " + install_apk_path, adb_shell=False, add_ticks=False)
            LOG.info("install command output: " + output)
            self.assertTrue("Failure" not in output, "could not install apk after freeing space")
        finally:
            AdbUtils.delete_files(fill_dir_path)
