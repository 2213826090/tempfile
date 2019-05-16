"""
These tests work on Ubuntu 14.04 out of the box
If you have previous versions of Ubuntu, you need to:
sudo add-apt-repository ppa:langdalepl/gvfs-mtp
and update
"""

from PyUiApi.tests.storage_usb_tests.mtp.base import *
from PyUiApi.linux_utils.gphoto2_utils import *
from PyUiApi.app_utils.camera_utils import *
from PyUiApi.common.media_utils import MediaInfo
from threading import Thread


class StorageUSBPTPTests(StorageUSBMTPTests):
    create_folder_name = "ptp_test_folder"
    create_file_name = "ptp_test_file.txt"
    test_content = "this_is_some_random_content"
    test_PTP_internal_storage_path = True

    def setUp(self):
        UiAutomatorUtils.unlock_screen()
        self.initial_orientation = d.orientation
        USBChooser.select_ptp_option(refresh=False)
        self.cleanup_paths = []
        Gphoto2Utils.enable_gvfs_monitor()
        if StorageUSBPTPTests.test_PTP_internal_storage_path:
            if EnvironmentUtils.get_dut_PTP_host_path() is None:
                self.test_select_PTP_usage()  # if PTP was disabled somehow, re-enable it
            Environment.initialize_ptp_paths()
            self.init_ptp_internal_storage_test_paths()
            self.ptp_dir = EnvironmentUtils.get_dut_PTP_host_path()
            self.assertIsNotNone(self.ptp_dir, "could not find a valid ptp path on host")
        LOG.info("Test started")

    def init_ptp_internal_storage_test_paths(self):
        self.cleanup_paths = []
        self.create_file_path = os.path.join(Environment.ptp_path,
                                             StorageUSBMTPTests.create_file_name)
        self.dut_created_file_path = os.path.join(Environment.emulated_storage_path,
                                                  StorageUSBMTPTests.create_file_name)
        self.create_folder_path = os.path.join(Environment.ptp_path,
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

    def check_prerequisites(self):
        gphoto2_present = ShellUtils.run_shell_cmd("gphoto2 -v")
        self.assertTrue("not installed" not in gphoto2_present,
                        "test requires gphoto2 to be installed on the linux host, please run"
                        " 'sudo apt-get install gphoto2'")

    def test_select_PTP_usage(self):
        d.press.home()
        self.assertTrue(USBChooser.select_ptp_option(), "could not select PTP mode for DUT")

    def test_capture_image_and_video(self):
        AdbUtils.grant_permission("com.android.camera2", "android.permission.ACCESS_FINE_LOCATION")
        QuickCamera.take_photos(nr_of_photos=3)
        QuickCamera.take_video(nr_of_videos=2)

    def test_connect_dut_as_ptp_device(self):
        self.assertTrue("gphoto2" in self.ptp_dir, "device is not connected to pc as camera")
        ptp_dir_contents = ShellUtils.run_shell_cmd("ls " + self.ptp_dir)
        self.assertTrue("DCIM" in ptp_dir_contents, "DCIM dir was not found in " + self.ptp_dir)

    def test_import_pictures_from_dut_to_pc(self):
        self.check_prerequisites()
        try:
            self.assertTrue(Gphoto2Utils.disable_gvfs_monitor(), "could not disable gphoto2 gvfs monitor")
            Gphoto2Utils.go_to_working_dir()
            import_result = Gphoto2Utils.import_all_files()
            LOG.info("ptp importing yielded: " + str(import_result))
            self.assertTrue(".thumbnails" not in import_result, "thumbnails must not be imported through PTP")
            imported_files_names = os.listdir(Gphoto2Utils.gphoto2_working_dir)
            video_imported = False
            image_imported = False
            for filename in imported_files_names:
                if ".jpg" in filename.lower() or ".jpeg" in filename.lower():
                    image_imported = True
                if ".mp4" in filename.lower() or ".mpeg" in filename.lower():
                    video_imported = True
            self.assertTrue(video_imported, "no video file was imported")
            self.assertTrue(image_imported, "no image file was imported")
        finally:
            Gphoto2Utils.clean_working_dir()
            Gphoto2Utils.enable_gvfs_monitor()

    def test_ptp_transfer_while_pull_push(self):
        adb_file_name = "adb_file.file"
        adb_file_path = os.path.join(Environment.tmp_dir_path, adb_file_name)
        ptp_file_name = "ptp_file.file"
        ptp_file_path = os.path.join(Environment.tmp_dir_path, ptp_file_name)
        adb_dut_file_path = os.path.join(Environment.emulated_storage_path, adb_file_name)
        ptp_dut_file_path = os.path.join(Environment.dcim_folder_path, ptp_file_name)
        try:
            ShellUtils.fallocate_file("200M", adb_file_path)
            ShellUtils.fallocate_file("200M", ptp_file_path)
            ptp_dcim_dir = os.path.join(self.ptp_dir, "DCIM")

            success_status = {"adb thread": False, "ptp thread": False}

            def adb_push_pull():
                LOG.info("adb pushing file: " + str(adb_file_path) + " to: " + str(Environment.emulated_storage_path))
                AdbUtils.push(adb_file_path, Environment.emulated_storage_path)
                time.sleep(1)
                LOG.info("adb pulling file from: " + str(adb_dut_file_path) + " to: " + str(Environment.tmp_dir_path))
                AdbUtils.pull(adb_dut_file_path, Environment.tmp_dir_path)
                success_status["adb thread"] = True

            def ptp_copy():
                LOG.info("ptp copy file: " + str(ptp_file_path) + " to: " + str(ptp_dcim_dir))
                Gphoto2Utils.gvfs_copy_file(ptp_file_path, ptp_dcim_dir)
                success_status["ptp thread"] = True

            adb_thread = Thread(target=adb_push_pull)
            ptp_thread = Thread(target=ptp_copy)
            adb_thread.start()
            time.sleep(5)
            ptp_thread.start()
            adb_thread.join()
            ptp_thread.join()

            self.assertTrue(success_status["adb thread"], "adb push pull was not successful")
            self.assertTrue(success_status["ptp thread"], "ptp copy was not successful")
            self.assertTrue(Environment.file_not_found_cmd_output not in AdbUtils.ls_dir(adb_dut_file_path),
                            "adb file not present on dut")
            self.assertTrue(Environment.file_not_found_cmd_output not in AdbUtils.ls_dir(ptp_dut_file_path),
                            "ptp file not present on dut")

        finally:
            ShellUtils.delete_file_or_folder(adb_file_path)
            ShellUtils.delete_file_or_folder(ptp_file_path)
            AdbUtils.delete_files(adb_dut_file_path, ptp_dut_file_path)

    def test_transfer_in_out_4GB(self):
        transfer_file_name = "big_file_2GB.file"
        transfer_file_path = os.path.join(Environment.tmp_dir_path, transfer_file_name)
        dut_transfer_file_path = os.path.join(Environment.dcim_folder_path, transfer_file_name)

        try:
            ShellUtils.fallocate_file("2300M", transfer_file_path)
            ptp_dcim_dir = os.path.join(self.ptp_dir, "DCIM")
            Gphoto2Utils.gvfs_copy_file(transfer_file_path, ptp_dcim_dir)
            self.assertTrue(Environment.file_not_found_cmd_output not in AdbUtils.ls_dir(dut_transfer_file_path),
                            "transfer file not present on dut")
        finally:
            ShellUtils.delete_file_or_folder(transfer_file_path)
            AdbUtils.delete_files(dut_transfer_file_path)

    def test_transfer_pictures_one_by_one(self):
        self.check_prerequisites()
        try:
            self.assertTrue(Gphoto2Utils.disable_gvfs_monitor(), "could not disable gphoto2 gvfs monitor")
            Gphoto2Utils.go_to_working_dir()
            files_info = Gphoto2Utils.get_files_info()
            imported_files = []
            for info in files_info:
                LOG.info("processing: " + str(info))
                if info.size > 0 and 'unknown' not in info.mime.lower():
                    Gphoto2Utils.import_file(info.index)
                    imported_files.append(info.name)
            current_dir_content = ShellUtils.run_shell_cmd("ls -al")
            self.assertTrue(len(imported_files) > 0, "no images were imported, check DCIM dir")
            for file_name in imported_files:
                LOG.info("checking file was imported: " + str(file_name))
                self.assertTrue(file_name in current_dir_content)
                properties = MediaInfo.get_media_info(os.path.join(".", file_name))
                LOG.info("image properties: " + str(properties))
                self.assertTrue(properties["Format"], "image has no valid format")
                self.assertTrue(properties["Width"] > 0, "image has no valid width")
                self.assertTrue(properties["Height"] > 0, "image has no valid height")
        finally:
            Gphoto2Utils.clean_working_dir()
            Gphoto2Utils.enable_gvfs_monitor()


class USBChoosePTP(StorageUSBPTPTests):

    def setUp(self):
        UiAutomatorUtils.unlock_screen()
        self.initial_orientation = d.orientation

    def prereq_select_PTP(self):
        dut_serial = dut_manager.active_uiautomator_device_serial
        other_devices = dut_manager.get_available_dut_serials()
        other_devices.remove(dut_serial)

        self.init_devices_usb_mode(other_devices)
        self.test_select_PTP_usage()

    def tearDown(self):
        if d.orientation != self.initial_orientation:
            d.orientation = self.initial_orientation
            d.freeze_rotation(False)