# -*- coding: utf-8 -*-
from PyUiApi.tests.storage_usb_tests.with_api.base import *
from PyUiApi.app_utils.photos_utils import Photos
from PyUiApi.tests.storage_usb_tests.mtp.portable import StorageUSBMTPTestsPortable
from PyUiApi.app_utils.settings_utils import Settings
from PyUiApi.tests.storage_usb_tests.generic import StorageUSBTests
from PyUiApi.tests.storage_usb_tests.mtp.adopted import StorageUSBMTPTestsAdopted
import sys

reload(sys)
sys.setdefaultencoding("utf-8")


class StorageUSBTestsPortableWithApi(StorageUSBTestsWithApi):
    emulated_storage_dir = EnvironmentUtils.get_emulated_storage_path()

    def setUp(self):
        super(StorageUSBTestsPortableWithApi, self).setUp()
        # make sure the external sdcard is portable
        self.assertTrue(StorageUsbUtils.check_sdcard_mounted(),
                        "SD Card must be portable")
        # common arguments for all the tests
        self.instrumentation_args = ApiTestsGenericExtraArgs(deleteCreatedFiles="true")
        self.storage_test_dir = EnvironmentUtils.get_sd_card_path() +\
            Environment.api_tests_data_cache_dir_sdcard

    def tearDown(self):
        LOG.info("Test finished")

    def test_copy_file_sdcard_to_emulated(self):
        test_args = self.instrumentation_args\
            .get_args_string(createDir=self.storage_test_dir + "test1/",
                             copyDir=StorageUSBTestsPortableWithApi.emulated_storage_dir + "test2/",
                             nrOfFiles=10, fileSizeKB=10)
        LOG.info("test args: " + test_args)
        result = ApiTestsInterface\
            .run_instrumentation(class_name="SystemStorageUSBTestsDriver",
                                 method_name="testCopyFile",
                                 instrumentation_args=test_args,
                                 runner_name="GenericArgumentPassingTestRunner")
        LOG.info(result)
        self.check_instrumentation_passed(result)

    def test_copy_folder_with_file_sdcard_to_emulated(self):
        test_args = self.instrumentation_args\
            .get_args_string(createDir=self.storage_test_dir + "test1/",
                             copyDir=StorageUSBTestsPortableWithApi.emulated_storage_dir + "test2/",
                             nrOfFiles=10, fileSizeKB=10)
        LOG.info("test args: " + test_args)
        result = ApiTestsInterface\
            .run_instrumentation(class_name="SystemStorageUSBTestsDriver",
                                 method_name="testCopyFolderWithFiles",
                                 instrumentation_args=test_args,
                                 runner_name="GenericArgumentPassingTestRunner")
        LOG.info(result)
        self.check_instrumentation_passed(result)

    def test_copy_file_emulated_to_sdcard(self):
        test_args = self.instrumentation_args\
            .get_args_string(createDir=StorageUSBTestsPortableWithApi.emulated_storage_dir + "test1/",
                             copyDir=self.storage_test_dir + "test2/",
                             nrOfFiles=10, fileSizeKB=10)
        LOG.info("test args: " + test_args)
        result = ApiTestsInterface\
            .run_instrumentation(class_name="SystemStorageUSBTestsDriver",
                                 method_name="testCopyFile",
                                 instrumentation_args=test_args,
                                 runner_name="GenericArgumentPassingTestRunner")
        LOG.info(result)
        self.check_instrumentation_passed(result)

    def test_copy_folder_with_file_emulated_to_sdcard(self):
        test_args = self.instrumentation_args\
            .get_args_string(createDir=StorageUSBTestsPortableWithApi.emulated_storage_dir + "test1/",
                             copyDir=self.storage_test_dir + "test2/",
                             nrOfFiles=10, fileSizeKB=10)
        LOG.info("test args: " + test_args)
        result = ApiTestsInterface\
            .run_instrumentation(class_name="SystemStorageUSBTestsDriver",
                                 method_name="testCopyFolderWithFiles",
                                 instrumentation_args=test_args,
                                 runner_name="GenericArgumentPassingTestRunner")
        LOG.info(result)
        self.check_instrumentation_passed(result)

    def test_copy_mp3_file_and_check_in_music_app(self):
        self.storage_test_dir = Environment.sd_card_path
        super(StorageUSBTestsPortableWithApi, self).test_copy_mp3_file_and_check_in_music_app()

    def test_special_chars_in_filename(self):
        # create the file locally and copy it to the sdcard
        test_filename = u"ひ이機الأبجàÖÑ12!@#hype.jpg"
        test_file_path = os.path.join(Environment.tmp_dir_path, test_filename)
        dut_copy_path = os.path.join(Environment.dcim_folder_path, test_filename)
        ShellUtils.run_shell_cmd("touch " + test_file_path)
        AdbUtils.run_adb_cmd("mkdir -p " + Environment.dcim_folder_path)
        AdbUtils.push(test_file_path, dut_copy_path)
        time.sleep(1)
        Photos.refresh_media_store(Environment.dcim_folder_path)
        time.sleep(2)  # wait a while for the media store to be updated
        test_args = self.instrumentation_args\
            .get_args_string(createDir=Environment.dcim_folder_path)
        result = ApiTestsInterface.run_instrumentation(class_name="FileSystemTestsDriver",
                                                       method_name="testFolderForSpecialCharsFile",
                                                       instrumentation_args=test_args,
                                                       runner_name="GenericArgumentPassingTestRunner")
        LOG.info(result)
        self.check_instrumentation_passed(result)

    def get_app_dirs(self):
        result = ApiTestsInterface\
            .run_instrumentation(class_name="SystemStorageUSBTestsDriver",
                                 method_name="testGetAppDirs",
                                 instrumentation_args=None,
                                 runner_name="GenericArgumentPassingTestRunner")
        LOG.info(result)
        self.check_instrumentation_passed(result)

    def interact_with_sdcard(self):
        sdcard_path = EnvironmentUtils.get_sd_card_path()
        LOG.info("after reboot sdcard path is: " + sdcard_path)
        self.assertIsNotNone(sdcard_path, "sdcard path was not found")
        # call this test to create ApiTests external dirs on sdcard if not created yet
        self.get_app_dirs()
        file_name = "test_file"
        folder_name = "test/folder/"
        test_args = self.instrumentation_args\
            .get_args_string(createDir=sdcard_path + Environment.api_tests_data_cache_dir_sdcard + folder_name,
                             fileName=file_name, fileSizeKB=2)
        LOG.info("test args: " + test_args)
        result = ApiTestsInterface\
            .run_instrumentation(class_name="SystemStorageUSBTestsDriver",
                                 method_name="testCreateFileWithName",
                                 instrumentation_args=test_args,
                                 runner_name="GenericArgumentPassingTestRunner")
        LOG.info(result)
        self.check_instrumentation_passed(result)

    def test_sdcard_sdsc_boot(self):
        UiAutomatorUtils.unlock_screen()
        UiAutomatorUtils.reboot_device()
        UiAutomatorUtils.unlock_screen()
        self.interact_with_sdcard()

    def test_sdcard_sdsc_operations(self):
        UiAutomatorUtils.unlock_screen()
        portable_succeded = False
        try:
            self.interact_with_sdcard()
            self.test_copy_mp3_file_and_check_in_music_app()
            StorageUSBMTPTestsPortable.test_MTP_sd_card_path = True
            d.press.home()
            self.assertTrue(USBChooser.select_mtp_option(), "could not select MTP mode for device")
            test_result = SingleMethodRunner.run_single_test(StorageUSBMTPTestsPortable,
                                                             "test_copy_file_from_host")
            self.assertTrue(test_result.wasSuccessful(), "mtp did not work properly")

            Settings.unmount_sdcard()
            self.assertTrue(Settings.vn.current_view_found, "could not unmount sdcard")
            Settings.mount_sdcard()
            self.assertTrue(Settings.vn.current_view_found, "could not remount sdcard")
            test_result = SingleMethodRunner.run_single_test(StorageUSBTests, "test_format_as_adoptable_migrate")
            self.assertTrue(test_result.wasSuccessful(), "could not adopt sdcard")
            d.press.home()
            self.assertTrue(USBChooser.select_mtp_option(), "could not select MTP mode for device")
            test_result = SingleMethodRunner.run_single_test(StorageUSBMTPTestsAdopted,
                                                             "test_copy_file_from_host")
            self.assertTrue(test_result.wasSuccessful(), "could not copy through MTP on adopted sdcard")
            test_result = SingleMethodRunner.run_single_test(StorageUSBTests, "test_format_as_portable")
            self.assertTrue(test_result.wasSuccessful(), "could not make portable sdcard")
            portable_succeded = True
        finally:
            if not portable_succeded:
                SingleMethodRunner.run_single_test(StorageUSBTests, "test_format_as_portable")
            UiAutomatorUtils.close_all_tasks()
