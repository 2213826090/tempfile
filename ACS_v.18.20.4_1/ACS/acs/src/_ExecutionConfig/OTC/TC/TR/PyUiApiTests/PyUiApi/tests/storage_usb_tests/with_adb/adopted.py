from PyUiApi.tests.storage_usb_tests.with_adb.base import *


class StorageUSBTestsAdoptedWithAdb(StorageUSBTestsWithAdb):
    def setUp(self):
        super(StorageUSBTestsAdoptedWithAdb, self).setUp()
        # make sure the external sdcard is adopted
        # remove check to decrease execution time
        # self.assertTrue(StorageUsbUtils.is_external_storage_adopted(),
        #                "Test requires adopted SD Card")
        self.test_dir_base_path = EnvironmentUtils.get_emulated_storage_path()
        self.test_file_content = "this is a test file"
        self.test_file_name = "test_file.txt"
        self.new_test_file_name = "new_test_file.txt"
        self.test_dir_name = "test/"
        self.test_new_dir_name = "new_test/"
        self.test_create_dir = self.test_dir_base_path + self.test_dir_name
        self.test_rename_dir = self.test_dir_base_path + self.test_new_dir_name
        self.test_data_dir = Environment.api_tests_data_cache_dir + self.test_dir_name
        self.do_clean_up = True

    def tearDown(self):
        if self.do_clean_up:
            self.cleanUp(self.test_create_dir,
                         self.test_rename_dir,
                         self.test_data_dir)
        LOG.info("Test finished")

