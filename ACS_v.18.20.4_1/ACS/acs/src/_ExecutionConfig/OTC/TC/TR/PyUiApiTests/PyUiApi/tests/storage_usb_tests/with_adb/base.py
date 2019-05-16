from PyUiApi.common.storage_usb_utils import *
from PyUiApi.common.system_utils import *
from PyUiApi.adb_helper.logcat_messaging_gateway import *


class StorageUSBTestsWithAdb(unittest.TestCase):
    no_such_file_or_dir_msg = Environment.file_not_found_cmd_output
    test_data_dir = Environment.api_tests_data_cache_dir + "test/"

    def setUp(self):
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

    def check_file_exists_and_validate_content(self, file_path_on_dut, reference_content):
        file_content = AdbUtils.cat_file(file_path_on_dut)
        self.assertFalse(StorageUSBTestsWithAdb.no_such_file_or_dir_msg in file_content,
                         "File does not exist on DUT")
        self.assertTrue(reference_content in file_content, "File does not contain proper content")

    def check_file_does_not_exist(self, file_path_on_dut):
        file_content = AdbUtils.cat_file(file_path_on_dut)
        self.assertTrue(StorageUSBTestsWithAdb.no_such_file_or_dir_msg in file_content,
                        "File should not exist on DUT")

    def check_dir_exists(self, dir_path_on_dut):
        ls_output = AdbUtils.ls_dir(dir_path_on_dut)
        self.assertFalse(StorageUSBTestsWithAdb.no_such_file_or_dir_msg in ls_output,
                         "Dir does not exist on DUT")

    def check_dir_does_not_exist(self, dir_path_on_dut):
        ls_output = AdbUtils.ls_dir(dir_path_on_dut)
        self.assertTrue(StorageUSBTestsWithAdb.no_such_file_or_dir_msg in ls_output,
                        "Dir should not exist on DUT")

    def cleanUp(self, *dirs_to_delete):
        for folder in dirs_to_delete:
            AdbUtils.delete_files(folder)

    def test_create_a_file(self):
        file_path = self.test_create_dir + self.test_file_name
        AdbUtils.create_file(self.test_file_name, self.test_create_dir,
                             self.test_file_content)
        self.check_file_exists_and_validate_content(file_path, self.test_file_content)

    def test_create_a_file_255_chars_name(self):
        self.test_file_name = reduce(lambda x, y: x+y, ["a" for i in range(255)])
        file_path = self.test_create_dir + self.test_file_name
        AdbUtils.create_file(self.test_file_name, self.test_create_dir,
                             self.test_file_content)
        self.check_file_exists_and_validate_content(file_path, self.test_file_content)

    def test_create_a_folder(self):
        AdbUtils.create_dir(self.test_create_dir,)
        self.check_dir_exists(self.test_create_dir)

    def test_create_a_folder_255_chars_name(self):
        self.test_create_dir = self.test_dir_base_path + \
            reduce(lambda x, y: x+y, ["a" for i in range(255)])
        AdbUtils.create_dir(self.test_create_dir,)
        self.check_dir_exists(self.test_create_dir)

    def test_delete_a_file_255_chars_name(self):
        self.test_file_name = reduce(lambda x, y: x+y, ["a" for i in range(255)])
        file_path = self.test_create_dir + self.test_file_name
        AdbUtils.create_file(self.test_file_name, self.test_create_dir,
                             self.test_file_content)
        self.check_file_exists_and_validate_content(file_path, self.test_file_content)
        AdbUtils.delete_files(file_path)
        file_content = AdbUtils.cat_file(file_path)
        self.assertTrue(StorageUSBTestsWithAdb.no_such_file_or_dir_msg in file_content)

    def test_delete_a_folder_255_chars_name(self):
        self.test_create_dir = self.test_dir_base_path + \
            reduce(lambda x, y: x+y, ["a" for i in range(255)])
        AdbUtils.create_dir(self.test_create_dir,)
        self.check_dir_exists(self.test_create_dir)
        AdbUtils.delete_files(self.test_create_dir)
        ls_output = AdbUtils.ls_dir(self.test_create_dir)
        self.assertTrue(StorageUSBTestsWithAdb.no_such_file_or_dir_msg in ls_output)

    def test_rename_file(self):
        old_file_path = self.test_create_dir + self.test_file_name
        new_file_path = self.test_create_dir + self.new_test_file_name
        AdbUtils.create_file(self.test_file_name, self.test_create_dir,
                             self.test_file_content)
        self.check_file_exists_and_validate_content(old_file_path, self.test_file_content)
        AdbUtils.rename(old_file_path, new_file_path)
        self.check_file_exists_and_validate_content(new_file_path, self.test_file_content)
        self.check_file_does_not_exist(old_file_path)

    def test_rename_folder(self):
        old_folder_path = self.test_create_dir
        new_folder_path = self.test_rename_dir
        old_file_path = self.test_create_dir + self.test_file_name
        new_file_path = self.test_rename_dir + self.test_file_name
        AdbUtils.create_file(self.test_file_name, old_folder_path,
                             self.test_file_content)
        self.check_file_exists_and_validate_content(old_file_path, self.test_file_content)
        AdbUtils.rename(old_folder_path, new_folder_path)
        self.check_dir_exists(new_folder_path)
        self.check_dir_does_not_exist(old_folder_path)
        self.check_file_exists_and_validate_content(new_file_path, self.test_file_content)
        self.check_file_does_not_exist(old_file_path)

    def test_copy_file_to_data(self):
        source_file_path = self.test_create_dir + self.test_file_name
        dest_file_path = self.test_data_dir + self.new_test_file_name
        AdbUtils.create_file(self.test_file_name, self.test_create_dir,
                             self.test_file_content)
        self.check_file_exists_and_validate_content(source_file_path, self.test_file_content)
        AdbUtils.copy(source_file_path, dest_file_path)
        self.check_file_exists_and_validate_content(dest_file_path, self.test_file_content)

    def test_copy_file_from_data(self):
        source_file_path = self.test_data_dir + self.test_file_name
        dest_file_path = self.test_create_dir + self.new_test_file_name
        AdbUtils.create_file(self.test_file_name, self.test_data_dir,
                             self.test_file_content)
        self.check_file_exists_and_validate_content(source_file_path, self.test_file_content)
        AdbUtils.copy(source_file_path, dest_file_path)
        self.check_file_exists_and_validate_content(dest_file_path, self.test_file_content)

    def test_copy_file_from_data_to_data(self):
        source_file_path = self.test_data_dir + self.test_file_name
        dest_file_path = self.test_data_dir + "copy/" + self.new_test_file_name
        AdbUtils.create_file(self.test_file_name, self.test_data_dir,
                             self.test_file_content)
        self.check_file_exists_and_validate_content(source_file_path, self.test_file_content)
        AdbUtils.copy(source_file_path, dest_file_path)
        self.check_file_exists_and_validate_content(dest_file_path, self.test_file_content)

    def test_copy_file(self):
        source_file_path = self.test_create_dir + self.test_file_name
        dest_file_path = self.test_create_dir + "copy/" + self.test_file_name
        AdbUtils.create_file(self.test_file_name, self.test_create_dir,
                             self.test_file_content)
        self.check_file_exists_and_validate_content(source_file_path, self.test_file_content)
        AdbUtils.copy(source_file_path, dest_file_path)
        self.check_file_exists_and_validate_content(dest_file_path, self.test_file_content)

    def test_copy_folder_to_data(self):
        source_folder = self.test_create_dir
        dest_folder = Environment.api_tests_data_cache_dir
        source_file_path = source_folder + self.test_file_name
        dest_file_path = self.test_data_dir + self.test_file_name
        AdbUtils.create_file(self.test_file_name, self.test_create_dir,
                             self.test_file_content)
        self.check_file_exists_and_validate_content(source_file_path, self.test_file_content)
        self.check_dir_exists(source_folder)
        AdbUtils.copy(source_folder, dest_folder)
        self.check_file_exists_and_validate_content(dest_file_path, self.test_file_content)
        self.check_dir_exists(dest_folder)

    def test_copy_folder_from_data(self):
        source_folder = self.test_data_dir
        dest_folder = self.test_dir_base_path
        source_file_path = source_folder + self.test_file_name
        dest_file_path = self.test_create_dir + self.test_file_name
        AdbUtils.create_file(self.test_file_name, self.test_data_dir,
                             self.test_file_content)
        self.check_file_exists_and_validate_content(source_file_path, self.test_file_content)
        self.check_dir_exists(source_folder)
        AdbUtils.copy(source_folder[:-1], dest_folder[:-1])
        self.check_file_exists_and_validate_content(dest_file_path, self.test_file_content)
        self.check_dir_exists(dest_folder)

    def test_copy_folder(self):
        source_folder = self.test_create_dir
        dest_folder = self.test_rename_dir
        source_file_path = source_folder + self.test_file_name
        dest_file_path = dest_folder + self.test_dir_name + self.test_file_name
        AdbUtils.create_file(self.test_file_name, self.test_create_dir, self.test_file_content)
        self.check_file_exists_and_validate_content(source_file_path, self.test_file_content)
        self.check_dir_exists(source_folder)
        AdbUtils.copy(source_folder, dest_folder)
        self.check_file_exists_and_validate_content(dest_file_path, self.test_file_content)
        self.check_dir_exists(dest_folder)

    def test_read_file_with_adb(self):
        read_test_file_name = "read_test.file"
        file_path_on_dut = os.path.join(self.test_dir_base_path, read_test_file_name)
        file_content = "This is the file content, it should be available when validating file"
        LOG.info("created file for search: " + file_path_on_dut)
        try:
            AdbUtils.create_file(read_test_file_name, self.test_dir_base_path, file_content)
            self.check_file_exists_and_validate_content(file_path_on_dut, file_content)
        finally:
            StorageUsbUtils.delete_file_from_dut(file_path_on_dut)
