from PyUiApi.common.storage_usb_utils import *
from PyUiApi.common.system_utils import *
from PyUiApi.common.acs_utils import *
from threading import Thread


class StorageUSBTestsWithApi(unittest.TestCase):
    instrumentation_pass_msg = InstrumentationInterface.instrumentation_one_test_pass_output
    no_such_file_or_dir_msg = Environment.file_not_found_cmd_output
    test_data_dir = Environment.api_tests_data_cache_dir + "test/"

    def setUp(self):
        # common stuff for all the tests
        self.instrumentation_args = ApiTestsGenericExtraArgs(deleteCreatedFiles="true")
        # default create dir
        self.storage_test_dir = EnvironmentUtils.get_emulated_storage_path()

    def tearDown(self):
        LOG.info("Test finished")

    def check_instrumentation_passed(self, result):
        self.assertTrue(StorageUSBTestsWithApi.instrumentation_pass_msg in result, "instrumentation failed")

    def test_create_a_file(self):
        test_args = self.instrumentation_args\
            .get_args_string(createDir=self.storage_test_dir + "test/",
                             nrOfFiles=10, fileSizeKB=2)
        LOG.info("test args: " + test_args)
        result = ApiTestsInterface\
            .run_instrumentation(class_name="SystemStorageUSBTestsDriver",
                                 method_name="testCreateFile",
                                 instrumentation_args=test_args,
                                 runner_name="GenericArgumentPassingTestRunner")
        LOG.info(result)
        self.check_instrumentation_passed(result)

    def test_create_a_file_255_char_name(self):
        long_file_name = reduce(lambda x, y: x+y, ["a" for i in range(255)])
        test_args = self.instrumentation_args\
            .get_args_string(createDir=self.storage_test_dir + "test/",
                             fileName=long_file_name, fileSizeKB=2)
        LOG.info("test args: " + test_args)
        result = ApiTestsInterface\
            .run_instrumentation(class_name="SystemStorageUSBTestsDriver",
                                 method_name="testCreateFileWithName",
                                 instrumentation_args=test_args,
                                 runner_name="GenericArgumentPassingTestRunner")
        LOG.info(result)
        self.check_instrumentation_passed(result)

    def test_delete_a_file_255_char_name(self):
        self.test_create_a_file_255_char_name()
        file_path = self.instrumentation_args.args["createDir"] + \
            self.instrumentation_args.args["fileName"]
        output = AdbUtils.run_adb_cmd("cat " + file_path)
        LOG.info("cat after deletion output:" + output)
        self.assertTrue(StorageUSBTestsWithApi.no_such_file_or_dir_msg in output)

    def test_create_a_folder_255_char_name(self):
        base_folder_path = self.storage_test_dir
        long_folder_name = reduce(lambda x, y: x+y, ["f" for i in range(255)])
        test_args = self.instrumentation_args\
            .get_args_string(createDir=base_folder_path + long_folder_name)
        LOG.info("test args: " + test_args)
        result = ApiTestsInterface\
            .run_instrumentation(class_name="SystemStorageUSBTestsDriver",
                                 method_name="testCreateFolder",
                                 instrumentation_args=test_args,
                                 runner_name="GenericArgumentPassingTestRunner")
        LOG.info(result)
        self.check_instrumentation_passed(result)

    def test_delete_a_folder_255_char_name_with_file(self):
        file_name = "test_file"
        long_folder_name = reduce(lambda x, y: x+y, ["f" for i in range(255)])
        folder_name = "test/" + long_folder_name + "/"
        test_args = self.instrumentation_args\
            .get_args_string(createDir=self.storage_test_dir + folder_name,
                             fileName=file_name, fileSizeKB=2)
        LOG.info("test args: " + test_args)
        result = ApiTestsInterface\
            .run_instrumentation(class_name="SystemStorageUSBTestsDriver",
                                 method_name="testCreateFileWithName",
                                 instrumentation_args=test_args,
                                 runner_name="GenericArgumentPassingTestRunner")
        LOG.info(result)
        self.check_instrumentation_passed(result)
        file_path = self.instrumentation_args.args["createDir"] + \
            self.instrumentation_args.args["fileName"]
        output = AdbUtils.run_adb_cmd("cat " + file_path)
        LOG.info("cat after deletion output:" + output)
        self.check_instrumentation_passed(result)

    def test_create_a_folder(self):
        base_folder_path = self.storage_test_dir
        long_folder_name = "test_folder"
        test_args = self.instrumentation_args\
            .get_args_string(createDir=base_folder_path + long_folder_name)
        LOG.info("test args: " + test_args)
        result = ApiTestsInterface\
            .run_instrumentation(class_name="SystemStorageUSBTestsDriver",
                                 method_name="testCreateFolder",
                                 instrumentation_args=test_args,
                                 runner_name="GenericArgumentPassingTestRunner")
        LOG.info(result)
        self.check_instrumentation_passed(result)

    def test_create_multiextension_file(self):
        multi_ext_filename = "test_file.txt.png.mp3"
        test_args = self.instrumentation_args\
            .get_args_string(createDir=self.storage_test_dir + "test/",
                             fileName=multi_ext_filename, fileSizeKB=2)
        LOG.info("test args: " + test_args)
        result = ApiTestsInterface\
            .run_instrumentation(class_name="SystemStorageUSBTestsDriver",
                                 method_name="testCreateFileWithName",
                                 instrumentation_args=test_args,
                                 runner_name="GenericArgumentPassingTestRunner")
        LOG.info(result)
        self.check_instrumentation_passed(result)

    def test_create_file_in_folder(self):
        file_name = "test_file"
        folder_name = "test/folder/"
        test_args = self.instrumentation_args\
            .get_args_string(createDir=self.storage_test_dir + folder_name,
                             fileName=file_name, fileSizeKB=2)
        LOG.info("test args: " + test_args)
        result = ApiTestsInterface\
            .run_instrumentation(class_name="SystemStorageUSBTestsDriver",
                                 method_name="testCreateFileWithName",
                                 instrumentation_args=test_args,
                                 runner_name="GenericArgumentPassingTestRunner")
        LOG.info(result)
        self.check_instrumentation_passed(result)

    def test_rename_file(self):
        old_file = "old_file_name"
        new_file = "new_file_name"
        test_args = self.instrumentation_args\
            .get_args_string(createDir=self.storage_test_dir + "test/",
                             fileName=old_file, fileSizeKB=2,
                             newFileName=new_file, notCreateTest="true")
        LOG.info("test args: " + test_args)
        result = ApiTestsInterface\
            .run_instrumentation(class_name="SystemStorageUSBTestsDriver",
                                 method_name="testRenameFile",
                                 instrumentation_args=test_args,
                                 runner_name="GenericArgumentPassingTestRunner")
        LOG.info(result)
        self.check_instrumentation_passed(result)

    def test_rename_folder(self):
        old_dir = "old_dir"
        new_dir = "new_dir"
        test_args = self.instrumentation_args\
            .get_args_string(createDir=self.storage_test_dir + old_dir,
                             newFileName=self.storage_test_dir + new_dir)
        LOG.info("test args: " + test_args)
        result = ApiTestsInterface\
            .run_instrumentation(class_name="SystemStorageUSBTestsDriver",
                                 method_name="testRenameFolder",
                                 instrumentation_args=test_args,
                                 runner_name="GenericArgumentPassingTestRunner")
        LOG.info(result)
        self.check_instrumentation_passed(result)

    def test_copy_file_from_data(self):
        test_args = self.instrumentation_args\
            .get_args_string(createDir=StorageUSBTestsWithApi.test_data_dir,
                             copyDir=self.storage_test_dir + "test1/",
                             nrOfFiles=10, fileSizeKB=10)
        LOG.info("test args: " + test_args)
        result = ApiTestsInterface\
            .run_instrumentation(class_name="SystemStorageUSBTestsDriver",
                                 method_name="testCopyFile",
                                 instrumentation_args=test_args,
                                 runner_name="GenericArgumentPassingTestRunner")
        LOG.info(result)
        self.check_instrumentation_passed(result)

    def test_copy_folder_with_file_from_data(self):
        test_args = self.instrumentation_args\
            .get_args_string(createDir=StorageUSBTestsWithApi.test_data_dir,
                             copyDir=self.storage_test_dir + "test1/",
                             nrOfFiles=10, fileSizeKB=10)
        LOG.info("test args: " + test_args)
        result = ApiTestsInterface\
            .run_instrumentation(class_name="SystemStorageUSBTestsDriver",
                                 method_name="testCopyFolderWithFiles",
                                 instrumentation_args=test_args,
                                 runner_name="GenericArgumentPassingTestRunner")
        LOG.info(result)
        self.check_instrumentation_passed(result)

    def test_copy_file_to_data(self):
        test_args = self.instrumentation_args\
            .get_args_string(createDir=self.storage_test_dir + "test1/",
                             copyDir=StorageUSBTestsWithApi.test_data_dir,
                             nrOfFiles=10, fileSizeKB=10)
        LOG.info("test args: " + test_args)
        result = ApiTestsInterface\
            .run_instrumentation(class_name="SystemStorageUSBTestsDriver",
                                 method_name="testCopyFile",
                                 instrumentation_args=test_args,
                                 runner_name="GenericArgumentPassingTestRunner")
        LOG.info(result)
        self.check_instrumentation_passed(result)

    def test_copy_folder_with_file_to_data(self):
        test_args = self.instrumentation_args\
            .get_args_string(createDir=self.storage_test_dir + "test1/",
                             copyDir=StorageUSBTestsWithApi.test_data_dir,
                             nrOfFiles=10, fileSizeKB=10)
        LOG.info("test args: " + test_args)
        result = ApiTestsInterface\
            .run_instrumentation(class_name="SystemStorageUSBTestsDriver",
                                 method_name="testCopyFolderWithFiles",
                                 instrumentation_args=test_args,
                                 runner_name="GenericArgumentPassingTestRunner")
        LOG.info(result)
        self.check_instrumentation_passed(result)

    def test_copy_file(self):
        test_args = self.instrumentation_args\
            .get_args_string(createDir=self.storage_test_dir + "test1/",
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

    def test_copy_folder_with_file(self):
        test_args = self.instrumentation_args\
            .get_args_string(createDir=self.storage_test_dir + "test1/",
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

    def test_search_file(self):
        search_test_file_name = "search_for_this.file"
        file_path_on_dut = os.path.join(self.storage_test_dir, search_test_file_name)
        LOG.info("created file for search: " + file_path_on_dut)
        try:
            StorageUsbUtils.create_file_on_dut(file_path_on_dut, 1)
            result = StorageUsbUtils.search_for_file_with_api(self.storage_test_dir, search_test_file_name)
            self.check_instrumentation_passed(result)
        finally:
            StorageUsbUtils.delete_file_from_dut(file_path_on_dut)

    def test_copy_mp3_file_and_check_in_music_app(self):
        # mp3 file will be downloaded from artifactory by ACS as specified in the acs xml file
        mp3_file_name = "MP3_16kHz_160kbps_Stereo.mp3"
        AcsUtils.copy_acs_artifact_to_dut(mp3_file_name, self.storage_test_dir)
        file_path_on_dut = os.path.join(self.storage_test_dir, mp3_file_name)
        try:
            # run instrumentation to start media scanner and try to find the freshly copied mp3 file
            test_args = file_path_on_dut + ":audio"
            result = ApiTestsInterface\
                .run_instrumentation(class_name="SystemStorageUSBTestsDriver",
                                     method_name="testCopyAudioAndMediaStoreRefresh",
                                     instrumentation_extra_args=test_args,
                                     runner_name="GenericArgumentPassingTestRunner")
            LOG.info(result)
            self.check_instrumentation_passed(result)
        finally:
            StorageUsbUtils.delete_file_from_dut(file_path_on_dut)

    nr_of_files_to_copy = 1
    file_size_kb = 500000

    def test_switch_orientation_when_copying_file(self):
        initial_orientation = d.orientation
        try:
            orientation_changer = ContinuousOrientationChanger()
            file_copier = FileCopier(Environment.emulated_storage_path + "test/", self.storage_test_dir,
                                     nr_of_files_to_copy=StorageUSBTestsWithApi.nr_of_files_to_copy,
                                     file_size_kb=StorageUSBTestsWithApi.file_size_kb)

            def orientation_change():
                orientation_changer.start_running()

            def file_copy():
                try:
                    file_copier.copy_file()
                finally:
                    ContinuousOrientationChanger.stop_operation = True

            orientation_change_thread = Thread(target=orientation_change)
            file_copy_thread = Thread(target=file_copy)
            orientation_change_thread.start()
            file_copy_thread.start()
            file_copy_thread.join()  # copy should finish first
            orientation_change_thread.join()
            self.assertTrue(file_copier.copy_successful, "large file copy was not successful")
        finally:
            if d.orientation != initial_orientation:
                d.orientation = initial_orientation
                d.freeze_rotation(False)
            UiAutomatorUtils.close_all_tasks()

