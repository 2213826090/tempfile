from PyUiApi.tests.storage_usb_tests.mtp.base import *


class StorageUSBMTPTestsAdopted(StorageUSBMTPTests):

    def setUp(self):
        StorageUSBMTPTests.test_MTP_internal_storage_path = False
        super(StorageUSBMTPTestsAdopted, self).setUp()
        self.assertTrue(EnvironmentUtils.is_sdcard_portable())
        UiAutomatorUtils.unlock_screen()
        self.initial_orientation = d.orientation
        Environment.initialize_mtp_paths()
        self.init_mtp_adopted_test_paths()
        self.assertTrue(Environment.mtp_internal_storage_path is None,
                        "adopted SD CARD must only have 'SD CARD' in host MTP path")
        LOG.info("Test started")

    def tearDown(self):
        if d.orientation != self.initial_orientation:
            d.orientation = self.initial_orientation
            d.freeze_rotation(False)
        # UiAutomatorUtils.close_all_tasks()
        super(StorageUSBMTPTestsAdopted, self).tearDown()
        LOG.info("Test finished")

    def init_mtp_adopted_test_paths(self):
        self.cleanup_paths = []
        self.create_file_path = os.path.join(Environment.mtp_adopted_path,
                                             StorageUSBMTPTests.create_file_name)
        self.dut_created_file_path = os.path.join(Environment.emulated_storage_path,
                                                  StorageUSBMTPTests.create_file_name)
        self.create_folder_path = os.path.join(Environment.mtp_adopted_path,
                                               StorageUSBMTPTests.create_folder_name)
        self.dut_created_folder_path = os.path.join(Environment.emulated_storage_path,
                                                    StorageUSBMTPTests.create_folder_name)

    def test_create_file_255_chars(self):
        file_name_length = 250 - len(Environment.mtp_adopted_path)
        StorageUSBMTPTests.create_file_name = reduce(lambda x, y: x+y, ["a" for i in range(file_name_length)]) + ".txt"
        self.init_mtp_adopted_test_paths()
        self.test_create_file()

    def test_create_folder_255_chars(self):
        StorageUSBMTPTests.create_folder_name = reduce(lambda x, y: x+y,
                                                       ["a" for i in range(254 - len(Environment.mtp_adopted_path))])
        self.init_mtp_adopted_test_paths()
        self.test_create_folder()
