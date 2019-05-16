from PyUiApi.common.storage_usb_utils import *
from PyUiApi.common.system_utils import *
from PyUiApi.adb_helper.logcat_messaging_gateway import *
from collections import Counter


class StorageUSBTestsContext(unittest.TestCase):
    instrumentation_pass_msg = InstrumentationInterface.instrumentation_one_test_pass_output

    data_cache_dir_emulated = EnvironmentUtils.get_emulated_storage_path() + Environment.context_tests_data_cache_dir
    data_external_dir_emulated = EnvironmentUtils.get_emulated_storage_path() + Environment.context_tests_data_external_dir
    obb_dir_emulated = EnvironmentUtils.get_emulated_storage_path() + Environment.context_tests_obb_dir

    data_cache_dir_portable = None
    data_external_dir_portable = None
    obb_dir_portable = None

    def init_portable_paths(self):
        StorageUSBTestsContext.data_cache_dir_portable = EnvironmentUtils.get_sd_card_path() + \
                                                         Environment.context_tests_data_cache_dir
        StorageUSBTestsContext.data_external_dir_portable = EnvironmentUtils.get_sd_card_path() + \
                                                            Environment.context_tests_data_external_dir
        StorageUSBTestsContext.obb_dir_portable = EnvironmentUtils.get_sd_card_path() + \
                                                  Environment.context_tests_obb_dir

    def setUp(self):
        LOG.info("Test started")

    def tearDown(self):
        LOG.info("Test finished")

    def check_instrumentation_passed(self, result):
        self.assertTrue(StorageUSBTestsContext.instrumentation_pass_msg in result)

    def context_instrumentation(self):
        result = ApiTestsInterface\
            .run_instrumentation(class_name="SystemStorageUSBTestsDriver",
                                 method_name="testGetAppDirs",
                                 instrumentation_args=None,
                                 runner_name="GenericArgumentPassingTestRunner")
        LOG.info(result)
        self.check_instrumentation_passed(result)

    def test_context_portable(self):
        self.init_portable_paths()
        self.assertTrue(StorageUsbUtils.check_sdcard_mounted(),
                        "SD Card must be portable")
        self.context_instrumentation()
        api_tests_msg = ApiTestsMessagingGateway()
        self.messages = api_tests_msg.get_logcat_messages()
        messages_obtained = api_tests_msg.get_latest_message_with_id("AppDirsMsgID").content_lines
        self.assertTrue(len(messages_obtained) is 6, "DUT with portable storage must have six context dirs")
        LOG.info("api context dirs" + str(messages_obtained))
        dirs_list = [StorageUSBTestsContext.data_cache_dir_emulated, StorageUSBTestsContext.data_external_dir_emulated,
                     StorageUSBTestsContext.obb_dir_emulated, StorageUSBTestsContext.data_cache_dir_portable,
                     StorageUSBTestsContext.data_external_dir_portable, StorageUSBTestsContext.obb_dir_portable]
        LOG.info("adb shell context dirs" + str(dirs_list))
        self.assertTrue(Counter(dirs_list) == Counter(messages_obtained), "Lists not the same")

    def test_context_adopted(self):
        self.assertTrue(StorageUsbUtils.is_external_storage_adopted(),
                        "Test requires adopted SD Card")
        self.context_instrumentation()
        api_tests_msg = ApiTestsMessagingGateway()
        self.messages = api_tests_msg.get_logcat_messages()
        messages_obtained = api_tests_msg.get_latest_message_with_id("AppDirsMsgID").content_lines
        self.assertTrue(len(messages_obtained) is 3, "DUT with adopted storage must have three context dirs")
        LOG.info(messages_obtained)
        dirs_list = [StorageUSBTestsContext.data_cache_dir_emulated, StorageUSBTestsContext.data_external_dir_emulated,
                     StorageUSBTestsContext.obb_dir_emulated]
        self.assertTrue(Counter(dirs_list) == Counter(messages_obtained), "Lists not the same")

    def test_context_portable_to_adopted(self):
        # check sdcard has been adopted with api
        result = ApiTestsInterface\
            .run_instrumentation(class_name="SystemStorageUSBTestsDriver",
                                 method_name="testSdcardIsAdopted",
                                 instrumentation_args=None,
                                 runner_name="GenericArgumentPassingTestRunner")
        if not InstrumentationInterface.was_instrumentation_test_successful(result):
            self.assertTrue(False, "Adopted storage failed the context api check")

