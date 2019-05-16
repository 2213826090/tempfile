from PyUiApi.tests.storage_usb_tests.with_api.base import *


class StorageUSBTestsAdoptedWithApi(StorageUSBTestsWithApi):

    def setUp(self):
        super(StorageUSBTestsAdoptedWithApi, self).setUp()
        # make sure the external sdcard is adopted
        # removed check to decrease execution time
        # self.assertTrue(StorageUsbUtils.is_external_storage_adopted(),
        #                "Test requires adopted SD Card")
        # common arguments for all the tests
        self.instrumentation_args = ApiTestsGenericExtraArgs(deleteCreatedFiles="true")
        self.storage_test_dir = EnvironmentUtils.get_emulated_storage_path()

    def tearDown(self):
        LOG.info("Test finished")
