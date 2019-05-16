from PyUiApi.tests.storage_usb_tests.with_api.base import *


class StorageUSBTestsEmulatedWithApi(StorageUSBTestsWithApi):

    def setUp(self):
        super(StorageUSBTestsEmulatedWithApi, self).setUp()
        # make sure the external sdcard is NOT adopted
        self.assertFalse(StorageUsbUtils.is_external_storage_adopted(),
                         "SD Card must not be adopted")
        # common arguments for all the tests
        self.instrumentation_args = ApiTestsGenericExtraArgs(deleteCreatedFiles="true")
        self.storage_test_dir = EnvironmentUtils.get_emulated_storage_path()

    def tearDown(self):
        LOG.info("Test finished")
