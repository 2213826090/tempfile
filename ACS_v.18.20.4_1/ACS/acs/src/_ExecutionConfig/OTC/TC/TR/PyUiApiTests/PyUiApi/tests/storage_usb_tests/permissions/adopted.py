from PyUiApi.tests.storage_usb_tests.with_adb.base import *


class StorageUSBTestsAdoptedPermissions(StorageUSBTestsWithAdb):
    instrumentation_pass_msg = InstrumentationInterface.instrumentation_one_test_pass_output
    storage_path = EnvironmentUtils.get_emulated_storage_path()
    appCacheDirIndex = 0

    def setUp(self):
        super(StorageUSBTestsAdoptedPermissions, self).setUp()
        self.test_create_dir = StorageUSBTestsAdoptedPermissions.storage_path
        self.test_file_name = "check_permissions_adopted.file"
        self.test_file_content = "this is a test file"
        self.instrumentation_args = ApiTestsGenericExtraArgs(deleteCreatedFiles="true")
        LOG.info("Test started")

    def tearDown(self):
        LOG.info("Test finished")

    def check_instrumentation_passed(self, result):
        self.assertTrue(StorageUSBTestsAdoptedPermissions.instrumentation_pass_msg in result)

    def run_check_permissions_instrumentation(self):
        test_args = self.instrumentation_args\
            .get_args_string(externalStorageFilePathForRead=self.test_create_dir+self.test_file_name,
                             externalStorageFileNameForWrite="check_perms_test_adopted.file")
        LOG.info("test args: " + test_args)
        result = CheckPermissionsInterface\
            .run_instrumentation(class_name="CheckPermissionTest",
                                 method_name="testPermission",
                                 instrumentation_args=test_args,
                                 runner_name="GenericArgumentPassingTestRunner")
        LOG.info(result)
        return result

    def test_permissions(self):
        self.assertTrue(EnvironmentUtils.is_sdcard_adopted())
        self.test_create_a_file()
        result = self.run_check_permissions_instrumentation()
        self.check_instrumentation_passed(result)

