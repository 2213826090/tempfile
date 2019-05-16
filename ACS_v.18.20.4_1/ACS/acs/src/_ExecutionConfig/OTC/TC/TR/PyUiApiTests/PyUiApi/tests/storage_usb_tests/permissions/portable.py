from PyUiApi.tests.storage_usb_tests.permissions.adopted import *


class StorageUSBTestsPortablePermissions(StorageUSBTestsAdoptedPermissions):
    instrumentation_pass_msg = InstrumentationInterface.instrumentation_one_test_pass_output
    storage_path = EnvironmentUtils.get_sd_card_path() + Environment.check_permissions_external_cache_dir
    externalStorageWriteable = None
    externalStorageReadable = None
    externalStorageDirPathForWrite = None

    def setUp(self):
        super(StorageUSBTestsPortablePermissions, self).setUp()
        self.test_create_dir = StorageUSBTestsPortablePermissions.storage_path
        self.test_file_name = "check_permissions_portable.file"
        LOG.info("Test started")

    def test_permissions(self):
        self.assertTrue(EnvironmentUtils.is_sdcard_portable())
        self.test_create_a_file()
        result = self.run_check_permissions_instrumentation()
        self.check_instrumentation_passed(result)

    def test_permissions_custom(self):
        self.assertTrue(EnvironmentUtils.is_sdcard_portable())
        self.test_create_a_file()
        test_args = self.instrumentation_args\
            .get_args_string(externalStorageFilePathForRead=self.test_create_dir+self.test_file_name,
                             externalStorageFileNameForWrite="check_permissions_portable.file",
                             appCacheDirIndex=str(StorageUSBTestsPortablePermissions.appCacheDirIndex),
                             externalStorageReadable=StorageUSBTestsPortablePermissions.externalStorageReadable,
                             externalStorageWriteable=StorageUSBTestsPortablePermissions.externalStorageWriteable,
                             externalStorageDirPathForWrite=StorageUSBTestsPortablePermissions
                             .externalStorageDirPathForWrite)
        LOG.info("test args: " + test_args)
        result = CheckPermissionsInterface\
            .run_instrumentation(class_name="CheckPermissionTest",
                                 method_name="testPermission",
                                 instrumentation_args=test_args,
                                 runner_name="GenericArgumentPassingTestRunner")
        self.check_instrumentation_passed(result)

    def test_permissions_negative(self):
        # directory try to acces the portable sdcard root directly, it should fail
        self.test_create_dir = EnvironmentUtils.get_sd_card_path()
        self.assertTrue(EnvironmentUtils.is_sdcard_portable())
        # Use ApiTests, because it has all permissions needed for the negative test
        test_args = self.instrumentation_args\
            .get_args_string(createDir=self.test_create_dir + "test/",
                             nrOfFiles=1, fileSizeKB=20)
        result = ApiTestsInterface\
            .run_instrumentation(class_name="SystemStorageUSBTestsDriver",
                                 method_name="testCreateFile",
                                 instrumentation_args=test_args,
                                 runner_name="GenericArgumentPassingTestRunner")
        LOG.info(result)
        self.assertFalse(StorageUSBTestsAdoptedPermissions.instrumentation_pass_msg in result)

