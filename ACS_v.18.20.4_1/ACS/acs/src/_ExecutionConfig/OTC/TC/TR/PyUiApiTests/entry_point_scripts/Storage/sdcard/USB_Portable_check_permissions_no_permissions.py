from _prerequisites import *
from PyUiApi.tests.storage_usb_tests.permissions.portable import *

# this will be the /sdcard/ dir on the device
StorageUSBTestsPortablePermissions.externalStorageReadable = "true"
# this will be the external sdcard cache dir on the device
StorageUSBTestsPortablePermissions.externalStorageWriteable = "false"
StorageUSBTestsPortablePermissions.externalStorageDirPathForWrite = Environment.emulated_storage_path
StorageUSBTestsPortablePermissions.appCacheDirIndex = 1

test_result = SingleMethodRunner.run_single_test(StorageUSBTestsPortablePermissions,
                                                 "test_permissions_custom")

if test_result.wasSuccessful():
    print "PASS"
else:
    TestUtils.print_test_result_problems(test_result)
    sys.exit("FAIL")


