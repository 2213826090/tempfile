from _prerequisites import *
from PyUiApi.tests.storage_usb_tests.permissions.portable import *

test_result = SingleMethodRunner.run_single_test(StorageUSBTestsPortablePermissions, "test_permissions_negative")

if test_result.wasSuccessful():
    print "PASS"
else:
    TestUtils.print_test_result_problems(test_result)
    sys.exit("FAIL")


