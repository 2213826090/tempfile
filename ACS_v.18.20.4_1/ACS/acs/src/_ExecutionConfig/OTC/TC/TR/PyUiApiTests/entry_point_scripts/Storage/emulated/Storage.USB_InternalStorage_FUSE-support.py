from _prerequisites import *
from PyUiApi.tests.storage_usb_tests.with_adb.misc import *

test_result = SingleMethodRunner.run_single_test(StorageUSBTestsWithAdbMisc, "test_internal_storage_fuse_support")

if test_result.wasSuccessful():
    print "TEST_PASSED"
else:
    TestUtils.print_test_result_problems(test_result)
    sys.exit("FAIL")
