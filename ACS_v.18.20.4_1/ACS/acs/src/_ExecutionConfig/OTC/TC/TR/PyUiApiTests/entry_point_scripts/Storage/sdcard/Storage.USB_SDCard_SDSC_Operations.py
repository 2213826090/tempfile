from _prerequisites import *
from PyUiApi.tests.storage_usb_tests.with_api.portable import *

test_result = SingleMethodRunner.run_single_test(StorageUSBTestsPortableWithApi, "test_sdcard_sdsc_operations")

if test_result.wasSuccessful():
    print "TEST_PASSED"
else:
    TestUtils.print_test_result_problems(test_result)
    sys.exit("FAIL")
