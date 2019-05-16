from _prerequisites import *
from PyUiApi.tests.storage_usb_tests.reliability import *

test_result = SingleMethodRunner.run_single_test(StorageUSBTestsReliability,
                                                 "test_adb_copy_10000_times_emulated")

if test_result.wasSuccessful():
    print "TEST_PASSED"
else:
    TestUtils.print_test_result_problems(test_result)
    sys.exit("FAIL")

