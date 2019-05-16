from _prerequisites import *
from PyUiApi.tests.storage_usb_tests.reliability import *

test_result = SingleMethodRunner.run_single_test(StorageUSBTestsReliability,
                                                 "test_adb_copy_3level_folder_10000_times")

if test_result.wasSuccessful():
    print "TEST_PASSED"
else:
    TestUtils.print_test_result_problems(test_result)
    sys.exit("FAIL")

