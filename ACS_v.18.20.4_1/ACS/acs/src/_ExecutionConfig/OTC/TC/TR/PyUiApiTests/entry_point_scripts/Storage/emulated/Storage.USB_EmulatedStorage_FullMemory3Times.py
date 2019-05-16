from _prerequisites import *
from PyUiApi.tests.storage_usb_tests.reliability import *

test_result = SingleMethodRunner.run_single_test(StorageUSBTestsReliability,
                                                 "test_full_memory_3_times")

if test_result.wasSuccessful():
    print "TEST_PASSED"
else:
    TestUtils.print_test_result_problems(test_result)
    sys.exit("FAIL")
