from _prerequisites import *
from PyUiApi.tests.storage_usb_tests.ptp.base import *

test_result = SingleMethodRunner.run_single_test(StorageUSBPTPTests, "test_ptp_transfer_while_pull_push")

if test_result.wasSuccessful():
    print "TEST_PASSED"
else:
    TestUtils.print_test_result_problems(test_result)
    sys.exit("FAIL")
