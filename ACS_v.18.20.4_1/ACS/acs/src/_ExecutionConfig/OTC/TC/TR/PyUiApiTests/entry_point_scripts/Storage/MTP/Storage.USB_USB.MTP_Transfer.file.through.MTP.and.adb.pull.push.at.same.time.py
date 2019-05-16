from _prerequisites import *
from PyUiApi.tests.storage_usb_tests.mtp.base import *

test_result = SingleMethodRunner.run_single_test(StorageUSBMTPTests, "test_mtp_transfer_while_adb_pull_push")

if test_result.wasSuccessful():
    print "TEST_PASSED"
else:
    TestUtils.print_test_result_problems(test_result)
    sys.exit("FAIL")
