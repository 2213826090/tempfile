from _prerequisites import *
from PyUiApi.tests.storage_usb_tests.mtp.misc import *

test_result = SingleMethodRunner.run_single_test(StorageUSBMTPMiscTests,
                                                 "test_adb_shell_connect_two_or_more_device")

if test_result.wasSuccessful():
    print "TEST_PASSED"
else:
    TestUtils.print_test_result_problems(test_result)
    sys.exit("FAIL")
