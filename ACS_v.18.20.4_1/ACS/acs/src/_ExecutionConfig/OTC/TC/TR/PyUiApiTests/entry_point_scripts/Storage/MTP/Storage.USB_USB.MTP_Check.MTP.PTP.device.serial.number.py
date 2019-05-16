from _prerequisites import *
from PyUiApi.tests.storage_usb_tests.mtp.misc import *

test_result = SingleMethodRunner.run_single_test(StorageUSBMTPMiscTests, "test_mtp_ptp_device_serial_number")

if test_result.wasSuccessful():
    print "TEST_PASSED"
else:
    TestUtils.print_test_result_problems(test_result)
    sys.exit("FAIL")
