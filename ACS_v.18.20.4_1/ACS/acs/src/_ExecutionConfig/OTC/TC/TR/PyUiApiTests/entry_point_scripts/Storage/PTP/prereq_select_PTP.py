from _prerequisites import *
from PyUiApi.tests.storage_usb_tests.ptp.base import *

StorageUSBPTPTests.test_PTP_internal_storage_path = False
test_result = SingleMethodRunner.run_single_test(USBChoosePTP, "prereq_select_PTP")

if test_result.wasSuccessful():
    print "PASS"
else:
    TestUtils.print_test_result_problems(test_result)
    sys.exit("FAIL")
