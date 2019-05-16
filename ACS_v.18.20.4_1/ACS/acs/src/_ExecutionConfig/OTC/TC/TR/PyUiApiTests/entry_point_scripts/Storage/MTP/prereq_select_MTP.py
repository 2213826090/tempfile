from _prerequisites import *
from PyUiApi.tests.storage_usb_tests.mtp.base import *

StorageUSBMTPTests.test_MTP_internal_storage_path = False
test_result = SingleMethodRunner.run_single_test(USBChooseMTP, "prereq_select_MTP")

if test_result.wasSuccessful():
    print "PASS"
else:
    TestUtils.print_test_result_problems(test_result)
    sys.exit("FAIL")
