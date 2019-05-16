from _prerequisites import *
from PyUiApi.tests.storage_usb_tests.ptp.base import *

test_result = SingleMethodRunner.run_single_test(StorageUSBPTPTests, "test_import_pictures_from_dut_to_pc")

if test_result.wasSuccessful():
    print "PASS"
else:
    TestUtils.print_test_result_problems(test_result)
    sys.exit("FAIL")
