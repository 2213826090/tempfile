from _prerequisites import *
from PyUiApi.tests.storage_usb_tests.mtp.base import *

test_result = SingleMethodRunner.run_single_test(StorageUSBMTPTests, "test_usb_chooser_options")

if test_result.wasSuccessful():
    print "PASS"
else:
    TestUtils.print_test_result_problems(test_result)
    sys.exit("FAIL")
