from _prerequisites import *
from PyUiApi.tests.storage_usb_tests.generic import *


test_result = SingleMethodRunner.run_single_test(StorageUSBTests, "test_copy_emulated_from_physical")

if test_result.wasSuccessful():
    print "PASS"
else:
    TestUtils.print_test_result_problems(test_result)
    sys.exit("FAIL")
