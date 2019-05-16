from _prerequisites import *
from PyUiApi.tests.storage_usb_tests.generic import *
import sys
test_result1 = SingleMethodRunner.run_single_test(StorageUSBTests, "test_format_as_adoptable_without_migrate")
test_result2 = SingleMethodRunner.run_single_test(StorageUSBTests, "test_format_as_portable")
test_result3 = SingleMethodRunner.run_single_test(StorageUSBTests, "test_format_as_adoptable_migrate")
test_result4 = SingleMethodRunner.run_single_test(StorageUSBTests, "test_format_as_portable")
if test_result1.wasSuccessful() and test_result2.wasSuccessful() and test_result3.wasSuccessful() and test_result4.wasSuccessful():
    print "PASS"
else:
    TestUtils.print_test_result_problems(test_result)
    sys.exit("FAIL")
