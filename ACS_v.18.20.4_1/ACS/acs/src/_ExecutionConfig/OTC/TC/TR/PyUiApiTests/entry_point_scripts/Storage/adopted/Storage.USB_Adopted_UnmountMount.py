from _prerequisites import *
from PyUiApi.tests.storage_usb_tests.gui import *

test_result = SingleMethodRunner.run_single_test(StorageUSBTestsGUI, "test_adopted_unmount_mount")

if test_result.wasSuccessful():
    print "TEST_PASSED"
else:
    TestUtils.print_test_result_problems(test_result)
    sys.exit("FAIL")