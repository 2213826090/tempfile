from _prerequisites import *
from PyUiApi.tests.storage_usb_tests.gui import *

test_result = SingleMethodRunner.run_single_test(StorageUSBTestsGUI, "test_open_all_apps_same_time")

if test_result.wasSuccessful():
    print "PASS"
else:
    TestUtils.print_test_result_problems(test_result)
    sys.exit("FAIL")
