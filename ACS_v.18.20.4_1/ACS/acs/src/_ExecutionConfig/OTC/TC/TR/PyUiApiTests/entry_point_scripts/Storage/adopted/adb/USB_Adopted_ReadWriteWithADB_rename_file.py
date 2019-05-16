from _prerequisites import *
from PyUiApi.tests.storage_usb_tests.with_adb.adopted import *

test_result = SingleMethodRunner.run_single_test(StorageUSBTestsAdoptedWithAdb, "test_rename_file")

if test_result.wasSuccessful():
    print "PASS"
else:
    TestUtils.print_test_result_problems(test_result)
    sys.exit("FAIL")

