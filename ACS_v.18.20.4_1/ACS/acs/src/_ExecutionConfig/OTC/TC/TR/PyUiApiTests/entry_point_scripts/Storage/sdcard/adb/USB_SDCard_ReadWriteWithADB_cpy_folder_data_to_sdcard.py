from _prerequisites import *
from PyUiApi.tests.storage_usb_tests.with_adb.portable import *

test_result = SingleMethodRunner.run_single_test(StorageUSBTestsPortableWithAdb, "test_copy_folder_from_data")

if test_result.wasSuccessful():
    print "PASS"
else:
    TestUtils.print_test_result_problems(test_result)
    sys.exit("FAIL")


