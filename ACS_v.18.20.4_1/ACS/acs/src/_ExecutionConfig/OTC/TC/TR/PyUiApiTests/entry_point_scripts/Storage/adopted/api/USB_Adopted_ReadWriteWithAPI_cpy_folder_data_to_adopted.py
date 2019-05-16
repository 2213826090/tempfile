from _prerequisites import *
from PyUiApi.tests.storage_usb_tests.with_api.adopted import *

test_result = SingleMethodRunner.run_single_test(StorageUSBTestsAdoptedWithApi, "test_copy_folder_with_file_from_data")

if test_result.wasSuccessful():
    print "PASS"
else:
    TestUtils.print_test_result_problems(test_result)
    sys.exit("FAIL")
