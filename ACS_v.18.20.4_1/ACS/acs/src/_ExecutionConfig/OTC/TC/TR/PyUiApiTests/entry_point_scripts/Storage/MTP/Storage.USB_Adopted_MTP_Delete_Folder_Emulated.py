from _prerequisites import *
from PyUiApi.tests.storage_usb_tests.mtp.adopted import *

test_result = SingleMethodRunner.run_single_test(StorageUSBMTPTestsAdopted, "test_delete_folder")

if test_result.wasSuccessful():
    print "PASS"
else:
    TestUtils.print_test_result_problems(test_result)
    sys.exit("FAIL")
