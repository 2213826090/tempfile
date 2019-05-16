from _prerequisites import *
from PyUiApi.tests.storage_usb_tests.mtp.portable import *

test_result = SingleMethodRunner.run_single_test(StorageUSBMTPTestsPortable,
                                                 "test_copy_file_from_host")

if test_result.wasSuccessful():
    print "PASS"
else:
    TestUtils.print_test_result_problems(test_result)
    sys.exit("FAIL")
