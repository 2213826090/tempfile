from _prerequisites import *
from PyUiApi.tests.storage_usb_tests.mtp.portable import *

test_result = SingleMethodRunner.run_single_test(StorageUSBMTPTestsPortable,
                                                 "test_delete_from_MTP_to_create_free_space_for_app")

if test_result.wasSuccessful():
    print "TEST_PASSED"
else:
    TestUtils.print_test_result_problems(test_result)
    sys.exit("FAIL")
