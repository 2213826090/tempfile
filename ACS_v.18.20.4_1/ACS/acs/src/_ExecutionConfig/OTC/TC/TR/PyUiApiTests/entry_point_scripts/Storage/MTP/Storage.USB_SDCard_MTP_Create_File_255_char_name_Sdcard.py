from _prerequisites import *
from PyUiApi.tests.storage_usb_tests.mtp.portable import *

StorageUSBMTPTestsPortable.test_MTP_sd_card_path = True
test_result = SingleMethodRunner.run_single_test(StorageUSBMTPTestsPortable,
                                                 "test_create_file_255_chars_through_adb")

if test_result.wasSuccessful():
    print "PASS"
else:
    TestUtils.print_test_result_problems(test_result)
    sys.exit("FAIL")
