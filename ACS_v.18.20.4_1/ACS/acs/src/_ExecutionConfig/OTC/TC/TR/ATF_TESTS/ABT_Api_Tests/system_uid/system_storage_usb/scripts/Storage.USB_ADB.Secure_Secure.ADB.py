from storage_usb_adb_support import *

test_result = SingleMethodRunner.run_single_test(StorageUSBAdbTests, "test_adb_secure")

if test_result.wasSuccessful():
    print "Test PASSED"

else:
    print "Test FAILED"

