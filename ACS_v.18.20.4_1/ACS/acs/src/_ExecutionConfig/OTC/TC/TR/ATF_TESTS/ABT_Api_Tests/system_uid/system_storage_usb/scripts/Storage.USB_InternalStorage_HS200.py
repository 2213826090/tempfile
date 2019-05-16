from storage_usb_tests import *

VERDICT = SUCCESS

test_result = SingleMethodRunner.run_single_test(StorageUSBTests, "test_timing_spec_parameter")

if test_result.wasSuccessful():
    VERDICT = SUCCESS
else:
    VERDICT = FAILURE
