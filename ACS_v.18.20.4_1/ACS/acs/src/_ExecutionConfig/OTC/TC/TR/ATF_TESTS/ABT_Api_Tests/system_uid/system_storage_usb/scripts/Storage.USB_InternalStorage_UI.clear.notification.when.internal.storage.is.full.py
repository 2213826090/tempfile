from storage_fill_memory import *

#VERDICT = SUCCESS

test_result = SingleMethodRunner.run_single_test(StorageUSBFillTests,
                                                 "test_clear_internal_memory_full_notification")

if test_result.wasSuccessful():
    #VERDICT = SUCCESS
    print "TEST PASSED"
else:
    # VERDICT = FAILURE
    print "TEST FAILED"