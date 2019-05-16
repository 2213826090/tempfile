from storage_fill_memory import *

#VERDICT = SUCCESS

test_result = SingleMethodRunner.run_single_test(StorageUSBFillTests,
                                                 "test_open_all_apps")

if test_result.wasSuccessful():
    #VERDICT = SUCCESS
    print "TEST PASSED"
else:
    # VERDICT = FAILURE
    print "TEST FAILED"