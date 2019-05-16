from storage_fill_memory import *

#VERDICT = SUCCESS

test_result = SingleMethodRunner.run_single_test(StorageUSBFillTests,
                                                 "test_fill_internal_memory_install_apk")

if test_result.wasSuccessful():
    #VERDICT = SUCCESS
    print "TEST PASSED"
else:
    # VERDICT = FAILURE
    print "TEST FAILED"