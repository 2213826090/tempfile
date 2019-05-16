from storage_multitask_with_alarm import *

#VERDICT = SUCCESS

test_result = SingleMethodRunner.run_single_test(StorageCheckMultiTask,
                                                 "test_multitasking_while_alarm_expire")

if test_result.wasSuccessful():
    #VERDICT = SUCCESS
    print "TEST PASSED"
else:
    # VERDICT = FAILURE
    print "TEST FAILED"