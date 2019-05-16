from storage_sdcard_media import *

#VERDICT = SUCCESS

test_result = SingleMethodRunner.run_single_test(Storage_sdcard_media_check,
                                                 "test_sdcard_media_check")

if test_result.wasSuccessful():
    #VERDICT = SUCCESS
    print "TEST PASSED"
else:
    # VERDICT = FAILURE
    print "TEST FAILED"