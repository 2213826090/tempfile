from _prerequisites import *
from PyUiApi.tests.telephony.call_wait.call_wait_tests import *

global test_outcome
test_result = SingleMethodRunner.run_single_test(CallWaitTests2, "test_call_wait_incoming_call_ignored")

if test_result.wasSuccessful():
    print "PASS"
    test_outcome = True
else:
    TestUtils.print_test_result_problems(test_result)
    print "FAIL"
    test_outcome = False
