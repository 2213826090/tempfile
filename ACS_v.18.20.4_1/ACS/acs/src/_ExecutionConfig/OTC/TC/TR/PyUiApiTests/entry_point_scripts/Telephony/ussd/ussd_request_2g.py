from _prerequisites import *
from PyUiApi.tests.telephony.ussd.ussd import *

global test_outcome
test_result = SingleMethodRunner.run_single_test(USSDTests, "test_ussd_request_2g")

if test_result.wasSuccessful():
    print "PASS"
    test_outcome = True
else:
    TestUtils.print_test_result_problems(test_result)
    print "FAIL"
    test_outcome = False
