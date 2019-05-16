from _prerequisites import *
from PyUiApi.tests.telephony.sms.sms import *

global test_outcome
test_result = SingleMethodRunner.run_single_test(SMSTests, "test_320_char_sms")

if test_result.wasSuccessful():
    print "PASS"
    test_outcome = True
else:
    TestUtils.print_test_result_problems(test_result)
    print "FAIL"
    test_outcome = False
