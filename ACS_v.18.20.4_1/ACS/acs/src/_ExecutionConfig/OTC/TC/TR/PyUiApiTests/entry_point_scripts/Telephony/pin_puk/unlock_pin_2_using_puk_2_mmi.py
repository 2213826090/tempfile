from _prerequisites import *
from PyUiApi.tests.telephony.pin_puk.pin_puk import *

global test_outcome
test_result = SingleMethodRunner.run_single_test(PINPUKTests, "test_unlock_pin_2_using_puk_2_MMI")

if test_result.wasSuccessful():
    print "PASS"
    test_outcome = True
else:
    TestUtils.print_test_result_problems(test_result)
    print "FAIL"
    test_outcome = False

