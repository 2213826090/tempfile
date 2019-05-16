from _prerequisites import *
from PyUiApi.tests.telephony.pdp.pdp import *

global test_outcome
test_result = SingleMethodRunner.run_single_test(PDPTests, "test_call_after_reboot_pdp_activated_3G")

if test_result.wasSuccessful():
    print "PASS"
    test_outcome = True
else:
    TestUtils.print_test_result_problems(test_result)
    print "FAIL"
    test_outcome = False

