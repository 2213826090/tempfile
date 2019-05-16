from _prerequisites import *
from PyUiApi.tests.telephony.pdp.pdp import *

global test_outcome
test_result = SingleMethodRunner.run_single_test(PDPTests, "test_internal_pdp_with_apn_in_acl")

if test_result.wasSuccessful():
    print "PASS"
    test_outcome = True
else:
    TestUtils.print_test_result_problems(test_result)
    print "FAIL"
    test_outcome = False
