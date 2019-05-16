from _prerequisites import *
from PyUiApi.tests.houdini_tests import *

test_result = SingleMethodRunner.run_single_test(HoudiniTests, "test_switch_between_arm_and_google_apps")

if test_result.wasSuccessful():
    print "PASS"
else:
    TestUtils.print_test_result_problems(test_result)
    print "FAIL"

