from _prerequisites import *
from PyUiApi.tests.houdini_tests import *

test_result = SingleMethodRunner.run_single_test(HoudiniTests, "test_run_houdini_test_tool")

if test_result.wasSuccessful():
    print "PASS"
else:
    TestUtils.print_test_result_problems(test_result)
    print "FAIL"

