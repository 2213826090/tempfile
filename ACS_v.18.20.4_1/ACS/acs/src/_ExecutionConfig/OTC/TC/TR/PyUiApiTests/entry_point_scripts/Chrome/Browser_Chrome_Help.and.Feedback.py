from _prerequisites import *
from PyUiApi.tests.chrome_tests import *

test_result = SingleMethodRunner.run_single_test(ChromeTests, "test_help_and_feedback")

if test_result.wasSuccessful():
    print "PASS"
else:
    TestUtils.print_test_result_problems(test_result)
    print "FAIL"

