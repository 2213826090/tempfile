from _prerequisites import *
from PyUiApi.tests.keyboard_tests import *

test_result = SingleMethodRunner.run_single_test(PinyinKeyboardTests, "test_lock_screen")

if test_result.wasSuccessful():
    print "PASS"
else:
    TestUtils.print_test_result_problems(test_result)
    print "FAIL"
