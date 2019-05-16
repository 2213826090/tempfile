from _prerequisites import *
from PyUiApi.tests.keyboard_tests import *

test_result = SingleMethodRunner.run_single_test(JapaneseKeyboardTests, "test_switch_between_japanese_english")

if test_result.wasSuccessful():
    print "PASS"
else:
    TestUtils.print_test_result_problems(test_result)
    print "FAIL"
