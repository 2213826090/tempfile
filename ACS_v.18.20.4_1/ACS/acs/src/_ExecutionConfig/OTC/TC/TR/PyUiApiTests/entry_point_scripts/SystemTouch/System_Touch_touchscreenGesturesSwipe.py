from _prerequisites import *
from PyUiApi.tests.system_touch_tests import *

test_result = SingleMethodRunner.run_single_test(SystemTouchTests, "test_touchscreen_gestures_swipe")

if test_result.wasSuccessful():
    print "PASS"
else:
    TestUtils.print_test_result_problems(test_result)
    print "FAIL"
