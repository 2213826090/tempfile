from _prerequisites import *
from PyUiApi.tests.system_os_tests import *

test_result = SingleMethodRunner.run_single_test(SystemOsTests, "test_alarm_after_reboot")

if test_result.wasSuccessful():
    print "TEST_PASSED"
else:
    TestUtils.print_test_result_problems(test_result)
    print "FAIL"