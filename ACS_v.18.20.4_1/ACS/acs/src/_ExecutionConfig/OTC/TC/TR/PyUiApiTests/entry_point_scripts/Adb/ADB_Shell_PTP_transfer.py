from _prerequisites import *
from PyUiApi.tests.adb_tests import *

test_result = SingleMethodRunner.run_single_test(AdbPTPTests, "test_adb_shell_ptp_transfer")

if test_result.wasSuccessful():
    print "PASS"
else:
    TestUtils.print_test_result_problems(test_result)
    print "FAIL"

