from _prerequisites import *
from PyUiApi.tests.system_os_tests import *
from PyUiApi.common.acs_utils import *


test_result = SingleMethodRunner.run_single_test(SystemOsTests, "test_hwui_optimization")

if test_result.wasSuccessful():
    print "TEST_PASSED"
else:
    TestUtils.print_test_result_problems(test_result)
    print "FAIL"
