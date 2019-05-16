from _prerequisites import *
from PyUiApi.common.acs_interface import *
try:
    acquire_acs_logger(ACS_LOGGER)
except:
    pass

from PyUiApi.tests.adb_tests import *

test_result = SingleMethodRunner.run_single_test(AdbTests, "test_get_serialno")

if test_result.wasSuccessful():
    print "PASS"
    test_outcome = True
else:
    TestUtils.print_test_result_problems(test_result)
    print "FAIL"

