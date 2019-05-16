from _prerequisites import *
from PyUiApi.tests.system_os_tests import *
if "RECORD_PYUIAPITESTS" in os.environ:
    del os.environ["RECORD_PYUIAPITESTS"]

test_result = SingleMethodRunner.run_single_test(SystemOsTests, "test_screen_recording")

if test_result.wasSuccessful():
    print "PASS"
else:
    TestUtils.print_test_result_problems(test_result)
    print "FAIL"
