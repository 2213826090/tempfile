from _prerequisites import *
from PyUiApi.tests.houdini_tests import *

test_result = SingleMethodRunner.run_single_test(HoudiniTests, "test_install_houdini_application_with_adb")

if test_result.wasSuccessful():
    print "PASS"
else:
    TestUtils.print_test_result_problems(test_result)
    print "FAIL"
