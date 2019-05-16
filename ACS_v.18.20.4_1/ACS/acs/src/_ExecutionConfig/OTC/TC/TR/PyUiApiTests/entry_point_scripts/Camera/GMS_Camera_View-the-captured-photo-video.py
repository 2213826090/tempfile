from _prerequisites import *
from PyUiApi.tests.camera_tests import *

test_result = SingleMethodRunner.run_single_test(CameraTests, "test_view_captured_photo")

if test_result.wasSuccessful():
    print "PASS"
else:
    TestUtils.print_test_result_problems(test_result)
    print "FAIL"
