from _prerequisites import *
from PyUiApi.tests.photos_tests import *

test_result = SingleMethodRunner.run_single_test(PhotosTests, "test_cancel_changes")

if test_result.wasSuccessful():
    print "PASS"
else:
    TestUtils.print_test_result_problems(test_result)
    print "FAIL"
