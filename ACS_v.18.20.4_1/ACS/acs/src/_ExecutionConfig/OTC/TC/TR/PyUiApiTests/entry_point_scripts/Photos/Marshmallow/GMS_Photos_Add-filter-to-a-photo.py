from _prerequisites import *
from PyUiApi.tests.photos_tests import *

test_result = SingleMethodRunner.run_single_test(PhotosTestsM, "test_apply_filter_to_photo")

if test_result.wasSuccessful():
    print "PASS"
else:
    TestUtils.print_test_result_problems(test_result)
    sys.exit("FAIL")
