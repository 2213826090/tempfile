from _prerequisites import *
from PyUiApi.tests.storage_usb_tests.context import *

test_result = SingleMethodRunner.run_single_test(StorageUSBTestsContext, "test_context_portable_to_adopted")

if test_result.wasSuccessful():
    print "PASS"
else:
    TestUtils.print_test_result_problems(test_result)
    sys.exit("FAIL")
