from _prerequisites import *
from PyUiApi.tests.storage_usb_tests.generic import *
from PyUiApi.tests.storage_usb_tests.device_test_env import *

test_result = SingleMethodRunner.run_single_test(DeviceTestENV, "setting_device_env")

if test_result.wasSuccessful():
    print "PASSED"
else:
    TestUtils.print_test_result_problems(test_result)
    sys.exit("FAIL")