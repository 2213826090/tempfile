from _prerequisites import *
from PyUiApi.tests.storage_usb_tests.generic import *
from PyUiApi.tests.storage_usb_tests.shell_resource_execute import *

ShellResourceExecuter.shell_file = './scripts/check_adb_shell_commands.sh'
test_result = SingleMethodRunner.run_single_test(ShellResourceExecuter, "excute_shell_file")

if test_result.wasSuccessful():
    print "TEST_PASSED"
else:
    TestUtils.print_test_result_problems(test_result)
    sys.exit("FAIL")