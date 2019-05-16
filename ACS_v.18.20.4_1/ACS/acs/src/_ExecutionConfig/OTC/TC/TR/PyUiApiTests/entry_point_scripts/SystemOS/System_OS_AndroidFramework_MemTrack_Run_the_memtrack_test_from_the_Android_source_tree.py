from _prerequisites import *
from PyUiApi.tests.system_os_tests import *
from PyUiApi.common.acs_utils import *

AcsUtils.copy_acs_artifact_to_dut("memtrack_test", "/data/data/memtrack_test")
AdbUtils.run_adb_cmd("chmod 777 /data/data/memtrack_test")
test_result = SingleMethodRunner.run_single_test(SystemOsTests, "test_memtrack_test_from_android_tree")

if test_result.wasSuccessful():
    print "PASS"
else:
    TestUtils.print_test_result_problems(test_result)
    print "FAIL"

