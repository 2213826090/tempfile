from _prerequisites import *
from PyUiApi.tests.system_os_tests import *
from PyUiApi.common.acs_utils import *

AcsUtils.copy_acs_artifact_to_host(SystemOsTests.gcm_sender_jar,
                                   Environment.tmp_dir_path)

test_result = SingleMethodRunner.run_single_test(SystemOsTests, "test_google_cloud_messaging_disable_wifi")

if test_result.wasSuccessful():
    print "TEST_PASSED"
else:
    TestUtils.print_test_result_problems(test_result)
    print "FAIL"
