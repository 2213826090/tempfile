from _prerequisites import *
from PyUiApi.tests.chrome_tests import *

check_cmd = '''cat /data/local/chrome-command-line'''
prepare_cmd = '''echo "chrome --enable-webgl --ignore-gpu-blacklist --disable-gesture-requirement-for-media-playback" > /data/local/chrome-command-line'''

check = AdbUtils.run_adb_cmd(check_cmd)
if "ignore-gpu-blacklist" not in check:
    AdbUtils.run_adb_cmd(prepare_cmd)

test_result = SingleMethodRunner.run_single_test(ChromeTests, "test_html5_webgl")

if test_result.wasSuccessful():
    print "PASS"
else:
    TestUtils.print_test_result_problems(test_result)
    print "FAIL"
