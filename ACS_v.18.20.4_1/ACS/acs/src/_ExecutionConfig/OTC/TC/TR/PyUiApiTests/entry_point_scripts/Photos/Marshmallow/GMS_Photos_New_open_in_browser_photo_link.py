from _prerequisites import *
from PyUiApi.tests.photos_tests import *
from PyUiApi.common.acs_utils import *
from PyUiApi.common.environment_utils import *
from PyUiApi.adb_helper.media_scanner import *

wallpaper_file_name = "blue_wallpaper.jpg"
wallpaper_file_path = os.path.join(Environment.dcim_folder_path, wallpaper_file_name)
AcsUtils.copy_acs_artifact_to_dut(wallpaper_file_name, Environment.dcim_folder_path)
MediaScanner.scan_single_file(wallpaper_file_path)

try:
    test_result = SingleMethodRunner.run_single_test(PhotosTestsM, "test_open_in_browser_photolink")
    if test_result.wasSuccessful():
        print "PASS"
    else:
        TestUtils.print_test_result_problems(test_result)
        sys.exit("FAIL")
finally:
    AdbUtils.delete_files(wallpaper_file_path)
