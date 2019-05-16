from _prerequisites import *
from PyUiApi.tests.photos_tests import *
from PyUiApi.common.acs_utils import *
from PyUiApi.common.environment_utils import *

wallpaper_file_path = "blue_wallpaper.jpg"
AcsUtils.copy_acs_artifact_to_dut(wallpaper_file_path, Environment.dcim_folder_path)

try:
    test_result = SingleMethodRunner.run_single_test(PhotosTestsM, "test_use_as_photo")
    if test_result.wasSuccessful():
        print "PASS"
    else:
        TestUtils.print_test_result_problems(test_result)
        sys.exit("FAIL")
finally:
    AdbUtils.delete_files(os.path.join(Environment.dcim_folder_path, wallpaper_file_path))

