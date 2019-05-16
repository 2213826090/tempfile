from _prerequisites import *
from PyUiApi.tests.photos_tests import *
from PyUiApi.common.acs_utils import *
from PyUiApi.common.environment_utils import *

image_file_name = "blue_wallpaper.jpg"
video_file_name = "H264_720x480_25fps.MP4"
AcsUtils.copy_acs_artifact_to_dut(image_file_name, Environment.dcim_folder_path)
AcsUtils.copy_acs_artifact_to_dut(video_file_name, Environment.dcim_folder_path)

try:
    test_result = SingleMethodRunner.run_single_test(PhotosTestsM, "test_delete_photo_movie")
    if test_result.wasSuccessful():
        print "PASS"
    else:
        TestUtils.print_test_result_problems(test_result)
        sys.exit("FAIL")
finally:
    AdbUtils.delete_files(os.path.join(Environment.dcim_folder_path, image_file_name))
    AdbUtils.delete_files(os.path.join(Environment.dcim_folder_path, video_file_name))

