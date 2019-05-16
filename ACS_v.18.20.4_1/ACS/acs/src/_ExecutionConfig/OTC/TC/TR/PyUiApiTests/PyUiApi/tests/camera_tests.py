from PyUiApi.app_utils.camera_utils import *
from PyUiApi.common.test_utils import *

MEDIA_PATH_ID = "DATA"
MEDIA_DATE_ID = "DATE_TAKEN"
MEDIA_HEIGHT_ID = "HEIGHT"
MEDIA_WIDTH_ID = "WIDTH"
MEDIA_ORIENTATION_ID = "ORIENTATION"

IMG_DETAILS_PATH_KEY = "Path"
video_heights = {'480p': '480', '720p': '720', '1080p': '1080'}


class CameraTests(unittest.TestCase):
    initial_latest_image_props = None
    initial_latest_vid_props = None
    initial_orientation = None

    def setUp(self):
        UiAutomatorUtils.unlock_screen()
        self.initial_orientation = d.orientation
        self.initial_latest_image_props = Camera.get_latest_image_props()
        self.initial_latest_vid_props = Camera.get_latest_video_props()
        Camera.launch()

    def tearDown(self):
        self.log_before_cleanup()
        if d.orientation != self.initial_orientation:
            d.orientation = self.initial_orientation
            d.freeze_rotation(False)
        UiAutomatorUtils.close_all_tasks()

    def log_before_cleanup(self):
        result = self._resultForDoCleanups
        if not result.wasSuccessful():
            TestUtils.log_screenshot_and_dump()

    def is_video_resolution_valid(self, option, video_props):
        for res in video_heights.keys():
            if res in option.text:
                return video_props[MEDIA_HEIGHT_ID] == video_heights[res]
        return True

    def is_image_resolution_valid(self, option, image_props):
        width = int(image_props[MEDIA_WIDTH_ID])
        height = int(image_props[MEDIA_HEIGHT_ID])
        mpixel_tenths = (width * height) / 100000
        mpx_value_1 = (mpixel_tenths - 1) / 10.0
        mpx_value_2 = mpixel_tenths / 10.0
        mpx_value_3 = (mpixel_tenths + 1) / 10.0
        print "computed megapixel middle value: ", mpx_value_2
        return str(mpx_value_1) in option.text or str(mpx_value_2) in option.text or \
            str(mpx_value_3) in option.text

    def is_image_valid(self, image_props):
        if self.initial_latest_image_props is None:
            return Camera.is_image_valid(image_props[MEDIA_PATH_ID])
        else:
            if int(self.initial_latest_image_props[MEDIA_DATE_ID]) > int(image_props[MEDIA_DATE_ID]):
                return False
            else:
                return Camera.is_image_valid(image_props[MEDIA_PATH_ID])

    def is_video_valid(self, video_props):
        if self.initial_latest_vid_props is None:
            return Camera.is_image_valid(video_props[MEDIA_PATH_ID])
        else:
            if int(self.initial_latest_vid_props[MEDIA_DATE_ID]) > int(video_props[MEDIA_DATE_ID]):
                return False
            else:
                return Camera.is_video_valid(video_props[MEDIA_PATH_ID])

    def test_take_a_photo(self):
        Camera.select_picture_mode()
        Camera.press_shutter()
        image_props = Camera.get_latest_image_props()
        is_valid = self.is_image_valid(image_props)
        self.assertTrue(is_valid)

    def test_take_video(self):
        Camera.select_video_mode()
        Camera.press_shutter()
        time.sleep(10)  # record video for 10 seconds
        Camera.press_shutter()
        video_props = Camera.get_latest_video_props()
        is_valid = self.is_video_valid(video_props)
        self.assertTrue(is_valid)

    def test_view_captured_photo(self):
        Camera.select_picture_mode()
        Camera.press_shutter()
        image_props = Camera.get_latest_image_props()
        is_valid = self.is_image_valid(image_props)
        self.assertTrue(is_valid)
        Camera.open_picture_and_video_view()
        Camera.open_media_details()
        details = Camera.get_media_details()
        print "ui details:", details
        self.assertTrue(details[IMG_DETAILS_PATH_KEY] == image_props[MEDIA_PATH_ID])
        Camera.go_to_main_screen()

    def test_set_video_size_front_camera(self):
        menu_path = [CAMERA_RES_QUALITY_MENU_TXT, CAMERA_FRONT_VID_OPTS_TXT]
        Camera.select_video_mode()
        Camera.select_front_camera()
        Camera.open_camera_settings()
        default, opts = Camera.get_available_options(menu_path)
        Camera.go_to_main_screen()
        opts.append(default)  # append default option at the end
        for option in opts:
            print "current size: ", str(option.text)
            Camera.open_camera_settings()
            Camera.select_option(menu_path, option)
            Camera.go_to_main_screen()
            Camera.press_shutter()
            time.sleep(5)  # record video for 5 seconds
            Camera.press_shutter()
            video_props = Camera.get_latest_video_props()
            is_valid = self.is_video_valid(video_props)
            self.assertTrue(is_valid)
            is_valid = self.is_video_resolution_valid(option, video_props)
            self.assertTrue(is_valid)
            self.initial_latest_vid_props = video_props

    def test_set_video_size_back_camera(self):
        menu_path = [CAMERA_RES_QUALITY_MENU_TXT, CAMERA_BACK_VID_OPTS_TXT]
        Camera.select_video_mode()
        Camera.select_back_camera()
        Camera.open_camera_settings()
        default, opts = Camera.get_available_options(menu_path)
        Camera.go_to_main_screen()
        opts.append(default)  # append default option at the end
        for option in opts:
            print "current size: ", str(option.text)
            Camera.open_camera_settings()
            Camera.select_option(menu_path, option)
            Camera.go_to_main_screen()
            Camera.press_shutter()
            time.sleep(5)  # record video for 5 seconds
            Camera.press_shutter()
            video_props = Camera.get_latest_video_props()
            is_valid = self.is_video_valid(video_props)
            self.assertTrue(is_valid)
            is_valid = self.is_video_resolution_valid(option, video_props)
            self.assertTrue(is_valid)
            self.initial_latest_vid_props = video_props

    def test_set_image_size_back_camera(self):
        menu_path = [CAMERA_RES_QUALITY_MENU_TXT, CAMERA_BACK_IMG_OPTS_TXT]
        Camera.select_picture_mode()
        Camera.select_back_camera()
        Camera.open_camera_settings()
        default, opts = Camera.get_available_options(menu_path)
        Camera.go_to_main_screen()
        opts.append(default)  # append default option at the end
        for option in opts:
            print "current size: ", str(option.text)
            Camera.open_camera_settings()
            Camera.select_option(menu_path, option)
            Camera.go_to_main_screen()
            Camera.press_shutter()
            image_props = Camera.get_latest_image_props()
            is_valid = self.is_image_valid(image_props)
            self.assertTrue(is_valid)
            is_valid = self.is_image_resolution_valid(option, image_props)
            self.assertTrue(is_valid)
            self.initial_latest_image_props = image_props

    def test_set_image_size_front_camera(self):
        menu_path = [CAMERA_RES_QUALITY_MENU_TXT, CAMERA_FRONT_IMG_OPTS_TXT]
        Camera.select_picture_mode()
        Camera.select_front_camera()
        Camera.open_camera_settings()
        default, opts = Camera.get_available_options(menu_path)
        Camera.go_to_main_screen()
        opts.append(default)  # append default option at the end
        for option in opts:
            print "current size: ", str(option.text)
            Camera.open_camera_settings()
            Camera.select_option(menu_path, option)
            Camera.go_to_main_screen()
            Camera.press_shutter()
            image_props = Camera.get_latest_image_props()
            is_valid = self.is_image_valid(image_props)
            self.assertTrue(is_valid)
            is_valid = self.is_image_resolution_valid(option, image_props)
            self.assertTrue(is_valid)
            self.initial_latest_image_props = image_props

    def test_set_manual_exposure(self):
        Camera.select_picture_mode()
        if not Camera.is_manual_exposure_available():
            Camera.enable_manual_exposure()
        Camera.go_to_main_screen()
        for exposure_resid in CAMERA_EXPOSURE_COMPENSATION_RESIDS:
            Camera.expand_exposure_compensation()
            if d(resourceId=exposure_resid).wait.exists(timeout=2000):
                d(resourceId=exposure_resid).click()
            Camera.press_shutter()
            image_props = Camera.get_latest_image_props()
            is_valid = self.is_image_valid(image_props)
            self.assertTrue(is_valid)
            self.initial_latest_image_props = image_props

    def test_rotate_screen_in_preview_mode(self):
        Camera.select_picture_mode()
        OrientationChanger.change_orientation("r")
        d.freeze_rotation()
        time.sleep(3)
        OrientationChanger.change_orientation("n")
        d.freeze_rotation()
        time.sleep(3)
        Camera.select_video_mode()
        OrientationChanger.change_orientation("l")
        d.freeze_rotation()
        time.sleep(3)
        Camera.open_camera_settings()  # check to see if camera is stable and settings can be opened

if __name__ == "__main__":
    # test_result = SingleMethodRunner.run_single_test(CameraTests, "test_take_a_photo")
    # test_result = SingleMethodRunner.run_single_test(CameraTests, "test_take_video")
    test_result = SingleMethodRunner.run_single_test(CameraTests, "test_rotate_screen_in_preview_mode")
    print test_result