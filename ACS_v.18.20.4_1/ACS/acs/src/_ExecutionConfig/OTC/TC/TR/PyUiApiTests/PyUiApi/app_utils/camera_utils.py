from PyUiApi.common.uiautomator_utils import *
from PyUiApi.common.environment_utils import Environment

CAMERA_STARTUP_TIMEOUT = 20
get_latest_image_data_cmd = '''am instrument -e class com.intel.test.apitests.tests.MediaStoreTestsDriver#testGetLatestImageData -w com.intel.test.apitests/com.intel.test.apitests.runners.GenericArgumentPassingTestRunner'''
get_latest_image_data_logcat = '''logcat -d | grep "found latest image $PROP$" '''
get_latest_video_data_cmd = '''am instrument -e class com.intel.test.apitests.tests.MediaStoreTestsDriver#testGetLatestVideoData -w com.intel.test.apitests/com.intel.test.apitests.runners.GenericArgumentPassingTestRunner'''
get_latest_video_data_logcat = '''logcat -d | grep "found latest video $PROP$" '''
test_image_cmd = '''am instrument -e class com.intel.test.apitests.tests.ImageCodecsTestsDriver#testCameraImage -e args 'cameraImagePath:$PATH$' -w com.intel.test.apitests/com.intel.test.apitests.runners.GenericArgumentPassingTestRunner'''
test_video_cmd = '''am instrument -e class com.intel.test.apitests.tests.VideoDecodingTestsDriver#testCameraVideo -e args 'cameraVideoPath:$PATH$' -w com.intel.test.apitests/com.intel.test.apitests.runners.GenericArgumentPassingTestRunner'''
path_cmd_template = "$PATH$"
prop_cmd_template = "$PROP$"

image_properties = [' ID', 'DATA', 'BUCKET_DISPLAY_NAME', 'DATE_TAKEN', 'MIME_TYPE', 'WIDTH', 'HEIGHT', 'ORIENTATION', 'SIZE']
video_properties = [' ID', 'DATA', 'BUCKET_DISPLAY_NAME', 'DATE_TAKEN', 'MIME_TYPE', 'WIDTH', 'HEIGHT', 'SIZE']

camera_launcher_string = "com.google.android.GoogleCamera/com.android.camera.CameraLauncher"


class Camera(object):
    @staticmethod
    def launch():
        AdbUtils.start_activity_from_shell(camera_launcher_string)
        camera_launched = Camera.wait_for_camera_launch()
        if not camera_launched:
            UiAutomatorUtils.launch_app_from_apps_menu(CAMERA_SHORTCUT_NAME)
            Camera.wait_for_camera_launch()

    @staticmethod
    def wait_for_camera_launch():
        for i in range(CAMERA_STARTUP_TIMEOUT):
            time.sleep(1)
            if d(text=CAMERA_ALLOW_LOCATION_ACCESS_TXT).wait.exists(timeout=1000):
                d(text=CAMERA_ALLOW_LOCATION_ACCESS_TXT).click()
            if d(resourceId=CAMERA_LAUNCH_CONFIRM_BUTTON_RESID).wait.exists(timeout=1000):
                vn = ViewNavigator()
                vn.nagivate_text(CAMERA_LAUNCH_CONFIRM_TXT_SEQ)
            if d(resourceId=CAMERA_SHUTTER_RESID).wait.exists(timeout=1000):
                time.sleep(3)
                if d(resourceId=CAMERA_SHUTTER_RESID).exists:
                    return True
        return False

    @staticmethod
    def press_shutter():
        if d(resourceId=CAMERA_SHUTTER_RESID).wait.exists(timeout=3000):
            d(resourceId=CAMERA_SHUTTER_RESID).click()
        time.sleep(3)

    @staticmethod
    def show_left_swipe_menu():
        ScreenSwiper.swipe_right()

    @staticmethod
    def open_picture_and_video_view():
        ScreenSwiper.swipe_left()

    @staticmethod
    def select_back_camera():
        if d(resourceId=CAMERA_MODE_OPTIONS_RESID).wait.exists(timeout=3000):
            d(resourceId=CAMERA_MODE_OPTIONS_RESID).click()
        if d(description=CAMERA_FRONT_DESC).wait.exists(timeout=3000):
            d(description=CAMERA_FRONT_DESC).click()

    @staticmethod
    def select_front_camera():
        if d(resourceId=CAMERA_MODE_OPTIONS_RESID).wait.exists(timeout=3000):
            d(resourceId=CAMERA_MODE_OPTIONS_RESID).click()
        if d(description=CAMERA_BACK_DESC).wait.exists(timeout=3000):
            d(description=CAMERA_BACK_DESC).click()

    @staticmethod
    def is_manual_exposure_available():
        if d(resourceId=CAMERA_MODE_OPTIONS_RESID).wait.exists(timeout=3000):
            d(resourceId=CAMERA_MODE_OPTIONS_RESID).click()
            if d(description=CAMERA_MANUAL_EXPOSURE_DESC).wait.exists(timeout=3000):
                return True
        return False

    @staticmethod
    def expand_exposure_compensation():
        if d(resourceId=CAMERA_MODE_OPTIONS_RESID).wait.exists(timeout=3000):
            d(resourceId=CAMERA_MODE_OPTIONS_RESID).click()
        if d(description=CAMERA_MANUAL_EXPOSURE_DESC).wait.exists(timeout=3000):
            d(description=CAMERA_MANUAL_EXPOSURE_DESC).click()

    @staticmethod
    def enable_manual_exposure():
        option_path = [CAMERA_ADVANCED_OPTIONS_TXT, CAMERA_MANUAL_EXPOSURE_OPTIONS_TXT]
        Camera.open_camera_settings()
        vc = ViewNavigator()
        vc.nagivate_text(option_path)
        time.sleep(2)


    @staticmethod
    def go_to_main_screen():
        while not d(resourceId=CAMERA_SHUTTER_RESID).wait.exists(timeout=3000):
            d.press.back()
        time.sleep(2)

    @staticmethod
    def open_media_details():
        if d(description=CAMERA_OPTIONS_DESC).wait.exists(timeout=3000):
            d(description=CAMERA_OPTIONS_DESC).click()
        if d(text=CAMERA_DETAILS_TXT).wait.exists(timeout=3000):
            d(text=CAMERA_DETAILS_TXT).click()
        time.sleep(2)

    @staticmethod
    def open_camera_settings():
        Camera.show_left_swipe_menu()
        if d(resourceId=CAMERA_SETTINGS_RESID).wait.exists(timeout=3000):
            d(resourceId=CAMERA_SETTINGS_RESID).click()

    @staticmethod
    def get_media_details():
        if d(resourceId=CAMERA_MEDIA_DETAIL_RESID).wait.exists(timeout=3000):
            details = d(resourceId=CAMERA_MEDIA_DETAIL_RESID)
            details_dict = {}
            for detail in details:
                info = detail.info["text"].split(":")
                info = [x for x in info if x != '']
                key = info[0].encode('ascii', 'ignore').replace('\n', '').strip()
                value = info[1].encode('ascii', 'ignore').replace('\n', '').strip()
                details_dict[key] = value
            return details_dict
        return None

    @staticmethod
    def get_available_options(options_menu_path):
        vc = ViewNavigator()
        vc.nagivate_text(options_menu_path)
        if d(resourceId=CAMERA_CONFIG_OPTIONS_RESID).wait.exists(timeout=3000):
            options = d(resourceId=CAMERA_CONFIG_OPTIONS_RESID)
            options_indexes = []
            default_option = None
            index = 0
            for option in options:
                if option.info["checked"]:
                    default_option = CameraOption(option.info["text"], index)
                else:
                    options_indexes.append(CameraOption(option.info["text"], index))
                index += 1
            return default_option, options_indexes
        return None


    @staticmethod
    def select_option(options_menu_path, option):
        vc = ViewNavigator()
        vc.nagivate_text(options_menu_path)
        if d(resourceId=CAMERA_CONFIG_OPTIONS_RESID).wait.exists(timeout=3000):
            options = d(resourceId=CAMERA_CONFIG_OPTIONS_RESID)
            options[option.index].click()
            return True
        return False

    @staticmethod
    def select_picture_mode():
        Camera.show_left_swipe_menu()
        if d(text=LEFT_SWIPE_MENU_CAMERA_TXT).wait.exists(timeout=5000):
            d(text=LEFT_SWIPE_MENU_CAMERA_TXT).click()
            time.sleep(2)

    @staticmethod
    def select_video_mode():
        Camera.show_left_swipe_menu()
        if d(text=LEFT_SWIPE_MENU_VIDEO_TXT).wait.exists(timeout=5000):
            d(text=LEFT_SWIPE_MENU_VIDEO_TXT).click()
            time.sleep(2)

    @staticmethod
    def get_latest_image_props():
        AdbUtils.run_adb_cmd(get_latest_image_data_cmd)
        output = AdbUtils.run_adb_cmd(get_latest_image_data_logcat.replace(prop_cmd_template, ''))
        lines = output.split('\n')
        lines = [x for x in lines if x != '']
        image_props = {}
        if "no hits" in lines[-1]:
            return None  # there is no image in media store
        else:
            for i in range(-1, -len(image_properties) - 1, -1):
                for prop in image_properties:
                    if prop in lines[i]:
                        value = lines[i].split(':')[-1].strip()
                        image_props[prop] = value
        print "latest image props: ", image_props
        return image_props

    @staticmethod
    def get_latest_video_props():
        AdbUtils.run_adb_cmd(get_latest_video_data_cmd)
        output = AdbUtils.run_adb_cmd(get_latest_video_data_logcat.replace(prop_cmd_template, ''))
        lines = output.split('\n')
        lines = [x for x in lines if x != '']
        video_props = {}
        if "no hits" in lines[-1]:
            return None  # there is no image in media store
        else:
            for i in range(-1, -len(video_properties) - 1, -1):
                for prop in image_properties:
                    if prop in lines[i]:
                        value = lines[i].split(':')[-1].strip()
                        video_props[prop] = value
        print "latest video props: ", video_props
        return video_props

    @staticmethod
    def is_image_valid(image_path):
        output = AdbUtils.run_adb_cmd(test_image_cmd.replace(path_cmd_template, image_path))
        return "OK" in output

    @staticmethod
    def is_video_valid(video_path):
        output = AdbUtils.run_adb_cmd(test_video_cmd.replace(path_cmd_template, video_path))
        return "OK" in output


class QuickCamera(object):
    @staticmethod
    def launch_photo_camera():
        AdbUtils.run_adb_cmd("am start -a android.media.action.STILL_IMAGE_CAMERA")
        QuickCamera.choose_photo_action_app()
        Camera.wait_for_camera_launch()
        QuickCamera.wait_for_shutter()

    @staticmethod
    def launch_video_camera():
        AdbUtils.run_adb_cmd("am start -a android.media.action.VIDEO_CAPTURE")
        QuickCamera.choose_video_action_app()
        Camera.wait_for_camera_launch()
        QuickCamera.wait_for_shutter()

    @staticmethod
    def choose_photo_action_app():
        if d(textContains=COMPLETE_ACTION_USING).wait.exists(timeout=3000):
            if d(text=CAMERA_PACKAGE).wait.exists(timeout=3000):
                d(text=CAMERA_PACKAGE).click()
            if d(text=COMPLETE_ACTION_ONCE).wait.exists(timeout=3000):
                d(text=COMPLETE_ACTION_ONCE).click()

    @staticmethod
    def choose_video_action_app():
        if d(textContains=COMPLETE_ACTION_USING).wait.exists(timeout=3000):
            if d(text="Camcorder").wait.exists(timeout=3000):
                d(text="Camcorder").click()
            if d(text=COMPLETE_ACTION_ONCE).wait.exists(timeout=3000):
                d(text=COMPLETE_ACTION_ONCE).click()

    @staticmethod
    def wait_for_shutter():
        d(descriptionContains="Shutter").wait.exists(timeout=3000)

    @staticmethod
    def validate_capture():
        if d(descriptionContains="Done").wait.exists(timeout=3000):
            d(descriptionContains="Done").click()
        time.sleep(2)

    @staticmethod
    def take_photos(nr_of_photos=1):
        QuickCamera.launch_photo_camera()
        for i in range(nr_of_photos):
            d.press.camera()
            QuickCamera.validate_capture()
            QuickCamera.wait_for_shutter()
        UiAutomatorUtils.close_all_tasks()

    @staticmethod
    def take_video(nr_of_videos=1, rec_time_secs=5):
        for i in range(nr_of_videos):
            QuickCamera.launch_video_camera()
            d.press.camera()
            time.sleep(rec_time_secs)
            d.press.camera()
            QuickCamera.validate_capture()
            QuickCamera.wait_for_shutter()
        UiAutomatorUtils.close_all_tasks()


class CameraFiles(object):
    @staticmethod
    def get_camera_pictures():
        list_dcim_pics_cmd = "ls -lR %s | grep .jp" % Environment.dcim_folder_path
        dcim_pics = AdbUtils.run_adb_cmd(list_dcim_pics_cmd, add_ticks=False)
        pictures = []
        for pic_info in dcim_pics.splitlines():
            pictures.append(pic_info.split()[-1])
        return pictures


class CameraOption(object):
    def __init__(self, text, index):
        self.text = text
        self.index = index

if __name__ == "__main__":
    # Camera.launch()
    # Camera.select_picture_mode()
    # Camera.select_video_mode()
    # props = Camera.get_latest_image_props()
    # print props
    # print Camera.is_image_valid(props["DATA"])
    # props = Camera.get_latest_video_props()
    # print props
    # print Camera.is_video_valid(props["DATA"])
    Camera.open_camera_settings()
    do, oo = Camera.get_available_options([CAMERA_RES_QUALITY_MENU_TXT, CAMERA_FRONT_VID_OPTS_TXT])