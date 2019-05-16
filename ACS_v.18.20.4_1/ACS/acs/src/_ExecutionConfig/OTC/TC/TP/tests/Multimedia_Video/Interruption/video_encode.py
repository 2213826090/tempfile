# coding: utf-8
from testlib.camera.mum_camera_impl import CameraImpl
from testlib.photos.mum_photos_impl import PhotosImpl
from testlib.util.common import g_common_obj
from testlib.multimedia.testcasebase import TestCaseBase
from testlib.multimedia.multimedia_camera_helper import MultiMediaCameraHelper
from testlib.multimedia.multimedia_setting import MultiMediaSetting
import os
import time
from testlib.multimedia.multimedia_log import MultimediaLogger
logger = MultimediaLogger.instance()

class VideoEncode(TestCaseBase):
    """
    @summary: Test Video encode
    """

    def setUp(self):
        """
        @summary: set up
        @return: None
        """
        super(VideoEncode, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        g_common_obj.stop_app_am("com.google.android.GoogleCamera")
        # Unlock screen
        g_common_obj.adb_cmd_capture_msg("input keyevent 82")

    def tearDown(self):
        """
        @summary: tearDown
        @return: None
        """
        super(VideoEncode, self).tearDown()
        print "[Teardown]: %s" % self._test_name
        g_common_obj.stop_exp_handle()
        time.sleep(3)
        g_common_obj.stop_app_am("com.google.android.GoogleCamera")
        time.sleep(3)
        g_common_obj.adb_cmd_capture_msg(" rm -rf /sdcard/DCIM/Camera/*")
        time.sleep(10)
        g_common_obj.adb_cmd_capture_msg(self.video.cfg.get("refresh_sd"))

    def appPrepare(self, case_name):
        cfg_file = os.path.join(os.environ.get('TEST_DATA_ROOT', ''), \
            'tests.tablet.mum_auto_video.conf')
        self.video = PhotosImpl(\
            self.config.read(cfg_file, case_name))
        self.camera = CameraImpl(\
            self.config.read(cfg_file, case_name))
        self.multimedia_setting = MultiMediaSetting(cfg_file)
        g_common_obj.adb_cmd_capture_msg(" rm -rf /sdcard/DCIM/Camera/*")
        g_common_obj.adb_cmd_capture_msg(self.video.cfg.get("refresh_sd"))
        self.video.set_orientation_n()
        self.camera.clean_up_camera_data()

    def changeCameraTypeStr(self, camera_type):
        if "Back" in camera_type:
            return "Back"
        return "Front"

    def videoEncodeSettingResolutionThenPlayback(self, case_name):
        """
        This test used to test video encode then playback
        The test case spec is following:
        1. Launch camera
        2. set resolution
        3. record video
        4. playback
        """
        print "run case is " + str(case_name)
        self.appPrepare(case_name)
        self.multimedia_camera_helper = MultiMediaCameraHelper()
        self.multimedia_camera_helper.camera.startCameraApp()
        self.multimedia_camera_helper.camera.selectMode("Video")
        camera_type = self.changeCameraTypeStr(self.video.cfg.get("camera_type"))
        if "bxtp" in self.multimedia_setting.get_paltform_hardware():
            logger.debug("For BXT, do not switch Rear/Front")
        else:
            self.multimedia_camera_helper.camera.switchRearOrFront(camera_type)
        resolution = self.multimedia_camera_helper.changeResolution(self.camera.cfg.get("resolution"))
        self.multimedia_camera_helper.camera.setVideoResolution(resolution, camera_type)
        self.multimedia_camera_helper.camera.recordVideo(1, int(self.video.cfg.get("record_time")))
        self.multimedia_camera_helper.camera.reviewPhotoAndVideo()
        print "case " + str(case_name) + " is pass"

    def testVideoEncode_Longlasting_RearCamera_480p_30mins(self):
        """
        This test used to test video record and playback
        The test case spec is following:
        1. Former name: test_video_encode_playback_mum_002
        """
        self.videoEncodeSettingResolutionThenPlayback('mum_test_video_encode_playback_002')

    def testVideoEncode_Longlasting_RearCamera_720p_1280x720_30mins(self):
        """
        This test used to test video record and playback
        The test case spec is following:
        1. Former name: test_video_encode_playback_mum_003
        """
        self.videoEncodeSettingResolutionThenPlayback('mum_test_video_encode_playback_003')

    def testVideoEncode_Longlasting_RearCamera_1080p_1920x1080_30mins(self):
        """
        This test used to test video record and playback
        The test case spec is following:
        1. Former name: test_video_encode_playback_mum_004
        """
        self.videoEncodeSettingResolutionThenPlayback('mum_test_video_encode_playback_004')

    def testVideoEncode_Longlasting_RearCamera_720p_PlaybackCheck(self):
        """
        This test used to test video record and playback
        The test case spec is following:
        1. Former name: test_video_encode_playback_mum_008
        """
        self.videoEncodeSettingResolutionThenPlayback('mum_test_video_encode_playback_008')

    def test_video_encode_playback_mum_009(self):
        """
        This test used to test video record and playback
        The test case spec is following:
        """
        self.videoEncodeSettingResolutionThenPlayback('mum_test_video_encode_playback_009')

    def test_video_encode_playback_mum_010(self):
        """
        This test used to test video record and playback
        The test case spec is following:
        """
        self.videoEncodeSettingResolutionThenPlayback('mum_test_video_encode_playback_010')

    def testVideoEncode_Longlasting_RearCamera_720p_PlaybackInParallel(self):
        """
        This test used to test video record and playback
        The test case spec is following:
        1. Former name: test_video_encode_playback_mum_011
        """
        self.videoEncodeSettingResolutionThenPlayback('mum_test_video_encode_playback_011')

    def testVideoEncode_Longlasting_RearCamera_720p_30mins(self):
        """
        This test used to test video record and playback
        The test case spec is following:
        1. Former name: test_video_encode_playback_mum_012
        """
        self.videoEncodeSettingResolutionThenPlayback('mum_test_video_encode_playback_012')

    def testVideoEncode_Longlasting_RearCamera_1080p_10mins_PlaybackCheck(self):
        """
        This test used to test video record long lasting
        The test case spec is following:
        1. Former name: test_video_encode_playback_mum_020
        """
        self.videoEncodeSettingResolutionThenPlayback('mum_test_video_encode_playback_020')

    def testVideoEncode_Longlasting_FrontCamera_480p_10mins_PlaybackCheck(self):
        """
        This test used to test video record long lasting
        The test case spec is following:
        1. Former name: test_video_encode_playback_mum_021
        """
        self.videoEncodeSettingResolutionThenPlayback('mum_test_video_encode_playback_021')
