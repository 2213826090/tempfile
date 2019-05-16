# coding: utf-8
import os
from testlib.photos.mum_photos_impl import PhotosImpl
from testlib.multimedia.testcasebase import TestCaseBase
from testlib.util.common import g_common_obj
from testlib.multimedia.multimedia_setting import MultiMediaSetting, MultiMediaHandle
from testaid.headset import Headset
import time


class VideoHeadsetTest(TestCaseBase):
    """
    @summary: Test Video PlayBack
    """

    def setUp(self):
        """
        @summary: set up
        @return: None
        """
        super(VideoHeadsetTest, self).setUp()
        self.d = g_common_obj.get_device()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        g_common_obj.stop_app_am("videoplayer.app.instrument.otc.intel.com.otcvideoplayer")
        # Unlock screen
        g_common_obj.adb_cmd_capture_msg("input keyevent 82")
        
        self.hs = Headset("/dev/ttyUSB0")
        self.hs.reset()

    def tearDown(self):
        """
        @summary: tear tearDown
        @return: None
        """
        super(VideoHeadsetTest, self).tearDown()
        print "[Teardown]: %s" % self._test_name
        g_common_obj.stop_exp_handle()
        time.sleep(3)
        g_common_obj.stop_app_am("videoplayer.app.instrument.otc.intel.com.otcvideoplayer")
        time.sleep(3)
        
        self.hs.reset()

    def appPrepare(self, case_name, model=1):
        self.cfg_file = os.path.join(os.environ.get('TEST_DATA_ROOT', ''), \
            'tests.tablet.mum_auto_video.conf')
        self.video = PhotosImpl(\
            self.config.read(self.cfg_file, case_name))
        g_common_obj.adb_cmd_capture_msg(" rm -rf /sdcard/DCIM/Camera/*")
        
        self.multimedia_handle = MultiMediaHandle()
        self.multimedia_setting = MultiMediaSetting(self.cfg_file)
        self.multimedia_setting.clearFolder(self.video.cfg.get("remove_video"))
        time.sleep(2)
        if model == 1:
            self.push_path = self.multimedia_setting.push_file(self.video.cfg.get("push_video"), self.video.cfg.get("datapath"))
            
        self.multimedia_setting.install_apk("video_apk")
        self.video.set_orientation_n()
        # Unlock screen
        g_common_obj.adb_cmd_capture_msg("input keyevent 82")

    def videoPlayBack(self, push_path=""):
        if push_path == "":
            push_path = self.push_path
        return self.multimedia_handle.videoPlayBack(push_path)

    def checkVideoPlayBack(self, s=60):
        return self.multimedia_handle.checkVideoPlayBack(s)
    
    def checkVideoPlayBackWithComparePicture(self, stoptime, bigfileskiptime=0):
        return self.multimedia_handle.checkVideoPlayBackWithComparePicture(stoptime, bigfileskiptime)

    def checkVideoPlayBackWithHeadset(self, case_name):
        print "run case is " + str(case_name)
        self.appPrepare(case_name)
        
        self.hs.plug_in()
        time.sleep(2)
        
        self.multimedia_handle.launchVideoApp()
        time.sleep(2)
        self.videoPlayBack()
        self.checkVideoPlayBack()
        time.sleep(1)
        
        assert self.rpc.isStreamActive("music", 500), "music not active when headset plug in"
        print "case " + str(case_name) + " is pass"

    def testVideo_Playback_1080P_Check_Audio_HS(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        3. Check headset
        """
        self.checkVideoPlayBackWithHeadset("test_API_video_playback_with_headset_001")

    def testVideo_Playback_720P_Check_Audio_HS(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        3. Check headset
        """
        self.checkVideoPlayBackWithHeadset("test_API_video_playback_with_headset_002")