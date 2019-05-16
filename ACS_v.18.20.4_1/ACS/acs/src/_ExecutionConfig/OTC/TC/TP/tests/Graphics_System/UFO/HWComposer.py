# -*- coding: utf-8 -*-
from testlib.util.common import g_common_obj
from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.extend_camera_impl import CameraExtendImpl
from testlib.graphics.development_settings_impl import DevelopmentSettingsImpl
from testlib.graphics.photos_impl import get_photo_implement
from testlib.graphics.html5_impl import Html5Impl
from testlib.graphics.extend_chrome_impl import chrome_impl
from testlib.graphics.common import adb32, logcat, file_sys, dbsetting, wifi_ctrl
from testlib.system_touch.system_touch import SystemTouch
from testlib.graphics.special_actions_impl import special_actions
import time
import threading


class HWComposer(UIATestBase):

    def setUp(self):
        super(HWComposer, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        g_common_obj.root_on_device()
        self.d = g_common_obj.get_device()
        self._extendcamera = CameraExtendImpl()
        self.photoImpl = get_photo_implement()
        self.develop_settings = DevelopmentSettingsImpl()
        self.develop_settings.set_disable_hw_overlays(switch='ON')

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(HWComposer, self).tearDown()
        self.develop_settings.set_disable_hw_overlays(switch='OFF')

    def test_HWComposer_camera_preview(self):
        print "[RunTest]: %s" % self.__str__()
        self._extendcamera.picTake()
        g_common_obj.assert_exp_happens()
        self._extendcamera.delete_capture_pictures()

    def test_HWComposer_suspend_resume(self):
        '''
        Not using power button to suspend DUT since adb will disconnect.
        Replace with shell stop and start command to turn off / on display.
        '''
        adb32.refresh_ui()

    def test_HWComposer_video_playback(self):
        print "[RunTest]: %s" % self.__str__()
        Html5Impl().check_chrome_installed()
        video_path = '/sdcard/video.mp4'
        youtube_video_url, youtube_video_key = chrome_impl.get_youtube_url_key('american_bobtail',
                                                                               'key_american_bobtail')
        self.photoImpl.deploy_photo_content("Videos", "video_006", video_path)
        self.photoImpl.refresh_sdcard()
        self.photoImpl.play_video_command(video_path, 8)
        g_common_obj.assert_exp_happens()
        self.photoImpl.stop_photos_am()
        chrome_impl.launch()
        chrome_impl.chrome_setup()
        chrome_impl.open_website(youtube_video_url)
        chrome_impl.web_check(youtube_video_key, 15)

    def test_HWComposer_video_recording(self):
        print "[RunTest]: %s" % self.__str__()
        self._extendcamera.videoRecord()
        g_common_obj.assert_exp_happens()
        self._extendcamera.delete_captured_video()


class HWComposer2(UIATestBase):


    def setUp(self):
        super(HWComposer2, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        g_common_obj.root_on_device()
        self.d = g_common_obj.get_device()
        wifi_ctrl.turn_off()
        self.systemTouch = SystemTouch()
        self.develop_settings = DevelopmentSettingsImpl()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(HWComposer2, self).tearDown()
        dbsetting.set_color_corretion(False)
        dbsetting.set_color_inversion(False)
        self.develop_settings.set_input_options(switch='OFF')
        wifi_ctrl.turn_on()

    def test_HWComposer_2_0_Check_state(self):
        print "[RunTest]: %s" % self.__str__()
        special_actions.setup()

        def long_touch_operation():
            print "Starting long touch operation."
            special_actions.long_touch_on_1_1()
        self.develop_settings.set_disable_hw_overlays(switch='OFF')
        self.develop_settings.set_input_options(switch='ON')
        logcat.check_dumpsys_SurfaceFlinger_info(keyword="h/w composer enabled",
                                                 assertword="h/w composer enabled")
        primaryPlaneInfo = file_sys.get_file_context(matchCase=True,
                                                     file_path='/d/dri/0/i915_display_info',
                                                     keyword='Plane.*PRI',
                                                     extend=True,
                                                     ext_cmd="cut -d 'x' -f2 | cut -d '=' -f2")
        assert int(primaryPlaneInfo) > 0, "Invalid primary plane info."
        thread = threading.Timer(2, long_touch_operation)
        thread.start()
        check_list = []
        for i in range(6):
            time.sleep(.5)
            pointerPlaneInfo = file_sys.get_file_context(matchCase=True,
                                                         file_path='/d/dri/0/i915_display_info',
                                                         keyword='Plane.*CUR',
                                                         extend=True,
                                                         ext_cmd="cut -d 'x' -f2 | cut -d '=' -f2")
            check_list.append(pointerPlaneInfo)
        assert sum(int(i) for i in check_list) > 0, "Pointer info not detected."

    def test_HWComposer_2_0_Color_correction(self):
        print "[RunTest]: %s" % self.__str__()
        pre_chk = logcat.check_dumpsys_SurfaceFlinger_info(True, 'composer', 'enabled')
        assert pre_chk, "Color correction is already taking effect."
        dbsetting.set_color_corretion(True)
        assert dbsetting.get_color_corretion() == 1, "Fail to enable color correction."
        for i in [11, 12, 13]: # Three mode in color correction
            dbsetting.set_color_corretion_mode(i)
            assert dbsetting.get_color_corretion_mode() == i, "Fail to set color correction mode."
            count = 0
            for i in range(5):
                time.sleep(1)
                count += logcat.check_dumpsys_SurfaceFlinger_info(True, 'composer', 'disabled')
                if count > 0: break
            assert count > 0, "Color inversion isn't taking effect."

    def test_HWComposer_2_0_Color_inversion(self):
        print "[RunTest]: %s" % self.__str__()
        pre_chk = logcat.check_dumpsys_SurfaceFlinger_info(True, 'composer', 'enabled')
        assert pre_chk, "Color inversion is already taking effect."
        dbsetting.set_color_inversion(True)
        assert dbsetting.get_color_inversion() == 1, "Fail to enable color inversion."
        count = 0
        for i in range(5):
            time.sleep(1)
            count += logcat.check_dumpsys_SurfaceFlinger_info(True, 'composer', 'disabled')
            if count > 0: break
        assert count > 0, "Color inversion isn't taking effect."

