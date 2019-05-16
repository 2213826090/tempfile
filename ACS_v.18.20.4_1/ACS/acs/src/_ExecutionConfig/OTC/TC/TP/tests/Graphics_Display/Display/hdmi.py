# -*- coding: utf-8 -*-
'''
@summary: HDMI test
@since: 09/18/2017
@author: Rui
'''
from testlib.graphics.sample_apidemo import SampleApiDemoImpl
from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.common import multi_display, logcat, adb32, g_common_obj
from testlib.graphics.development_settings_impl import DevelopmentSettingsImpl
from testlib.graphics.html5_impl import html5
from testlib.graphics.extend_chrome_impl import chrome_impl
import time


class HDMI(UIATestBase):

    @classmethod
    def setUpClass(self):
        super(HDMI, self).setUpClass()
        SampleApiDemoImpl().install_apk()

    def setUp(self):
        super(HDMI, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.apiDemoImpl = SampleApiDemoImpl()
        self.develop_settings = DevelopmentSettingsImpl()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(HDMI, self).tearDown()
        self.apiDemoImpl.stop_app_am()
        self.apiDemoImpl = None

    def test_HDMI_1_4(self):
        multi_display.is_multi_displayed()
        self.apiDemoImpl.launch_app_am()
        self.apiDemoImpl.perform_presentation_through_mediarouter()
        multi_display.is_presentation_with_mediarouter()
        assert logcat.check_dumpsys_SurfaceFlinger_info_with_multiple_keys(\
                                keyword="SurfaceView", assertword=['1920','3840']),\
                                "Fail to get SurfaceView info from dumpsys log."

    def test_OGL_Pre_rotation_Optimization(self):
        print "[RunTest]: %s" % self.__str__()
        adb32.change_automatic_rotation(0) # Turn off auto rotation
        self.develop_settings.set_disable_hw_overlays(switch='ON')
        rotate_range = [1, 2, 3, 0]
        for i in rotate_range:
            adb32.screen_rotation(i)
            time.sleep(.5)
        self.develop_settings.set_disable_hw_overlays(switch='OFF')
        g_common_obj.assert_exp_happens()

    def test_DisplayInterface_HDMI_FHD(self):
        multi_display.is_multi_displayed()
        html5.check_chrome_installed()
        youtube_video_url, youtube_video_key = chrome_impl.get_youtube_url_key('american_bobtail',
                                                                               'key_american_bobtail')
        g_common_obj.assert_exp_happens()
        chrome_impl.launch()
        chrome_impl.chrome_setup()
        chrome_impl.open_website(youtube_video_url)
        chrome_impl.web_check(youtube_video_key, 15)

    def test_HDMI_Idle(self):
        multi_display.is_multi_displayed()
        multi_display.is_clone_mode()
        adb32.adb_shell_stop()
        adb32.adb_shell_start()

    def test_HDMI_CloneMode(self):
        multi_display.is_multi_displayed()
        multi_display.is_clone_mode()
