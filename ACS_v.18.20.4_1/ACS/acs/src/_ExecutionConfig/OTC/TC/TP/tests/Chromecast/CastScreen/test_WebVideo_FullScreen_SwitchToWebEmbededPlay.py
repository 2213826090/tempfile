# -*- coding: utf-8 -*-
'''
Created on Feb 15, 2015
@author: Ding, JunnanX
'''

from testlib.util.common import g_common_obj
from testlib.util.repo import Artifactory
from testlib.graphics.extend_systemui_impl import SystemUiExtendImpl
from testlib.graphics.extend_chrome_impl import ChromeExtendImpl
from testlib.chromecast.chromecast_test_template import ChromeCastTestBase


class WebVideoFullScreenSwitchToWebEmbededPlayTest(ChromeCastTestBase):

    @classmethod
    def setUpClass(cls):
        print "[setUpClass]: %s" % cls.__name__
        super(WebVideoFullScreenSwitchToWebEmbededPlayTest, cls).setUpClass()

        config_file = 'tests.castscreen.cases.conf'
        cls.test_conf = cls.config.read(config_file, cls.__name__)
        arti = Artifactory(cls.test_conf.get('artifactory_location'))

        cls.local_path = arti.get(cls.test_conf.get("video_file"))
        cls.remote_path = cls.test_conf.get("dut_video_file")

    def clean(self):
        dut_shell =\
            "rm %s; sync; "\
            "am broadcast -a android.intent.action.MEDIA_MOUNTED -d file://%s;"\
            % (self.remote_path, self.remote_path)
        g_common_obj.adb_cmd_capture_msg(repr(dut_shell))

    def setUp(self):
        print "[Setup]:%s" % self.__class__.__name__
        super(WebVideoFullScreenSwitchToWebEmbededPlayTest, self).setUp()

        self.clean()
        ret = g_common_obj.push_file(self.local_path, self.remote_path)
        assert ret, 'Failed push %s' % (self.remote_path)

        cmd = 'date +"%m-%d %H:%M:%S"'
        self.test_time_mark = g_common_obj.adb_cmd_capture_msg(repr(cmd))

        self.chrome_cast = self.IChromeCastImpl(self.cast_model)
        self.chrome_browser = ChromeExtendImpl()
        self.systemui = SystemUiExtendImpl()

        self.systemui.unlock_screen()
        self.d.screen.on()
        self.d.orientation = 'natural'

        self.chrome_browser.browser_setup()
        self.chrome_cast.setup()

    def tearDown(self):
        print "[tearDown]:%s" % self.__class__.__name__
        cmd = 'logcat -d -v time -t "%s.000" *:F|grep -i Fatal' % (
            self.test_time_mark)
        fatal_msg = g_common_obj.adb_cmd_common(cmd)
        assert not fatal_msg, \
            "occurred Fatal error during testing:%s" % (fatal_msg)
        self.clean()
        super(WebVideoFullScreenSwitchToWebEmbededPlayTest, self).tearDown()

    @classmethod
    def tearDownClass(cls):
        print "[tearDownClass]: %s" % cls.__name__
        super(WebVideoFullScreenSwitchToWebEmbededPlayTest, cls).tearDownClass()

    def test_WebVideo_FullScreen_SwitchToWebEmbededPlay(self):
        """
        test_WebVideo_FullScreen_SwitchToWebEmbededPlay

        Steps:
        1. Turn on Wireless Display on DUT.
            The Chromecast adapter is shown on the scan list.
        2. Connect to the Chromecast adapter
            The DUT is connected to the Chromecast adapter.
        3. Choose 1 web video playing in full screen mode
            video could be played correct
        4. switch from full screen to web embeded playing
            Platform shall enter extended mode if video is played in full screen.
        """
        print "[RunTest]: %s" % self.__str__()

        print """[Step] 1. Turn on Wireless Display on DUT.
            The Chromecast adapter is shown on the scan list."""
        self.turn_on_wireless()
        self.chrome_cast.launch()
        cast_real_name = self.chrome_cast.choose_castscreen(self.cast_model)

        print """[Step] 2. Connect to the Chromecast adapter
            The DUT is connected to the Chromecast adapter."""
        self.chrome_cast.connect_cast_screen(cast_real_name)

        print """[Step] 3. Choose 1 web video playing in full screen mode
            video could be played correct."""
        self.chrome_browser.launch()
        self.chrome_browser.open_website('file://%s' % (self.remote_path))
        self.chrome_browser.play_video(secs=20)

        print """[Step] 4. switch from full screen to web embeded playing
            Platform shall enter extended mode if video is played in full screen."""
        self.systemui.switch_recent_app('Chrome')
        self.chrome_browser.enter_video_fullscreen()
        self.chrome_browser.play_video(secs=20)

        print """[Step] 5. Verify connection.
                    The connection is not lost."""
        self.chrome_cast.watch_connection_status(count=1, step=1)
        self.chrome_cast.verify_connection_from_quick_settings(cast_real_name)

        self.chrome_cast.resume()
        self.chrome_cast.disconnect_cast_screen()
