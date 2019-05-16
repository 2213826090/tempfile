# -*- coding: utf-8 -*-
from testlib.graphics.game_impl import game_impl
from testlib.graphics.common import wifi_ctrl
from testlib.util.common import g_common_obj
from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.html5_impl import html5
from testlib.graphics.extend_chrome_impl import chrome_impl
import random


class Game(UIATestBase):

    def setUp(self):
        super(Game, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        wifi_ctrl.turn_off() # Block ads
        game_impl.install_apk("SpinTops")

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(Game, self).tearDown()
        game_impl.stop_spintops_am()
        wifi_ctrl.turn_on()

    def test_3DGame_SpinTops_MSAA(self):
        print "[RunTest]: %s" % self.__str__()
        game_impl.launch_spintops_am()
        game_impl.init_spintops(3)
        game_impl.play_spintops(random.randint(1, 7), 5)

    def test_3DGame_Basemark_MSAA(self):
        '''
        Basemark is not supported on non-Houdini device, replaced app with spin-tops
        '''
        print "[RunTest]: %s" % self.__str__()
        for i in range(1, 5):
            game_impl.launch_spintops_am()
            game_impl.init_spintops(3)
            game_impl.play_spintops(i, 5)
            game_impl.stop_spintops_am()


class Youtube(UIATestBase):

    def setUp(self):
        super(Youtube, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.d = g_common_obj.get_device()
        wifi_ctrl.turn_on()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(Youtube, self).tearDown()

    def test_YouTube_VideoPlayback(self):
        print "[RunTest]: %s" % self.__str__()
        html5.check_chrome_installed()
        youtube_video_url, youtube_video_key = chrome_impl.get_youtube_url_key('american_bobtail',
                                                                               'key_american_bobtail')
        g_common_obj.assert_exp_happens()
        chrome_impl.launch()
        chrome_impl.chrome_setup()
        chrome_impl.open_website(youtube_video_url)
        chrome_impl.web_check(youtube_video_key, 15)
        chrome_impl.enter_youtube_fullscreen(10)
