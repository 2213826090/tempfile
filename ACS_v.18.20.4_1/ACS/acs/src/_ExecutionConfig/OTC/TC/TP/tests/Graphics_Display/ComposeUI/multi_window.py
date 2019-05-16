# -*- coding: utf-8 -*-

from testlib.graphics.extend_systemui_impl import SplitWindow
from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.extend_chrome_impl import chrome_impl
from testlib.graphics.game_impl import game_impl
from testlib.graphics.common import adb32, wifi_ctrl
from testlib.common.common import g_common_obj
import time
from random import choice
from testlib.util.log import Logger

LOG = Logger.getlogger(__name__)


class ComposeUI(UIATestBase):

    def setUp(self):
        super(ComposeUI, self).setUp()
        self.split_window_impl = SplitWindow()
        self.d = g_common_obj.get_device()

    def tearDown(self):
        g_common_obj.back_home()
        if self.split_window_impl.split_button.exists: self.split_window_impl._exit_split()
        wifi_ctrl.turn_on()
        adb32.screen_rotation(0)
        try:
            game_impl.uninstall_hillclimb()
        except:
            pass

    def test_MultiWindow_SplitScreen(self):
        self.split_window_impl.split_app_window(app_name="Settings", orientation="right")
        try:
            # Close recent dashboard
            self.d(resourceId='com.android.systemui:id/title', description='Chrome').click.wait()
        except:
            pass
        chrome_impl.open_website('m.youtube.com')
        chrome_impl.web_check('Home', 8)

    def test_MultiWindow_SplitScreen_PlayGame_ImgeViewing(self):
        wifi_ctrl.turn_off()
        game_impl.install_apk("HillClimb")
        game_impl.launch_hillclimb_am()
        game_impl.play_hillclimb()
        self.split_window_impl.split_app_window("Hill Climb Racing", "left")
        try:
            # Close recent dashboard
            self.d(resourceId='com.android.systemui:id/title', description='Chrome').click.wait()
        except:
            pass
        rotate_value = [1, 2, 3, 0] * 8 # Rotate 32 times.
        LOG.info("Start rotation")
        for r in rotate_value:
            adb32.screen_rotation(r)
            time.sleep(1)
        for i in range(2):
            self.d.press.power()
            time.sleep(6)
        assert self.split_window_impl.split_button.exists, "Split mode was disabled."
        self.split_window_impl.change_split_size(choice(['left', 'right']))
        g_common_obj.assert_exp_happens()

    def test_MultiWindow_CombineScreen(self):
        self.split_window_impl.split_app_window(app_name="Settings", orientation="right")
        self.split_window_impl.change_split_size('edge')
