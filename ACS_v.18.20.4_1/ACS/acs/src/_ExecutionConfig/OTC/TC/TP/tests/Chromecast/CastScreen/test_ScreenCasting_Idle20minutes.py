# -*- coding: utf-8 -*-
'''
Created on Feb 15, 2015
@author: Ding, JunnanX
'''

from testlib.util.common import g_common_obj
from testlib.graphics.extend_systemui_impl import SystemUiExtendImpl
from testlib.chromecast.chromecast_test_template import ChromeCastTestBase


class ScreenCastingIdleTest(ChromeCastTestBase):

    @classmethod
    def setUpClass(cls):
        print "[setUpClass]: %s" % cls.__name__
        super(ScreenCastingIdleTest, cls).setUpClass()

    def setUp(self):
        print "[Setup]:%s" % self.__class__.__name__
        super(ScreenCastingIdleTest, self).setUp()

        cmd = 'date +"%m-%d %H:%M:%S"'
        self.test_time_mark = g_common_obj.adb_cmd_capture_msg(repr(cmd))

        self.chrome_cast = self.IChromeCastImpl(self.cast_model)
        self.systemui = SystemUiExtendImpl()

        self.chrome_cast.setup()

        self.systemui.unlock_screen()
        self.d.screen.on()

    def tearDown(self):
        print "[tearDown]:%s" % self.__class__.__name__
        cmd = 'logcat -d -v time -t "%s.000" *:F|grep -i Fatal' % (self.test_time_mark)
        fatal_msg = g_common_obj.adb_cmd_common(cmd)
        assert not fatal_msg,\
            "occurred Fatal error during testing:%s" % (fatal_msg)
        super(ScreenCastingIdleTest, self).tearDown()

    @classmethod
    def tearDownClass(cls):
        print "[tearDownClass]: %s" % cls.__name__
        super(ScreenCastingIdleTest, cls).tearDownClass()

    def test_ScreenCasting_Idle20minutes(self):
        """
        test_ScreenCasting_Idle20minutes

        Steps:
        1. Turn on Chromecast on DUT.
                The Chromecast is shown on the scan list.
        2. Connect to the Chromecast.
                The DUT is connected to the Chromecast.
        3. Press Home button.
                The Home screen is shown.
        4. Idle 20 minutes.
                The connection is not lost.
        """
        print "[RunTest]: %s" % self.__str__()

        print """[Step] 1. Turn on Chromecast on DUT.
                    The Chromecast is shown on the scan list."""
        self.turn_on_wireless()

        self.chrome_cast.launch()
        cast_real_name = self.chrome_cast.choose_castscreen(self.cast_model)

        print """[Step] 2. Connect to the Chromecast.
                    The DUT is connected to the Chromecast."""
        self.chrome_cast.connect_cast_screen(cast_real_name)

        print """[Step] 3. Press Home button.
                    The Home screen is shown."""
        self.d.press.home()
        print self.systemui.get_current_focus()

        print """[Step] 4. Idle 20 minutes.
                    The connection is not lost."""
        self.chrome_cast.watch_connection_status(count=20, step=60)
        self.chrome_cast.verify_connection_from_quick_settings(self.cast_model)

        self.chrome_cast.resume()
        self.chrome_cast.disconnect_cast_screen()
