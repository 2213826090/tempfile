# -*- coding: utf-8 -*-
'''
Created on Feb 15, 2015
@author: Ding, JunnanX
'''
import time

from testlib.util.common import g_common_obj
from testlib.graphics.extend_systemui_impl import SystemUiExtendImpl
from testlib.camera.camera_impl import CameraImpl
from testlib.chromecast.chromecast_test_template import ChromeCastTestBase


class RotationCameraPerviewTest(ChromeCastTestBase):

    @classmethod
    def setUpClass(cls):
        print "[setUpClass]: %s" % cls.__name__
        super(RotationCameraPerviewTest, cls).setUpClass()

    def setUp(self):
        print "[Setup]:%s" % self.__class__.__name__
        super(RotationCameraPerviewTest, self).setUp()

        cmd = 'date +"%m-%d %H:%M:%S"'
        self.test_time_mark = g_common_obj.adb_cmd_capture_msg(repr(cmd))

        self.chrome_cast = self.IChromeCastImpl(self.cast_model)
        self.camera = CameraImpl()
        self.systemui = SystemUiExtendImpl()

        self.chrome_cast.setup()

        self.systemui.unlock_screen()
        self.d.screen.on()

        self.d.freeze_rotation()
        self.d.orientation = 'natural'

    def tearDown(self):
        print "[tearDown]:%s" % self.__class__.__name__
        cmd = 'logcat -d -v time -t "%s.000" *:F|grep -i Fatal' % (self.test_time_mark)
        fatal_msg = g_common_obj.adb_cmd_common(cmd)
        assert not fatal_msg,\
            "occurred Fatal error during testing:%s" % (fatal_msg)
        super(RotationCameraPerviewTest, self).tearDown()

    @classmethod
    def tearDownClass(cls):
        print "[tearDownClass]: %s" % cls.__name__
        super(RotationCameraPerviewTest, cls).tearDownClass()

    def test_Rotation_CameraPerview(self):
        """
        test_ScreenCasting_Idle20minutes

        Steps:
        1. Turn on Wireless Display on DUT.
            The Chromecast is shown on the scan list.
        2. Cast screen to Chromecast
            Screen is cast to Chromecast .
        3. Launch Camera app.
            Camera preview displayed in both DUT and remote monitor.
        4. Rotate DUT in clockwise and then rotate it back.
            Both DUT and remote monitor display normally.
        5. Rotate DUT in anticlockwise and then rotate it back.
            Both DUT and remote monitor display normally.
        """
        print "[RunTest]: %s" % self.__str__()

        print """[Step] 1. Turn on Wireless Display on DUT.
                    The Chromecast is shown on the scan list."""
        self.turn_on_wireless()

        self.chrome_cast.launch()
        cast_real_name = self.chrome_cast.choose_castscreen(self.cast_model)

        print """[Step] 2. Cast screen to Chromecast
                    Screen is cast to Chromecast."""
        self.chrome_cast.connect_cast_screen(cast_real_name)

        print """[Step] 3. Launch Camera app.
                    Camera preview displayed in both DUT and remote monitor."""
        self.d.press.home()
        self.camera.launchCamera()

        print """[Step] 4. Rotate DUT in clockwise and then rotate it back.
                    Both DUT and remote monitor display normally."""
        self.d.orientation = "right"
        print self.d.info
        time.sleep(5)
        self.d.orientation = 'natural'
        print self.d.info
        time.sleep(5)

        print """[Step] 5. Rotate DUT in anticlockwise and then rotate it back.
                    Both DUT and remote monitor display normally."""
        self.d.orientation = "left"
        print self.d.info
        time.sleep(5)
        self.d.orientation = 'natural'
        print self.d.info
        time.sleep(5)

        self.chrome_cast.resume()
        self.chrome_cast.disconnect_cast_screen()
