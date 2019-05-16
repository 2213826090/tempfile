# -*- coding: utf-8 -*-
'''
Created on Feb 15, 2015
@author: Ding, JunnanX
'''

from testlib.chromecast.chromecast_test_template import ChromeCastTestBase
from testlib.graphics.common import Logcat
from testlib.graphics.extend_camera_impl import CameraExtendImpl
from testlib.graphics.extend_systemui_impl import SystemUiExtendImpl
from testlib.util.common import g_common_obj


class CameraFrontbackCameraPictureCapturingTest(ChromeCastTestBase):

    @classmethod
    def setUpClass(cls):
        print "[setUpClass]: %s" % cls.__name__
        super(CameraFrontbackCameraPictureCapturingTest, cls).setUpClass()

    def setUp(self):
        print "[Setup]:%s" % self.__class__.__name__
        super(CameraFrontbackCameraPictureCapturingTest, self).setUp()

        cmd = 'date +"%m-%d %H:%M:%S"'
        self.test_time_mark = g_common_obj.adb_cmd_capture_msg(repr(cmd))

        self.logcat = Logcat(g_common_obj)
        self.chrome_cast = self.IChromeCastImpl(self.cast_model)
        self.camera = CameraExtendImpl()
        self.systemui = SystemUiExtendImpl()

        self.camera.clean()

        self.systemui.unlock_screen()
        self.d.screen.on()
        self.d.orientation = 'natural'

        self.chrome_cast.setup()
        self.camera.startApp()

    def tearDown(self):
        print "[tearDown]:%s" % self.__class__.__name__
        cmd = 'logcat -d -v time -t "%s.000" *:F|grep -i Fatal' % (self.test_time_mark)
        fatal_msg = g_common_obj.adb_cmd_common(cmd)
        assert not fatal_msg,\
            "occurred Fatal error during testing:%s" % (fatal_msg)
        self.camera.clean()
        super(CameraFrontbackCameraPictureCapturingTest, self).tearDown()

    @classmethod
    def tearDownClass(cls):
        print "[tearDownClass]: %s" % cls.__name__
        super(CameraFrontbackCameraPictureCapturingTest, cls).tearDownClass()

    def test_Camera_FrontbackCameraPictureCapturing(self):
        """
        test_Camera_720pHDVideoRecording

        Steps:
        1. Turn on Wireless Display on DUT.
            The Chromecast adapter is shown on the scan list.
        2. Connect to the Chromecast adapter
            The DUT is connected to the Chromecast adapter.
        3. Start the Camera application.
            The Camera application is opened.
        4. Take some pictures with front/back camera.
            File pictures are taken and screencasting has no degradation.
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

        print """[Step] 3. Start the Camera application.
            The Camera application is opened."""
        self.d.press.home()
        self.camera.launchCamera()

        print """[Step] 4. Take some pictures with front/back camera.
            File pictures are taken and screencasting has no degradation."""
        self.camera.cameraSwitchBackFront(to='front')
        self.camera.picTake()
        pics = self.camera.get_stored_pics()
        assert len(pics) == 1,\
            "[FAILURE] Failed to taken picture"

        self.camera.cameraSwitchBackFront(to='back')
        self.camera.picTake()
        pics = self.camera.get_stored_pics()
        assert len(pics) == 2,\
            "[FAILURE] Failed to taken picture"
        print "[Debug] %s" % (pics)

        print """[Step] 5. Verify connection.
                    The connection is not lost."""
        self.chrome_cast.watch_connection_status(count=1, step=1)
        self.chrome_cast.verify_connection_from_quick_settings(cast_real_name)

        self.chrome_cast.resume()
        self.chrome_cast.disconnect_cast_screen()
