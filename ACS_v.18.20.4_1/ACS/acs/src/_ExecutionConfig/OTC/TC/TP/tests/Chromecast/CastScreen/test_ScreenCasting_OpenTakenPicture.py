# -*- coding: utf-8 -*-
'''
Created on Feb 15, 2015
@author: Ding, JunnanX
'''
import time

from testlib.util.common import g_common_obj
from testlib.graphics.extend_systemui_impl import SystemUiExtendImpl
from testlib.chromecast.chromecast_test_template import ChromeCastTestBase
from testlib.graphics.extend_photos_impl import PhotosExtendImpl
from testlib.graphics.extend_camera_impl import CameraExtendImpl

class ScreenCastingOpenTakenPictureTest(ChromeCastTestBase):

    @classmethod
    def setUpClass(cls):
        print "[setUpClass]: %s" % cls.__name__
        super(ScreenCastingOpenTakenPictureTest, cls).setUpClass()

    def setUp(self):
        print "[Setup]:%s" % self.__class__.__name__
        super(ScreenCastingOpenTakenPictureTest, self).setUp()

        cmd = 'date +"%m-%d %H:%M:%S"'
        self.test_time_mark = g_common_obj.adb_cmd_capture_msg(repr(cmd))

        self.chrome_cast = self.IChromeCastImpl(self.cast_model)
        self.camera = CameraExtendImpl()
        self.photos = PhotosExtendImpl()
        self.systemui = SystemUiExtendImpl()

        self.camera.clean()

        self.systemui.unlock_screen()
        self.d.screen.on()
        self.d.orientation = 'natural'

        self.chrome_cast.setup()
        self.photos.setup()
        self.camera.startApp()

    def tearDown(self):
        print "[tearDown]:%s" % self.__class__.__name__
        cmd = 'logcat -d -v time -t "%s.000" *:F|grep -i Fatal' % (self.test_time_mark)
        fatal_msg = g_common_obj.adb_cmd_common(cmd)
        assert not fatal_msg,\
                "occurred Fatal error during testing:%s" % (fatal_msg)
        self.camera.clean()
        super(ScreenCastingOpenTakenPictureTest, self).tearDown()

    @classmethod
    def tearDownClass(cls):
        print "[tearDownClass]: %s" % cls.__name__
        super(ScreenCastingOpenTakenPictureTest, cls).tearDownClass()

    def test_ScreenCasting_OpenTakenPicture(self):
        """
        test_ScreenCasting_OpenTakenPicture

        Steps:
        1. Turn on Wireless Display on DUT.
            The Chromecast adapter is shown on the scan list.
        2. Connect to the Chromecast adapter
            The DUT is connected to the Chromecast adapter.
        3. Take a picture using the Camera application
            The picture is taken
        4. Open the Photos application and view the picture
            The picture is opened with Gallery app and screencasting is successful.
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

        print """[Step] 3. Take a picture using the Camera application
                    The picture is taken"""
        self.d.press.home()
        self.camera.launchCamera()
        self.camera.picTake()

        print """[Step] 4. Open the Photos application and view the picture
                    The picture is opened with Gallery app and screencasting is successful."""
        self.photos.launch()
        index = self.photos.find_photo()
        self.photos.click_list_item(index)
        time.sleep(5)

        print """[Step] 5. Verify connection.
                    The connection is not lost."""
        self.chrome_cast.watch_connection_status(count=1, step=1)
        self.chrome_cast.verify_connection_from_quick_settings(self.cast_model)

        self.chrome_cast.resume()
        self.chrome_cast.disconnect_cast_screen()
