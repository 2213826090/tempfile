# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 03/10/2015
@author: Yingjun Jin
'''

from testlib.util.uiatestbase import UIATestBase
from testlib.chromecast.chromecastpower_impl import ChromeCastImpl
from testlib.graphics.common import DBSettingsSetGet
from testlib.chromecast.chromecast_semi_auto_impl import UsbCamera, upload_file_to_prdshtwsv2d01

class ChromeCast(UIATestBase):

    def setUp(self):
        super(ChromeCast, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self._chromecast = ChromeCastImpl()
        self._settings = DBSettingsSetGet()
        self._settings.set_screen_off_timeout(60000)
        self._settings.set_status_stay_awake(False)
        self._chromecast.connect_chromecast()
        self._output_file = self._test_name.split(".")[-1]
        self._usb_camera_obj = UsbCamera(self._output_file)

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(ChromeCast, self).tearDown()
        self._settings.set_screen_off_timeout(1800000)
        self._settings.set_status_stay_awake(True)
        self._chromecast.disconnect_chromecast()

    def test_chromecast_autosleep_1minute(self):
        ''' refer TC test_Chromecast_AutoSleep_1minute
        '''
        try:
            self._usb_camera_obj.start_record()
            self._chromecast.sleep_time(61)
            self._chromecast.check_screen_status()
            recorded_video = self._usb_camera_obj.stop_record()
            upload_file_to_prdshtwsv2d01(self, recorded_video)
        except:
            self._usb_camera_obj.stop_record()