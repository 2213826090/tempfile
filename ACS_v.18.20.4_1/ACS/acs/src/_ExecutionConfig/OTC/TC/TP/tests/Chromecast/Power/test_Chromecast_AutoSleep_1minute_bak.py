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

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(ChromeCast, self).tearDown()
        self._settings.set_screen_off_timeout(1800000)
        self._settings.set_status_stay_awake(True)
        self._chromecast.disconnect_chromecast()

    def test_chromecast_autosleep_1minute(self):
        ''' refer TC test_Chromecast_AutoSleep_1minute
        '''
        self._chromecast.sleep_time(61)
        self._chromecast.check_screen_status()
