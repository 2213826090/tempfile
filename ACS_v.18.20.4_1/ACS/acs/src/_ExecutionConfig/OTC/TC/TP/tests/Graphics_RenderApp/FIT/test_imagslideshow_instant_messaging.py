# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 02/05/2015
@author: Yingjun Jin
'''

from testlib.graphics.test_template.render_app_testbase import RenderAppTestBase
from testlib.graphics.fit_impl import FitImpl
from testlib.util.common import g_common_obj
from testlib.util.config import TestConfig
import os
import time

class ImageSlideshow(RenderAppTestBase):

    @classmethod
    def setUpClass(self):
        """
        install apk
        """
        super(ImageSlideshow, self).setUpClass()
        config = TestConfig()
        FitImpl.set_environment()
        self._fit = FitImpl()
        cfg_file = 'tests.tablet.fit.conf'
        cfg_alarm = config.read(cfg_file, 'alarm')
        self.delay = cfg_alarm.get('delay')


    def setUp(self):
        super(ImageSlideshow, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name


    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(ImageSlideshow, self).tearDown()

    def test_imagslideshow_instant_messaging(self):
        ''' refer TC test_Graphics_PSS
        '''
        self._fit.sync_time_with_host()
        self._fit.launch_alarm()
        self._fit.delete_all_alarms()
        self._fit.add_alarm_by_delay_time(float(self.delay))
        time.sleep(self._fit.sleep_time)
        print "wait to view picture before alarm ring"
        self._fit.slide_view()
        time.sleep(20)
        self._fit.dismiss_alarm()
        self._fit.stop_app_am()