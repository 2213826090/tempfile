# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 04/16/2015
@author: Yingjun Jin
'''

from testlib.graphics.test_template.render_app_testbase import RenderAppTestBase
from testlib.graphics.fit_impl import FitImpl


class Fit(RenderAppTestBase):

    def setUp(self):
        super(Fit, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self._fit = FitImpl()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(Fit, self).tearDown()
        self._fit.unset_airplane()

    def test_airplanemode_enableconnectwifi_suspendresume(self):
        ''' refer TC test_AirplaneMode_EnableConnectWIFI_SuspendResume
        '''
        print "[RunTest]: %s" % self.__str__()
        self._fit.set_airplane()
        self._fit.enable_wifi()
        self._fit.suspend_and_resume()
