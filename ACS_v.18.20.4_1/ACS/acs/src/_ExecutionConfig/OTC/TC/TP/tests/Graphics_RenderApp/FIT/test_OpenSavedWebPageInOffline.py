# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 04/24/2015
@author: Yingjun Jin
'''

from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.fit_impl import FitImpl
from testlib.graphics.html5_impl import html5


class Fit(UIATestBase):

    def setUp(self):
        super(Fit, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self._fit = FitImpl()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(Fit, self).tearDown()
        self._fit.open_wifi()

    def test_opensavedwebpageinoffline(self):
        ''' refer TC test_OpenSavedWebPageInOffline
        '''
        print "[RunTest]: %s" % self.__str__()
        html5.check_chrome_installed()
        html5.launch_chrome_am()
        self._fit.open_google_web_and_disconnect_wifi()
        html5.stop_chrome_am()
        html5.launch_chrome_am()
        self._fit.check_web_after_disconnect_wifi()
        html5.stop_chrome_am()
