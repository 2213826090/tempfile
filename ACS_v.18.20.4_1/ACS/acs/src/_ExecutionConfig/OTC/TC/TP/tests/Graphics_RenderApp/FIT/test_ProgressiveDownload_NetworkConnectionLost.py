# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 04/23/2015
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
        self._fit.open_wifi()

    def test_progressivedownload_networkconnectionlost(self):
        ''' refer TC test_ProgressiveDownload_NetworkConnectionLost
        '''
        print "[RunTest]: %s" % self.__str__()
        self._fit.launch_playstore_am()
        self._fit.close_wifi_during_download_airattack()
        self._fit.stop_playstore_am()
