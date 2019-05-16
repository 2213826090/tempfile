# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 03/03/2015
@author: Yingjun Jin
'''

from testlib.util.uiatestbase import UIATestBase
from testlib.chromecast.chromecastcastscreen_impl import ChromeCastImpl

class ChromeCast(UIATestBase):

    def setUp(self):
        super(ChromeCast, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self._chromecast = ChromeCastImpl()
        self._chromecast.init_wps()
        self._chromecast.connect_chromecast()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(ChromeCast, self).tearDown()
        self._chromecast.disconnect_chromecast()
        self._chromecast.uninstall_wps()

    def test_screencasting_pdf_file_presentation(self):
        ''' refer TC test_ScreenCasting_PDFFilePresentation
        '''
        self._chromecast.launch_wps_am()
        self._chromecast.view_pdf_with_wps()
        self._chromecast.stop_wps_am()
