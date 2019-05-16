# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 06/04/2015
@author: Xiangyi Zhao
'''

from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.tools import ConfigHandle
from testlib.graphics.composeui_impl import ComposeUiImpl


class ComposeUI(UIATestBase):

    def setUp(self):
        super(ComposeUI, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self._composeui = ComposeUiImpl()
        config_handle = ConfigHandle()
        result = config_handle.check_apps("org.geometerplus.zlibrary.ui.android")
        if result == 0:
            self._composeui.install_apk('FBReader')
        result = config_handle.check_apps("org.geometerplus.fbreader.plugin.pdf")
        if result == 0:
            self._composeui.install_apk('PDFplugin')
        self._composeui.init_local_pdf()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(ComposeUI, self).tearDown()
        self._composeui.uninstall_fbreader_and_pdfplugin()
        self._composeui.delete_local_pdf()

    def test_immersemode_fbreader(self):
        ''' refer TC test_ImmerseMode_FBReader
        '''
        print "[RunTest]: %s" % self.__str__()
        self._composeui.launch_fbreader_and_pdfplugin()
        self._composeui.check_notification_in_pdfreader()
        self._composeui.stop_fbreader_and_pdfplugin()
