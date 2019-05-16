# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 04/03/2015
@author: Yingjun Jin
'''

from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.tools import ConfigHandle
from testlib.graphics.composeui_impl import ComposeUiImpl
from testlib.graphics.fit_impl import FitImpl

class ComposeUI(UIATestBase):

    def setUp(self):
        super(ComposeUI, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self._composeui = ComposeUiImpl()
        self._fit = FitImpl()
        config_handle = ConfigHandle()
        result = config_handle.check_apps("com.mxtech.videoplayer.ad")
        if result == 0:
            self._composeui.install_apk('Mxplayer')
        self._composeui.init_local_video()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(ComposeUI, self).tearDown()
        self._composeui.uninstall_mxtech()
        self._fit.open_wifi()
        self._composeui.delete_local_video()

    def test_immersemode_mxplayer(self):
        ''' refer TC test_ImmerseMode_MXPlayer
        '''
        print "[RunTest]: %s" % self.__str__()
        self._composeui.launch_mxtech_am()
        self._composeui.check_notification_during_playing()
        self._composeui.stop_mxtech_am()
