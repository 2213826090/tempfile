# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 05/04/2015
@author: Yingjun Jin
'''

from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.composeui_impl import ComposeUiImpl
from testlib.util.common import g_common_obj


class ComposeUI(UIATestBase):

    def setUp(self):
        super(ComposeUI, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self._composeui = ComposeUiImpl()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(ComposeUI, self).tearDown()
        g_common_obj.set_vertical_screen()

    def test_rotatedutwithnotificationbar_20times(self):
        ''' refer TC test_RotateDUTWithNotificationBar_20times
        '''
        print "[RunTest]: %s" % self.__str__()
        self._composeui.launch_settings_am()
        self._composeui.rotate_to_left()
        self._composeui.popup_notification_bar()
        self._composeui.rotate_20times()
        self._composeui.stop_settings_am()
