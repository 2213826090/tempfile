# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 04/01/2015
@author: Yingjun Jin
'''

from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.composeui_impl import ComposeUiImpl

class ComposeUI(UIATestBase):

    def setUp(self):
        super(ComposeUI, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self._composeui = ComposeUiImpl()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(ComposeUI, self).tearDown()

    def test_rotate_clockwise_5times(self):
        ''' refer TC test_Rotate_Clockwise_5times
        '''
        print "[RunTest]: %s" % self.__str__()
        self._composeui.launch_settings_am()
        self._composeui.Rotate_clockwise_5times()
        self._composeui.stop_settings_am()