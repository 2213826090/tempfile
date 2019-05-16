# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 05/05/2015
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
        self._composeui.roate_to_n()

    def test_rotate_anticlockwise_5times(self):
        ''' refer TC test_Rotate_Anticlockwise_5times
        '''
        print "[RunTest]: %s" % self.__str__()
        self._composeui.launch_settings_am()
        self._composeui.rotate_anticlockwise_5times()
        self._composeui.stop_settings_am()
