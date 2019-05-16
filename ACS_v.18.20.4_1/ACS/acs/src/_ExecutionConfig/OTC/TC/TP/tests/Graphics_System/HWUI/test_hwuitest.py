# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 09/15/2015
@author: Xiangyi Zhao
'''

from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.hwui_impl import HwuiImpl
from testlib.util.common import g_common_obj


class Hwui(UIATestBase):

    def setUp(self):
        super(Hwui, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self._hwui = HwuiImpl()
        g_common_obj.root_on_device()
        g_common_obj.remount_device()
        self._hwui.init_environment()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(Hwui, self).tearDown()

    def test_HWUI_test(self):
        ''' refer TC test_HWUI_test
        '''
        print "[RunTest]: %s" % self.__str__()
        self._hwui.run_case()

