# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 04/07/2015
@author: Yingjun Jin
'''

from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.hal_impl import HalImpl
from testlib.util.common import g_common_obj


class Hal(UIATestBase):

    def setUp(self):
        super(Hal, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self._hal = HalImpl()
        g_common_obj.root_on_device()
        g_common_obj.remount_device()
        self._hal.init_environment()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(Hal, self).tearDown()

    def test_HAL_PIXEL_FORMAT_RAW_OPAQUE(self):
        ''' refer TC test_HAL_PIXEL_FORMAT_RAW_OPAQUE
        '''
        print "[RunTest]: %s" % self.__str__()
        self._hal.run_case("RAW_OPAQUE")

    def test_HAL_PIXEL_FORMAT_RAW16(self):
        ''' refer TC test_HAL_PIXEL_FORMAT_RAW16
        '''
        print "[RunTest]: %s" % self.__str__()
        self._hal.run_case("FORMAT_RAW16")
