# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 05/06/2015
@author: Yingjun Jin
'''

from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.androidextensionpack_impl import AndroidExtensionPackImpl


class AndroidExtensionPack(UIATestBase):

    def setUp(self):
        super(AndroidExtensionPack, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self._aep = AndroidExtensionPackImpl()
        self._aep.init_environment()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(AndroidExtensionPack, self).tearDown()
        self._aep.rm_log

    def test_OES_texture_storage_multisample_2d_array_Present(self):
        ''' refer TC test_OES_texture_storage_multisample_2d_array_Present
        '''
        print "[RunTest]: %s" % self.__str__()
        self._aep.run_case("array_Present")

    def test_OES_texture_stencil8_Present(self):
        ''' refer TC test_OES_texture_stencil8_Present
        '''
        print "[RunTest]: %s" % self.__str__()
        self._aep.run_case("stencil8_Present")

    def test_OES_sample_variables_Present(self):
        ''' refer TC test_OES_sample_variables_Present
        '''
        print "[RunTest]: %s" % self.__str__()
        self._aep.run_case("variables_Present")

    def test_KHR_debug_Present(self):
        ''' refer TC test_KHR_debug_Present
        '''
        print "[RunTest]: %s" % self.__str__()
        self._aep.run_case("debug_Present")
