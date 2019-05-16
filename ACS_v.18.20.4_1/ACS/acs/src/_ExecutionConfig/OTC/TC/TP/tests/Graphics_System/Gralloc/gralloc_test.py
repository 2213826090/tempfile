# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 03/30/2015
@author: Yingjun Jin
'''

from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.gralloc_impl import GrallocImpl
from testlib.graphics.common import osversion

class Gralloc(UIATestBase):

    def setUp(self):
        super(Gralloc, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self._gralloc = GrallocImpl()
        self._gralloc.init_environment()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(Gralloc, self).tearDown()

    def test_gralloc_alloc_free_HAL_PIXEL_FORMAT_RAW_OPAQUE(self):
        ''' refer TC test_gralloc_alloc_free_HAL_PIXEL_FORMAT_RAW_OPAQUE
        '''
        print "[RunTest]: %s" % self.__str__()
        self._gralloc.run_case("OPAQUE")

    def test_gralloc_async_lock_HAL_PIXEL_FORMAT_RGBA_8888(self):
        ''' refer TC test_gralloc_async_lock_HAL_PIXEL_FORMAT_RGBA_8888
        '''
        print "[RunTest]: %s" % self.__str__()
        self._gralloc.run_case_special("lock_RGBA_8888")

    def test_gralloc_alloc_free_HAL_PIXEL_FORMAT_YCbCr_420_888(self):
        ''' refer TC test_gralloc_alloc_free_HAL_PIXEL_FORMAT_YCbCr_420_888
        '''
        print "[RunTest]: %s" % self.__str__()
        self._gralloc.run_case("YCbCr_420_888")

    def test_gralloc_async_lock_HAL_PIXEL_FORMAT_YCbCr_420_888(self):
        ''' refer TC test_gralloc_async_lock_HAL_PIXEL_FORMAT_YCbCr_420_888
        '''
        print "[RunTest]: %s" % self.__str__()
        self._gralloc.run_case_special("lock_YCbCr_420_888")

    def test_gralloc_alloc_free_HAL_PIXEL_FORMAT_sRGB_A_8888(self):
        ''' refer TC test_gralloc_alloc_free_HAL_PIXEL_FORMAT_sRGB_A_8888
        '''
        print "[RunTest]: %s" % self.__str__()
        self._gralloc.run_case("sRGB_A_8888")

    def test_gralloc_alloc_free_HAL_PIXEL_FORMAT_RGBA_8888(self):
        ''' refer TC test_gralloc_alloc_free_HAL_PIXEL_FORMAT_RGBA_8888
        '''
        print "[RunTest]: %s" % self.__str__()
        self._gralloc.run_case("free_RGBA_8888")

    def test_gralloc_alloc_free_HAL_PIXEL_FORMAT_RAW16(self):
        ''' refer TC test_gralloc_alloc_free_HAL_PIXEL_FORMAT_RAW16
        '''
        print "[RunTest]: %s" % self.__str__()
        self._gralloc.run_case("RAW16")

    def test_gralloc_open_close_gralloc(self):
        ''' refer TC test_gralloc_open_close_gralloc
        '''
        print "[RunTest]: %s" % self.__str__()
        self._gralloc.run_case("close_gralloc")

    def test_gralloc_HAL_PIXEL_FORMAT_sRGB_X_8888(self):
        ''' refer TC test_gralloc_HAL_PIXEL_FORMAT_sRGB_X_8888
        '''
        print "[RunTest]: %s" % self.__str__()
        self._gralloc.run_case("sRGB_X_8888")

    def test_gralloc_alloc_free_HAL_PIXEL_FORMAT_YV12(self):
        ''' refer TC test_gralloc_alloc_free_HAL_PIXEL_FORMAT_YV12
        '''
        print "[RunTest]: %s" % self.__str__()
        self._gralloc.run_case("FORMAT_YV12")

    def test_gralloc_alloc_free_HAL_PIXEL_FORMAT_Y8(self):
        ''' refer TC test_gralloc_alloc_free_HAL_PIXEL_FORMAT_Y8
        '''
        print "[RunTest]: %s" % self.__str__()
        self._gralloc.run_case("FORMAT_Y8")

    def test_gralloc_alloc_free_HAL_PIXEL_FORMAT_Y16(self):
        ''' refer TC test_gralloc_alloc_free_HAL_PIXEL_FORMAT_Y16
        '''
        print "[RunTest]: %s" % self.__str__()
        self._gralloc.run_case("FORMAT_Y16")

    def test_gralloc_alloc_free_HAL_PIXEL_FORMAT_RGB_888(self):
        ''' refer TC test_gralloc_alloc_free_HAL_PIXEL_FORMAT_RGB_888
        '''
        print "[RunTest]: %s" % self.__str__()
        self._gralloc.run_case("FORMAT_RGB_888")

    def test_gralloc_alloc_free_HAL_PIXEL_FORMAT_RGB_565(self):
        ''' refer TC test_gralloc_alloc_free_HAL_PIXEL_FORMAT_RGB_565
        '''
        print "[RunTest]: %s" % self.__str__()
        self._gralloc.run_case("FORMAT_RGB_565")