# -*- coding: utf-8 -*-
'''
@summary: run Oglconform
@since: 06/13/2017
@author: rui
'''
from testlib.graphics.framebuffer_compression_impl import FBCImpl
from testlib.util.common import g_common_obj
from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.common import adb32, wifi_ctrl
from testlib.androidframework.common import EnvironmentUtils


class RunFBC(UIATestBase):


    @classmethod
    def setUpClass(self):
        super(RunFBC, self).setUpClass()
        if g_common_obj.adb_cmd_capture_msg("ps | grep adbd")[0:4] != "root":
            g_common_obj.root_on_device()
        self.android_version = EnvironmentUtils().get_android_version()

    @classmethod
    def tearDownClass(self):
        super(RunFBC, self).tearDownClass()

    def setUp(self):
        super(RunFBC, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.fbcImpl = FBCImpl()
        wifi_ctrl.turn_off()
        adb32.adb_disable_verity()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(RunFBC, self).tearDown()
        self.fbcImpl = None
        if self.android_version == "M":
            adb32._adb_reboot()
        wifi_ctrl.turn_on()

    def test_Framebuffer_Compression_blt(self):
        print "[RunTest]: %s" % self.__str__()
        self.fbcImpl.run_test(subtest='blt')

    def test_Framebuffer_Compression_context(self):
        print "[RunTest]: %s" % self.__str__()
        self.fbcImpl.run_test(subtest='context')

    def test_Framebuffer_Compression_mmap_cpu(self):
        print "[RunTest]: %s" % self.__str__()
        self.fbcImpl.run_test(subtest='mmap_cpu')

    def test_Framebuffer_Compression_mmap_gtt(self):
        print "[RunTest]: %s" % self.__str__()
        self.fbcImpl.run_test(subtest='mmap_gtt')

    def test_Framebuffer_Compression_page_flip(self):
        print "[RunTest]: %s" % self.__str__()
        self.fbcImpl.run_test(subtest='page_flip')

    def test_Framebuffer_Compression_page_flip_and_blt(self):
        print "[RunTest]: %s" % self.__str__()
        self.fbcImpl.run_test(subtest='page_flip_and_blt')

    def test_Framebuffer_Compression_page_flip_and_context(self):
        print "[RunTest]: %s" % self.__str__()
        self.fbcImpl.run_test(subtest='page_flip_and_context')

    def test_Framebuffer_Compression_page_flip_and_mmap_cpu(self):
        print "[RunTest]: %s" % self.__str__()
        self.fbcImpl.run_test(subtest='page_flip_and_mmap_cpu')

    def test_Framebuffer_Compression_page_flip_and_mmap_gtt(self):
        print "[RunTest]: %s" % self.__str__()
        self.fbcImpl.run_test(subtest='page_flip_and_mmap_gtt')

    def test_Framebuffer_Compression_page_flip_and_render(self):
        print "[RunTest]: %s" % self.__str__()
        self.fbcImpl.run_test(subtest='page_flip_and_render')

    def test_Framebuffer_Compression_render(self):
        print "[RunTest]: %s" % self.__str__()
        self.fbcImpl.run_test(subtest='render')
