# -*- coding: utf-8 -*-
'''
Created on 04/07/2015
@author: Ding, JunnanX
'''

from testlib.util.common import g_common_obj
from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.skia_impl import SkiaImpl


class SkiaBenchTest(UIATestBase):

    @classmethod
    def setUpClass(cls):
        print "[setUpClass]: %s" % cls.__name__
        super(SkiaBenchTest, cls).setUpClass()
        cls.d = g_common_obj.get_device()
        g_common_obj.root_on_device()
        g_common_obj.remount_device()

    def setUp(self):
        print "[Setup]:%s" % self.__class__.__name__
        super(SkiaBenchTest, self).setUp()

        self.skiatest = SkiaImpl()
        self.skiatest.clean()
        self.skiatest.setup()

    def tearDown(self):
        print "[tearDown]:%s" % self.__class__.__name__
        self.skiatest.clean()
        super(SkiaBenchTest, self).tearDown()

    @classmethod
    def tearDownClass(cls):
        print "[tearDownClass]: %s" % cls.__name__
        super(SkiaBenchTest, cls).tearDownClass()

    def test_Skia_bench(self):
        """
        test_Skia_bench

        Steps:
        1. run skia_bench on DUT.
            no fail, no error, no warning is reported during running skia_test in logcat.
        """
        print "[RunTest]: %s" % self.__str__()

        print """[Step] 1. run skia_bench on DUT.
            no fail, no error, no warning is reported during running skia_test in logcat."""
        self.skiatest.run_skia_bench()
