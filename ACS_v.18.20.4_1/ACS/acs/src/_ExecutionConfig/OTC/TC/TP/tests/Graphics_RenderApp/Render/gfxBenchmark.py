# -*- coding: utf-8 -*-
'''
Created on 09/20/2017
@author: Rui
'''
from testlib.util.common import g_common_obj
from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.extend_systemui_impl import SystemUiExtendImpl
from testlib.graphics.gfxbench31_impl import Gfxbench31Impl
import time


class RunBenchmark(UIATestBase):

    @classmethod
    def setUpClass(cls):
        print "[setUpClass]: %s" % cls.__name__
        super(RunBenchmark, cls).setUpClass()
        cls.d = g_common_obj.get_device()
        g_common_obj.root_on_device()

    def setUp(self):
        print "[Setup]:%s" % self.__class__.__name__
        super(RunBenchmark, self).setUp()
        self.systemui = SystemUiExtendImpl()
        self.gfxbench = Gfxbench31Impl()
        self.gfxbench.setup()

    def tearDown(self):
        print "[tearDown]:%s" % self.__class__.__name__
        self.gfxbench.clean()
        super(RunBenchmark, self).tearDown()

    @classmethod
    def tearDownClass(cls):
        print "[tearDownClass]: %s" % cls.__name__
        super(RunBenchmark, cls).tearDownClass()

    def test_EnableGFXBurst(self):
        cur_frequency = self.gfxbench.get_current_frequency()
        self.gfxbench.launch()
        self.gfxbench.start_test("T-Rex")
        time.sleep(5)
        cur_fequency2 = self.gfxbench.get_current_frequency()
        assert cur_fequency2 > cur_frequency, \
            "Current frequency is not higher than before."
        time.sleep(10)
        ct = 0
        while True:
            ct += 1; time.sleep(.5)
            if ct == 30:
                raise Exception("Current frequency is not lower than before.")
            else:
                cur_fre = self.gfxbench.get_current_frequency()
                print ">" * 10 + "Got current frequency: %s" % cur_fre
                if cur_fre < cur_fequency2: return True
