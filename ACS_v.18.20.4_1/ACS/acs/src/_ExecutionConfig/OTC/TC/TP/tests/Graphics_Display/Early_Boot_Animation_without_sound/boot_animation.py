# -*- coding: utf-8 -*-
'''
@summary: Boot Animation Test
@since: 09/19/2017
@author: Rui
'''
import re
from testlib.util.uiatestbase import UIATestBase
from testlib.util.common import g_common_obj
from testlib.graphics.splash_impl import SplashImpl


class SplashScreen(UIATestBase):

    def setUp(self):
        super(SplashScreen, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.common = g_common_obj
        self.sImpl = SplashImpl()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(SplashScreen, self).tearDown()

    def test_Animation_not_block_during_boot(self):
        print "[RunTest]: %s" % self.__str__()
        self.sImpl.check_earlyEvs_init()
        self.sImpl.check_static_splashfile()
        self.sImpl.check_animation_splashfile()

    def test_Animation_not_delay_boot_times(self):
        print "[RunTest]: %s" % self.__str__()
        self.sImpl.check_earlyEvs_init()
        self.sImpl.check_static_splashfile()
        self.sImpl.check_animation_splashfile()
        t_result = self.common.adb_cmd_capture_msg("free -m")
        get_value_parttern = re.compile(r"Mem: *(\d*) *\d*")
        t_value = get_value_parttern.findall(t_result)[0]
        print t_value
        wait_time = 18 if int(t_value) < 3000 else 15
        assert self.sImpl.get_bootanimation_time() <= wait_time, "Boot animation time is more than %s seconds." % wait_time

    def test_Boot_To_AOS_without_sound(self):
        assert not self.sImpl.earlyaudio_check(), "Early audio is heard during boot animation."
