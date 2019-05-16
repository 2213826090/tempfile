#! /usr/bin/env python
# coding:utf-8

import time
from testlib.util.uiatestbase import UIATestBase
from testlib.em.constants_def import BXT_O
from testlib.em.apps import Chrome
from testlib.em.tools import *
from testlib.em.web_ui_dptf import WebUI_DPTF

class WebDPTF(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        print
        print "[Setup]: %s" % self._test_name
 #       self.thermal = Thermal()
        self.dptf = WebUI_DPTF().get_web_dptf()
        self.dptf.adb_root()
        assert self.dptf.adb_remount(), "adb remount failed"
        self.dptf.push_web_dptf_files()
        self.dptf.start_dptf_web_server()
        super(WebDPTF, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        #super(WebDPTF, self).tearDown()

    def test_passive_policy2_max_and_min_pl1_value(self):
        print "[RunTest]: %s" % self.__str__()
        self.dptf.open_web_dptf()
        max_power = self.dptf.get_passive_policy2_cpu_power("Max")
        assert max_power == 13000, "Max power is not 13W"
        #req_power = self.dptf.get_passive_policy2_cpu_power("Requested Value")
        #assert max_power == max_power, "Request power is not Max power"
        min_power = self.dptf.get_passive_policy2_cpu_power("Min")
        assert min_power == 3000, "Min power is not 3W"
        granted_power = self.dptf.get_passive_policy2_cpu_power("Granted Value")
        diff_rate = abs(granted_power - max_power) / 13000.0
        assert diff_rate < 0.1, "Granted power is wrong"

    def test_power_limit_max_step_size_200_cpu_temp_less_than_90c(self):
        print "[RunTest]: %s" % self.__str__()
        self.dptf.open_web_dptf()
        step = self.dptf.get_psvt_step_size(90)
        assert step == 200

    def test_power_limit_max_step_size_200_cpu_temp_90c_to_105c(self):
        print "[RunTest]: %s" % self.__str__()
        self.dptf.open_web_dptf()
        step = self.dptf.get_psvt_step_size(105)
        assert step == 200

    def test_power_limit_max_step_size_400_cpu_temp_105c_to_110c(self):
        print "[RunTest]: %s" % self.__str__()
        self.dptf.open_web_dptf()
        step = self.dptf.get_psvt_step_size(110)
        assert step == 400

    def test_adaptive_policy_ambient_temp_less_than_59_9c_use_psvt_norm_more_than_60c_use_psvt_high(self):
        print "[RunTest]: %s" % self.__str__()
        """
        Needs TAMB < 60
        """
        self.dptf.open_web_dptf()
        self.dptf.check_psvt_solution()
        step = self.dptf.get_psvt_step_size(90)
        assert step == 200
        step = self.dptf.get_psvt_step_size(105)
        assert step == 200
        step = self.dptf.get_psvt_step_size(110)
        assert step == 400

    def test_critical_policy_show_trip_point_status(self):
        print "[RunTest]: %s" % self.__str__()
        self.dptf.open_web_dptf()
        self.dptf.check_critical_policy()

    def test_active_policy_show_trip_point_status_art_status(self):
        print "[RunTest]: %s" % self.__str__()
        from testlib.em.thermal import Thermal
        self.dptf.open_web_dptf()
        tskn, tamb, actual_fan = Thermal().get_fan_status()
        tcpu = self.dptf.ivi_get_cpu_temp() / 1000
        print "TCPU: %s, TSKN: %s, TAMB: %s" % (tcpu, tskn, tamb)
        art_fan = self.dptf.get_art_status(tcpu, tskn, tamb)
        print "ART fan", art_fan
        print "Actual fan", actual_fan
        assert art_fan == actual_fan

