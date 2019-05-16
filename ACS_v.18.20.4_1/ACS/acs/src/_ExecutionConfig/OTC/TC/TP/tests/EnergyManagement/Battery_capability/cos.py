#! /usr/bin/env python
# coding:utf-8

import time
from testlib.util.uiatestbase import UIATestBase
from testlib.em.power import get_power_obj
from testlib.em.cos_mode import get_cos_obj
from testlib.em.constants_def import SDP, CDP, DCP

class COS(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.power = get_power_obj()
        self.power.adb_root()
        super(COS, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name

    def check_cos_mode(self, charger_type, check_exit = True):
        cos = get_cos_obj(charger_type)
        cos.power_off_with_charger()
        enter_cos = cos.check_enter_cos_with_charger()
        exit_cos = cos.check_exit_cos_remove_charger()
        cos.boot_up()
        assert enter_cos, "not enter COS mode"
        if check_exit:
            assert exit_cos, "not exit COS mode"

    def test_COS_remove_CDP_charger(self):
        print "[RunTest]: %s" % self.__str__()
        self.check_cos_mode(CDP)

    def test_COS_remove_DCP_charger(self):
        print "[RunTest]: %s" % self.__str__()
        self.check_cos_mode(DCP)

    def test_COS_remove_HUB_charger(self):
        print "[RunTest]: %s" % self.__str__()
        self.check_cos_mode(SDP)

    def test_COS_remove_SDP_charger(self):
        print "[RunTest]: %s" % self.__str__()
        self.check_cos_mode(SDP)

    def test_CDP_charging_power_off_enter_COS(self):
        print "[RunTest]: %s" % self.__str__()
        self.check_cos_mode(CDP, check_exit = False)

    def test_DCP_charging_power_off_enter_COS(self):
        print "[RunTest]: %s" % self.__str__()
        self.check_cos_mode(DCP, check_exit = False)

    def test_SDP_charging_power_off_enter_COS(self):
        print "[RunTest]: %s" % self.__str__()
        self.check_cos_mode(SDP, check_exit = False)

