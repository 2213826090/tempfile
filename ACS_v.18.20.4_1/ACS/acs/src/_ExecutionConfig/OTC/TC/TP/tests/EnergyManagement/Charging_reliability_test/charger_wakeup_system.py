import time
from testlib.util.uiatestbase import UIATestBase
from testlib.em.em_impl import EMImpl

SDP = "USB"
CDP = "USB_CDP"
DCP = "USB_DCP"

class ChargerWakeupSystem(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.emImpl = EMImpl()
        self.suspend_time = int(self.emImpl.get_config_value("s0ix", "suspend_time"))
        self.emImpl.adb_root()
        self.emImpl.set_screen_status("on")
        self.emImpl.unlock_screen()
        self.emImpl.setSleepMode("30 minutes")
        self.emImpl.install_em_tools()
        super(ChargerWakeupSystem, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        self.emImpl.three_way_cutter_reconnect_sdp(3, 2, 5)
        super(ChargerWakeupSystem, self).tearDown()

    def charger_wakeup_system_in_S0i3(self, charger_type):
        s3_pre = self.emImpl.get_s0i3_suspend_stat()
        self.emImpl.set_screen_status("on")
        self.emImpl.unlock_screen()
        self.emImpl.start_monitor_screen_on_off()
        self.emImpl.set_screen_status("off")
        time.sleep(2)
        if charger_type == SDP:
            self.emImpl.enable_sdp_charging(self.suspend_time, 5)
        elif charger_type == CDP:
            self.emImpl.enable_cdp_charging(self.suspend_time, 5)
        else:
            self.emImpl.enable_dcp_charging(self.suspend_time, 5)
        # wait screen off
        time.sleep(15)
        self.emImpl.three_way_cutter_reconnect_sdp(3, 2, 5)
        self.emImpl.unlock_screen()
        history = self.emImpl.get_screen_on_off_history().split()
        result = True
        if history.count("ON") < 3:
            result = False
        if history.count("OFF") < 3:
            result = False
        s3_post = self.emImpl.get_s0i3_suspend_stat()
        if s3_pre == s3_post:
            result = False
        return result

    def test_DCP_wakeup_system_in_S0i3(self):
        print "[RunTest]: %s" % self.__str__()
        assert self.charger_wakeup_system_in_S0i3(DCP)

    def test_DCP_wakeup_system_in_s0i3_10_times(self):
        print "[RunTest]: %s" % self.__str__()
        fail_num = 0
        cycles = 10
        for i in range(1, 1 + cycles):
            print "[info]--- Cycle: %s/%s" % (i, cycles)
            if not self.charger_wakeup_system_in_S0i3(DCP):
                fail_num += 1
            assert fail_num <= 2, "Failed times: %s" % fail_num

    def test_SDP_CDP_wakeup_system_in_s0i3_10_times(self):
        print "[RunTest]: %s" % self.__str__()
        fail_num = 0
        cycles = 10
        chargers = [SDP, CDP]
        for i in range(1, 1 + cycles):
            print "[info]--- Cycle: %s/%s" % (i, cycles)
            if not self.charger_wakeup_system_in_S0i3(chargers[i % 2]):
                fail_num += 1
            assert fail_num <= 2, "Failed times: %s" % fail_num

