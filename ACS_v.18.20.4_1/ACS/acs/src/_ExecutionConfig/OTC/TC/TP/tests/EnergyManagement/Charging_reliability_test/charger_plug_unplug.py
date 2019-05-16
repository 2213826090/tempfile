import time
from testlib.util.uiatestbase import UIATestBase
from testlib.em.constants_def import SDP, CDP, DCP
from testlib.em.apps import EMToolsCharger, EMToolsScreen
from testlib.em.settings import DisplaySetting
from testlib.em.charging import get_charging_result
from testlib.em.energy import Energy

class ChargerPlugUnplug(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.emtools = EMToolsCharger()
        self.energy = Energy()
        self.emtools.install()
        self.emtools.grant_permissions()
        self.emtools.adb_root()
        self.emtools.set_screen_status("on")
        self.emtools.unlock_screen()
        DisplaySetting().set_sleep_mode("30 minutes")
        super(ChargerPlugUnplug, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(ChargerPlugUnplug, self).tearDown()

    def check_reliability_charging(self, each_cycle, total_times, low_battery = False):
        if low_battery:
            info = self.energy.get_battery_info()
            assert info["level"] <= 15
        cycles = total_times / len(each_cycle)
        pass_num = int(cycles * 0.9)
        result = get_charging_result(self.emtools, each_cycle, cycles)
        charger_types = [unit[0] for unit in each_cycle]
        count = {}
        for item in result:
            count[item] = count.get(item, 0) + 1
        print "pass_num:", pass_num
        success = True
        for key in count.keys():
            print key, count[key]
            if key not in charger_types:
                continue
            if count[key] < pass_num:
                success = False
        assert success

    # Single charging
    def test_SDP_plug_unplug_50_times(self):
        each_cycle = ((SDP, 3, 3), )
        self.check_reliability_charging(each_cycle, 50)

    def test_SDP_plug_unplug_low_battery_50_times(self):
        each_cycle = ((SDP, 3, 3), )
        self.check_reliability_charging(each_cycle, 50, low_battery = True)

    def test_CDP_plug_unplug_50_times(self):
        each_cycle = ((CDP, 3, 3), )
        self.check_reliability_charging(each_cycle, 50)

    def test_CDP_plug_unplug_low_battery_50_times(self):
        each_cycle = ((CDP, 3, 3), )
        self.check_reliability_charging(each_cycle, 50, low_battery = True)

    def test_DCP_plug_unplug_50_times(self):
        each_cycle = ((DCP, 3, 3), )
        self.check_reliability_charging(each_cycle, 50)

    def test_DCP_plug_unplug_low_battery_50_times(self):
        each_cycle = ((DCP, 3, 3), )
        self.check_reliability_charging(each_cycle, 50, low_battery = True)

    def test_DCP_slow_plug_unplug_20_times(self):
        each_cycle = ((DCP, 5, 5), )
        self.check_reliability_charging(each_cycle, 20)

    def test_DCP_plug_unplug_low_battery_10_times(self):
        each_cycle = ((DCP, 3, 3), )
        self.check_reliability_charging(each_cycle, 10, low_battery = True)

    def test_stress_sdp_plug_unplug(self):
        each_cycle = ((SDP, 3, 3), )
        self.check_reliability_charging(each_cycle, 25)

    def test_stress_cdp_plug_unplug(self):
        each_cycle = ((CDP, 3, 3), )
        self.check_reliability_charging(each_cycle, 25)

    def test_stress_CDP_plug_unplug_10_battery(self):
        each_cycle = ((CDP, 3, 3), )
        self.check_reliability_charging(each_cycle, 25, low_battery = True)

    def test_stress_dcp_plug_unplug(self):
        each_cycle = ((DCP, 3, 3), )
        self.check_reliability_charging(each_cycle, 25)

    # Mixed charging
    def test_SDP_CDP_DCP_quickly_plug_unplug_50_times(self):
        each_cycle = ((DCP, 3, 3), (CDP, 3, 3), (SDP, 3, 3))
        self.check_reliability_charging(each_cycle, 50)

    def test_SDP_CDP_DCP_plug_unplug_50_times_low_battery(self):
        each_cycle = ((DCP, 3, 3), (CDP, 3, 3), (SDP, 3, 3))
        self.check_reliability_charging(each_cycle, 50, low_battery = True)

    def test_SDP_CDP_quickly_plug_unplug_50_times(self):
        each_cycle = ((CDP, 3, 3), (SDP, 3, 3))
        self.check_reliability_charging(each_cycle, 50)

    def test_SDP_CDP_slow_plug_unplug_20_times(self):
        each_cycle = ((CDP, 5, 5), (SDP, 5, 5))
        self.check_reliability_charging(each_cycle, 20)

    def test_SDP_DCP_quickly_plug_unplug_50_times(self):
        each_cycle = ((DCP, 3, 3), (SDP, 3, 3))
        self.check_reliability_charging(each_cycle, 50)

    def test_CDP_DCP_quickly_plug_unplug_50_times(self):
        each_cycle = ((DCP, 3, 3), (CDP, 3, 3))
        self.check_reliability_charging(each_cycle, 50)

    # charger plug unplug in sleep
    def check_plug_unplug_in_sleep(self, each_cycle, total_times):
        cycles = total_times / len(each_cycle)
        pass_num = int(total_times * 2)
        result = get_charging_result(EMToolsScreen(), each_cycle, cycles)
        screen_on_count = result.count("ON")
        print "pass_num:", pass_num
        print "screen on count:", screen_on_count
        assert screen_on_count >= pass_num

    def test_SDP_plug_unplug_in_sleep(self):
        each_cycle = ((SDP, 20, 20), )
        self.check_plug_unplug_in_sleep(each_cycle, 50)

    def test_CDP_plug_unplug_in_sleep(self):
        each_cycle = ((CDP, 20, 20), )
        self.check_plug_unplug_in_sleep(each_cycle, 50)

    def test_DCP_plug_unplug_in_sleep(self):
        each_cycle = ((DCP, 20, 20), )
        self.check_plug_unplug_in_sleep(each_cycle, 50)

