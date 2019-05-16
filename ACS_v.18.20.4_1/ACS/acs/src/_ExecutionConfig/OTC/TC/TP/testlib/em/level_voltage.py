# -*- coding: utf-8 -*-
#import os
import time

#from constants_def import SDP, CDP, DCP
from energy import Energy

class ALevelVoltage(object):

    def __init__(self, level, voltage):
        self.energy = Energy()
        self.level = level
        self.voltage = voltage

    def make_level_voltage(self):
        assert False, "needs be implemented in subclass"

    def verify_level_voltage_match(self):
        self.make_level_voltage()
        info = self.energy.get_battery_info()
        print info
        assert abs(self.level - info["level"]) <= 5
        assert abs(self.voltage - info["voltage"]) <= 50


class CLevelVoltagePowerSupply(ALevelVoltage):

    def __init__(self, level, voltage):
        ALevelVoltage.__init__(self, level, voltage)
        from power_supply import get_power_supply_obj
        from power import get_power_obj
        from usb_cut import USBCut
        self.usbcut = USBCut()
        self.power_supply = get_power_supply_obj()
        self.power = get_power_obj()

    def make_level_voltage(self):
        self.power.power_off()
        time.sleep(10)
        self.usbcut.usb_disconnect()
        time.sleep(5)
        self.power_supply.set_power_supply_output(0)
        time.sleep(5)
        self.power_supply.set_power_supply_voltage(self.voltage)
        time.sleep(5)
        self.power_supply.set_power_supply_output(1)
        time.sleep(5)
        self.usbcut.usb_connect()
        time.sleep(30)
        self.power.boot_up_device()
        self.power.adb_root()


#    def reconnect_usb(self):
#        self.charger.connect_usb_retry()


class CLevelVoltageCharger(ALevelVoltage):

    def __init__(self, level, voltage):
        ALevelVoltage.__init__(self, level, voltage)
        from relay08 import get_three_way_cutter
        self.charger = get_three_way_cutter()

    def make_level_voltage(self):
        level = self.energy.get_battery_info()["level"]
        level_diff = level - self.level
        if abs(level_diff) < 3:
            return
        elif level_diff < 0:
            self.charge()
        else:
            self.consume()

    def charge(self):
        cycles = 20
        for i in range(1, 1 + cycles):
            level = self.energy.get_battery_info()["level"]
            if level >= self.level:
                break
            wait_time = (target - level + 1) * 60
            print "[Info]: Charge cycle: %s/%s, wait %ss" % (i, cycles, wait_time)
            self.charger.enable_dcp_charging()
            time.sleep(wait_time)
            self.charger.enable_sdp_charging()
        else:
            assert False

    def consume(self):
        pass
        from apps import GLBenchmark
        app = GLBenchmark()
        app.install()
        cycles = 200
        for i in range(1, 1 + cycles):
            level = self.energy.get_battery_info()["level"]
            if level <= self.level:
                app.stop()
                break
            if not app.check_running():
                app.launch()
                app.select_all_tests()
                app.run_tests()
            time.sleep(60)
        else:
            assert False


def verify_level_voltage_match(level, voltage):
    level_votage = CLevelVoltageCharger(level, voltage)
    level_votage.verify_level_voltage_match()

