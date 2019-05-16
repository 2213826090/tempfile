# -*- coding: utf-8 -*-
import os
import time

from constants_def import SDP, CDP, DCP
from apps import EMToolsCharger

class CChargerConf(object):

    def __init__(self, each_cycle, cycle_num):
        '''
        each_cycle: like ((SDP, 10), (None, 10))
        cycle: int
        '''
        self.each_cycle = each_cycle
        self.cycle_num = cycle_num

class ACharger(object):

    def __init__(self, config):
        self.charger = None
        self.config = config

    def reconnect_usb(self):
        assert False, "needs be implemented in subclass"

    def start(self):
        for i in range(1, 1 + self.config.cycle_num):
            print "Cycle: %d/%s" % (i, self.config.cycle_num)
            for charge_unit in self.config.each_cycle:
                print "Charge type:", charge_unit[0]
                self.charge(charge_unit)

    def charge(self, charger_unit):
        '''
        charger_type: SDP, CPD, DCP
        '''
        assert False, "needs be implemented in subclass"

class CUSBCharger(ACharger):

    def __init__(self, config):
        ACharger.__init__(self, config)
        from usb_cut import USBCut
        self.charger = USBCut()
        self.config = config

    def charge(self, charger_unit):
        self.charger.usb_disconnect()
        time.sleep(charger_unit[1])
        self.charger.usb_connect()
        time.sleep(charger_unit[2])

    def reconnect_usb(self):
        self.charger.connect_usb_retry()

class C3WayCharger(ACharger):

    def __init__(self, config):
        ACharger.__init__(self, config)
        from relay08 import get_three_way_cutter
        from energy import Energy
        self.charger = get_three_way_cutter()
        self.config = config
        self.energy = Energy()

    def charge(self, charger_unit):
        if charger_unit[0] == None:
            self.charger.set_usb_line_status(0)
            time.sleep(charger_unit[1])
        elif charger_unit[0] == CDP:
            self.charger.enable_cdp_charging(charger_unit[1], charger_unit[2])
        elif charger_unit[0] == DCP:
            self.charger.enable_dcp_charging(charger_unit[1], charger_unit[2])
        else:
            self.charger.enable_sdp_charging(charger_unit[1], charger_unit[2])

    def reconnect_usb(self):
        for i in range(3):
            info = self.energy.get_battery_info()
            if info and info["USB"]:
                break
            self.charger.enable_sdp_charging(2, 5)
        else:
            assert False


class Charging(object):

    def __init__(self, emtools, charger):
        self.charger = charger
        self.emtools = emtools

    def start_monitor(self):
        self.emtools.start_monitor()
        self.charger.start()

    def stop_monitor(self):
        self.charger.reconnect_usb()
        self.emtools.stop_monitor()

    def get_history(self):
        return self.emtools.get_history()

def get_charging_result(emtools, each_cycle, cycle_num):
    conf = CChargerConf(each_cycle, cycle_num)
    for unit in each_cycle:
        if unit[0] in (DCP, CDP):
            charger = C3WayCharger(conf)
            break
    else:
        charger = CUSBCharger(conf)
    charging = Charging(emtools, charger)
    charging.start_monitor()
    charging.stop_monitor()
    hist = charging.get_history()
    return hist.splitlines()

