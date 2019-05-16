#! /usr/bin/env python
# coding:utf-8

import time
from testlib.util.uiatestbase import UIATestBase
from testlib.em.power import get_power_obj
from testlib.em.constants_def import SDP, CDP, DCP

class ACOS(object):

    def __init__(self):
        self.power = get_power_obj()
        self.charger_type = None

    def power_off_with_charger(self):
        assert False, "Needs be implemented in subclass"

    def check_enter_cos_with_charger(self):
        # check enter COS mode
        print "[Info]--- Check if enter COS mode"
        # if enter COS mode, switch to SDP, device will be found very soon
        self.usb_connect()
        for i in range(3):
            if self.power.get_state():
                print "[Info]--- Enter COS mode by", self.charger_type
                return True
            time.sleep(2)
        print "[Info]--- Not enter COS mode by", self.charger_type
        return False

    def check_exit_cos_remove_charger(self):
        # make the device exit COS mode
        self.remove_charger()
        time.sleep(20)
        # check the device exit COS mode
        self.usb_connect()
        time.sleep(3)
        if self.power.get_state():
            print "[Info]--- Not exit COS mode"
            return False
        else:
            print "[Info]--- Exit COS mode"
            return True

    def connect_charger(self):
        assert False, "Needs be implemented in subclass"

    def remove_charger(self):
        assert False, "Needs be implemented in subclass"

    def usb_connect(self):
        assert False, "Needs be implemented in subclass"

    def boot_up(self):
        # waite screen off to boot up
        time.sleep(30)
        self.power.boot_up()
        self.power.adb_root()

class CCOS(ACOS):

    def __init__(self):
        ACOS.__init__(self)
        from testlib.em.relay08 import get_relay_obj
        self.cutter = get_relay_obj()
        self.charger_type = SDP

    def power_off_with_charger(self):
        self.power.power_off()
        time.sleep(30)

    def connect_charger(self):
        self.cutter.usb_connect()

    def remove_charger(self):
        self.cutter.usb_disconnect()

    def usb_connect(self):
        self.cutter.usb_connect()


class CCOS_AC(ACOS):

    def __init__(self, charger_type):
        ACOS.__init__(self)
        from testlib.em.relay08 import get_three_way_cutter
        self.cutter = get_three_way_cutter()
        self.charger_type = charger_type

    def power_off_with_charger(self):
        self.power.power_off_timer(10)
        self.connect_charger()
        # waite to enter COS mode
        time.sleep(40)

    def connect_charger(self):
        if self.charger_type == CDP:
            self.cutter.enable_cdp_charging(1, 0)
        else:
            self.cutter.enable_dcp_charging(1, 0)

    def remove_charger(self):
        self.cutter.set_usb_line_status(0)

    def usb_connect(self):
        self.cutter.enable_sdp_charging(1, 1)

def get_cos_obj(charger_type):
    if charger_type == SDP:
        return CCOS()
    elif charger_type in [CDP, DCP]:
        return CCOS_AC(charger_type)
    else:
        assert False, "Not supported charger type: " + charger_type

