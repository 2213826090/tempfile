# -*- coding: utf-8 -*-
import os
import time
from tools import UIBase


class APower(UIBase):

    def __init__(self, serial = None, hard = True):
        UIBase.__init__(self, serial)
        if hard:
            from relay08 import get_relay_obj
            self.relay08 = get_relay_obj()
        else:
            self.relay08 = None

    def check_boot_completed(self):
        msg = self.testDevice.adb_cmd_capture_msg("getprop sys.boot_completed")
        if '1' == msg:
            return True
        return False

    def wait_boot_completed(self):
        for _ in range(20):
            time.sleep(5)
            if self.check_boot_completed():
                print "[info]--- Boot up completed"
                break
        else:
            return False
        return True

    def press_power_key(self, duration):
        self.relay08.press_power_key(duration)

    def power_off(self):
        assert False, "Should be implemented in subclass"

    def power_off_wait(self):
        self.power_off()
        for _ in range(50):
            time.sleep(2)
            if not self.check_boot_completed():
                print "[info]--- Power off completed"
                return True
        return False

    def force_power_off(self):
        self.relay08.press_power_key(10)

    def power_on(self):
        assert False, "Should be implemented in subclass"

    def boot_up(self, retry = 1):
        while retry:
            retry -= 1
            self.power_on()
            if self.wait_boot_completed():
                return
            elif retry:
                self.force_power_off()
                time.sleep(2)
        assert False, "Boot up failed"

    def reboot(self, retry = 1):
        assert self.power_off_wait()
        time.sleep(60)
        self.boot_up(retry)

    def adb_reboot(self):
        self.testDevice.adb_cmd_common("reboot")
        time.sleep(10)
        self.wait_boot_completed()

    def power_off_on_os_by_ignition(self):
        self.force_power_off()
        #self.power_off_wait()
        time.sleep(600)
        self.boot_up()

class CPowerNormal(APower):

    def __init__(self, serial = None, hard = True):
        APower.__init__(self, serial, hard)

    def trigger_power_off_dialog(self, retry = 6):
        print "[info]--- trigger power off dialog"
        for i in range(retry):
            self.relay08.press_power_key(4)
            if self.d(text="Power off").exists:
                bounds = self.d(text="Power off").bounds
                return bounds
            time.sleep(2)
        else:
            assert False, "Power off dialog not show"

    def power_off(self):
        self.trigger_power_off_dialog()
        print "[info]--- Click 'Power off' on device screen"
        self.d(text="Power off").click.wait()

    def power_off_timer(self, wait_time = 10):
        from nohup_process import NohupProcess
        bounds = self.trigger_power_off_dialog()
        x = (bounds["left"] + bounds["right"]) / 2
        y = (bounds["top"] + bounds["bottom"]) / 2
        script_name = "click.sh"
        script_path = os.path.join(os.path.dirname(__file__), "device_scripts", script_name)
        exec_dir = "/sdcard/"
        string_args = "%s %s %s" % (wait_time, x, y)
        np = NohupProcess(self.testDevice, script_path, exec_dir, string_args)
        np.start()

    def power_on(self):
        self.relay08.press_power_key(5)


class CPowerIVI(APower):

    def __init__(self, serial = None, hard = True):
        APower.__init__(self, serial, hard)

    def power_off(self):
        self.relay08.press_power_key(10)

    def power_on(self):
        self.relay08.press_power_key(0.5)

def get_power_obj(serial = None, hard = True):
    from constants_def import BXT_M, BXT_O
    product = UIBase().get_product()
    if product in [BXT_M, BXT_O]:
        return CPowerIVI(serial, hard)
    else:
        return CPowerNormal(serial, hard)

