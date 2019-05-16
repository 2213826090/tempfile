# -*- coding: utf-8 -*-
import os
import time
from settings import SecuritySetting

class AEncrypt(SecuritySetting):

    def __init__(self, serial = None):
        SecuritySetting.__init__(self, serial)
        self.action = "android.settings.SECURITY_SETTINGS"
        self.lock = None
        self.encrypt = None

    def set_lock(self):
        self.launch()
        time.sleep(2)
        self.choose_lock()
        self.input_encrypt()
        self.click_next()
        self.input_encrypt()
        self.click_next()

    def choose_lock(self):
        self.d(text="Screen lock").click()
        self.d(text=self.lock).click()
        self.d(text="No thanks").click()
        if self.d(text="Continue").exists:
            self.d(text="Continue").click()

    def input_encrypt(self):
        assert False, "Needs be implemented in subclass"

    def click_next(self):
        self.d.press.enter()

    def remove_lock(self):
        self.launch()
        time.sleep(2)
        self.d(text="Screen lock").click()
        self.input_encrypt()
        self.click_next()
        self.d(text="Swipe").click()
        self.d(resourceId="android:id/button1").click()

    def unlock_screen(self, passwd):
        assert False, "Needs be implemented in subclass"


class CPassword(AEncrypt):

    def __init__(self, serial = None):
        AEncrypt.__init__(self, serial)
        self.lock = "Password"
        self.encrypt = "aaaa"

    def input_encrypt(self):
        self.d(resourceId="com.android.settings:id/password_entry").set_text(self.encrypt)

    def unlock_screen(self):
        if self.d(resourceId="com.android.systemui:id/lock_icon").exists:
            self.d.press.menu()
        time.sleep(2)
        for i in range(5):
            if self.d(resourceId="com.android.systemui:id/passwordEntry").exists:
                self.d(resourceId="com.android.systemui:id/passwordEntry").set_text(self.encrypt)
                break
            time.sleep(3)
        self.d.press.enter()

class CPIN(AEncrypt):

    def __init__(self, serial = None):
        AEncrypt.__init__(self, serial)
        self.lock = "PIN"
        self.encrypt = "1111"

    def input_encrypt(self):
        self.d(resourceId="com.android.settings:id/password_entry").set_text(self.encrypt)

    def unlock_screen(self):
        if self.d(resourceId="com.android.systemui:id/lock_icon").exists:
            self.d.press.menu()
        self.d(resourceId="com.android.systemui:id/pinEntry").set_text(self.encrypt)
        self.d.press.enter()

class CPattern(AEncrypt):

    def __init__(self, serial = None):
        AEncrypt.__init__(self, serial)
        self.lock = "Pattern"
        #self.encrypt = "L"
        self.event_info = self.get_event_info()
        self.remote_script = "/sdcard/draw_pattern_lock.sh"
        self.push_script()

    def get_event_info(self):
        import re
        cmd = "dumpsys display | grep mDefaultViewport"
        msg = self.testDevice.adb_cmd_capture_msg(cmd)
        p = re.compile("deviceWidth=(\d+), deviceHeight=(\d+)")
        s = p.search(msg)
        width = int(s.group(1))
        height = int(s.group(2))
        event_info = {}
        cmd = "getevent -i"
        msg = self.testDevice.adb_cmd_capture_msg(cmd)
        p1 = re.compile("0035.+max\s(\d+)")
        p2 = re.compile("0036.+max\s(\d+)")
        for line in msg.splitlines():
            if line.startswith("add device"):
                event_info["dev_num"] = line[-1]
            elif line.startswith(" "):
                s1 = p1.search(line)
                if s1:
                    max_53 = int(s1.group(1))
                    continue
                s2 = p2.search(line)
                if s2:
                    max_54 = int(s2.group(1))
                    break
            else:
                continue
        event_info["scale_x"] = 1.0 * max_53 / width
        event_info["scale_y"] = 1.0 * max_54 / height
        print event_info
        return event_info

    def push_script(self):
        import os
        script_name = "draw_pattern_lock.sh"
        script_path = os.path.join(os.path.dirname(__file__), "device_scripts", script_name)
        assert os.path.exists(script_path)
        cmd = "push %s %s" % (script_path, self.remote_script)
        self.testDevice.adb_cmd_common(cmd)

    def draw_pattern(self, bounds):
        margin = (bounds["right"] - bounds["left"]) / 6
        left = int((bounds["left"] + margin) * self.event_info["scale_x"])
        top = int((bounds["top"] + margin) * self.event_info["scale_y"])
        right = int((bounds["right"] - margin) * self.event_info["scale_x"])
        bottom = int((bounds["bottom"] - margin) * self.event_info["scale_y"])
        cmd = "sh %s %s %s %s %s %s" % (self.remote_script, self.event_info["dev_num"], left, top, right, bottom)
        self.testDevice.adb_cmd(cmd)

    def input_encrypt(self):
        bounds = self.d(resourceId="com.android.settings:id/lockPattern").bounds
        self.draw_pattern(bounds)

    def click_next(self):
        if self.d(resourceId="com.android.settings:id/footerRightButton").exists:
            self.d(resourceId="com.android.settings:id/footerRightButton").click()

    def unlock_screen(self):
        #self.d.press.menu()
        self.d(resourceId="com.android.systemui:id/lock_icon").drag.to(resourceId="com.android.systemui:id/clock_view")
        bounds = self.d(resourceId="com.android.systemui:id/lockPatternView").bounds
        self.draw_pattern(bounds)

