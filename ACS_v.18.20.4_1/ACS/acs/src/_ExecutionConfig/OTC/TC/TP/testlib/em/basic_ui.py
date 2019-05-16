# -*- coding: utf-8 -*-
import time
from tools import UIBase

class LockScreenUI(UIBase):

    def __init__(self, serial = None):
        UIBase.__init__(self, serial)
        self.d = self.testDevice.get_device()

    def lock_screen(self):
        self.d.screen("off")
        time.sleep(2)
        self.d.screen("on")
        time.sleep(2)

    def get_charging_status(self):
        return self.d(text = "Charging").exists or self.d(text = "Charged").exists

    def get_battery_level(self):
        text = self.d(resourceId="com.android.systemui:id/battery_level").text
        level = text.split("%")[0]
        print "[info]--- Battery level: %s" % level
        return int(level)

    def get_battery_icon_rect(self):
        bounds = self.d(resourceId="com.android.systemui:id/battery").bounds
        return (bounds["left"], bounds["top"], bounds["right"], bounds["bottom"])


class Notification(UIBase):

    def __init__(self, serial = None):
        UIBase.__init__(self, serial)
        self.d = self.testDevice.get_device()

    def open(self, open_all = False):
        #self.d.open.notification()
        from constants_def import O_MR1
        if self.get_build_version() == O_MR1:
            self.d.open.notification()
        else:
            info = self.d.info
            x = info["displayWidth"] / 2
            y = info["displayHeight"] / 2
            self.d.swipe(x, 0, x, y, steps = 10)
            if open_all:
                self.d.swipe(x, 0, x, y, steps = 10)

    def get_battery_level(self):
        self.open(True)
        text = self.d(textMatches="\d+%").text
        level = text.split("%")[0]
        #print "[info]--- Get battery level from notification: %s" % level
        self.close()
        return int(level)

    def close(self):
        if self.d(resourceId="com.android.systemui:id/notification_stack_scroller").exists:
            bounds = self.d(resourceId="com.android.systemui:id/notification_stack_scroller").bounds
            info = self.d.info
            x = (bounds["right"] + info["displayWidth"]) / 2
            y = info["displayHeight"] / 2
            self.d.click(x, y)

