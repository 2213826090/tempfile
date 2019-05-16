# coding: UTF-8
import os, sys
import time
from testlib.common.uiatestbase import UIATestBase2
from testlib.clock.clock import Clock as Impl
from testlib.system_domains.system_impl import SystemImpl

class Alarm(UIATestBase2):
    impl = Impl
    hal=SystemImpl
    def testSetAlarm(self):
        alarm='Alarm'
        self.impl.launchAlarm()
        self.impl.setAlarm2()
        self.impl.returnToHomeScreenAndSleep()
        self.hal.snooze_alarm()
        self.hal.wake_up()
        ret=self.hal.check_notification(alarm)
        self.hal.wake_up()
        self.hal.del_all_alarm()
        assert ret



