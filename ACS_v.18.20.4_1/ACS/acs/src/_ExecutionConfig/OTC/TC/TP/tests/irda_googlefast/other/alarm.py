# coding: UTF-8
import os, sys
import time
from testlib.common.uiatestbase import UIATestBase2
from testlib.clock.clock import Clock as Impl

class Alarm(UIATestBase2):
    impl = Impl
    def testSetAlarm(self):
        self.impl.launchAlarm()
        self.impl.setAlarm2()
        self.impl.returnToHomeScreenAndSleep()
        ret = self.impl.waitForAlarm()
        #self.delAlarm()
        self.impl.unlock()
        self.impl.delAlarm2()
        assert ret



