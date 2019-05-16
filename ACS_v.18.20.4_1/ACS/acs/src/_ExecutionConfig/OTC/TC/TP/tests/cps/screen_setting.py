import unittest

import time
import sys
from testlib.settings.settings import Settings
from testlib.common.uiatestbase import UIATestBase2
from testlib.common.common import g_common_obj2


class ScreenSetting(UIATestBase2):
    impl = Settings()

    def setUp(self):
        super(ScreenSetting, self).setUp()
        self.impl.setOrientation("n")
        g_common_obj2.unlock()

    def testScreenSetting(self):
        self.impl.launchSetting()
        self.impl.openDisplay()
        self.impl.openDisplaySleepTime()
        self.impl.setDisplaySleepTime("5 minutes")
        self.impl.pressBack()
        self.impl.openSecurity()
        self.impl.openScreenLock()
        self.impl.ChoseScreenLock("None")