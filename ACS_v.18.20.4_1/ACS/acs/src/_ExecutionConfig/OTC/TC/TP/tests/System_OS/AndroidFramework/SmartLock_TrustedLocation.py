# -*- coding:utf-8 -*-

'''
@summary: Test SmartLock TrustedLocation.
@since: 07/07/2016
@author: Lijin Xiong
'''

import time
from testlib.util.uiatestbase import UIATestBase
from testlib.androidframework.common import UiAutomatorUtils,Settings,AdbUtils
from testlib.util.log import Logger
from testlib.util.common import g_common_obj
from testlib.util.constant import LOCK_UNLOCK_SCREEN

LOG = Logger.getlogger(__name__)

TEST_PIN = "1234"
ANDROID_KEYGUARD_PIN_VIEW_RESID = "com.android.systemui:id/keyguard_pin_view"
SETTINGS_TRUSTED_FIRST_TIME_RESID = "com.google.android.gms:id/auth_trust_agent_first_use_notification_button_id"

class SmartLockTrustedLocation(UIATestBase):

    def setUp(self):
        super(SmartLockTrustedLocation,self).setUp()
        self.__test_name = __name__
        print "Setup: %s" % self.__test_name
        self.d = g_common_obj.get_device()
        UiAutomatorUtils.unlock_screen()

    def tearDown(self):
#            UiAutomatorUtils.close_all_tasks()
#            Settings.disable_location()
        Settings.disable_pin()
        UiAutomatorUtils.close_all_tasks()

    def test_SmartLock_TrustedLocation(self):
        trusted_place = "Havana, Cuba"
        try:
            # Enable location
            LOG.info("Enabling location")
            Settings.enable_location()

            # Enable screen lock pin and add a trusted place (current location)
            LOG.info("Enabling PIN screen lock")
            Settings.enable_pin()
            LOG.info("Adding trusted place - current location")
            Settings.add_trusted_place()

            self.d.press.home()
            # Lock the screen
            LOG.info("Locking the screen")
            self.d.press.power()
            time.sleep(5)

            # Unlock the device
            LOG.info("Unlocking the screen")
            UiAutomatorUtils.unlock_screen()

            LOG.info("Checking the PIN keyguard in NOT present")
            assert not self.d(resourceId=ANDROID_KEYGUARD_PIN_VIEW_RESID).wait.exists(
                    timeout=5000), "The keyguard should not be present in the current trusted place"
            time.sleep(3)

            #Below "if" codes is to handle an info window of trusted unlocking.
            if self.d(resourceId=SETTINGS_TRUSTED_FIRST_TIME_RESID).exists:
                self.d(resourceId=SETTINGS_TRUSTED_FIRST_TIME_RESID).click()

            # Remove the current trusted place
            LOG.info("Removing the current trusted place")
            Settings.remove_trusted_place()

            # Add a trusted place in another location
            LOG.info("Adding trusted place - {0}".format(trusted_place))
            Settings.add_trusted_place(trusted_place)

            self.d.press.home()
            # Lock the screen
            LOG.info("Locking the screen")
            self.d.press.power()
            time.sleep(2)

            # Unlock the device
            LOG.info("unlocking the screen")
            UiAutomatorUtils.unlock_screen()

            LOG.info("Checking the PIN keyguard is present in a non-trusted place")
            assert self.d(resourceId=ANDROID_KEYGUARD_PIN_VIEW_RESID).wait.exists(
                    timeout=5000), "The keyguard should appear when not in a trusted location"

            # Input the PIN via shell
            AdbUtils.input_text(TEST_PIN)
            time.sleep(1)
            self.d.press.enter()
            time.sleep(3)

            # Remove the trusted place
            LOG.info("Removing the trusted place - {0}".format(trusted_place))
            Settings.remove_trusted_place(trusted_place)
        except Exception, e:
            AdbUtils.input_text(TEST_PIN)
            time.sleep(3)
            self.d.press.enter()
            time.sleep(5)
            raise e
        finally:
            Settings.remove_trusted_place()
            self.d.press.home()
            Settings.disable_pin()