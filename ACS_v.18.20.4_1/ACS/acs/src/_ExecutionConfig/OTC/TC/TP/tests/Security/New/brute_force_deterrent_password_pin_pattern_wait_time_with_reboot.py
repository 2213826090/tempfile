from testlib.util.uiatestbase import UIATestBase
from testlib.security.security_impl import SecurityImpl

import time


class ScreenLockTypeUnlockWaitTime(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        self.securityImpl = SecurityImpl()
        self.def_passwd = "qwer123"
        self.incorrect_passwd = "qwer"
        self.incorrect_passwd_long = "abcdefg123456"
        self.def_pin = 1234
        self.incorrect_pin = 1111
        self.incorrect_pin_long = 1234567890
        print
        print "[Setup]: %s" % self._test_name
        super(ScreenLockTypeUnlockWaitTime, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(ScreenLockTypeUnlockWaitTime, self).tearDown()

    def test_brute_force_password_pin_pattern_wait_time_with_reboot(self):
        print "[RunTest]: %s" % self.__str__()
        self.securityImpl.logger.info("")
        # 1. Password
        print "[Test_type]------Test lock type with <Password> lock type------"
        self.securityImpl.set_screen_lock_type_all("Password", self.def_passwd)
        self.securityImpl.action_screen_lock_security("Unlock")
        try:
            self.securityImpl.resume_unlock_screen_lock_type_all("Password", 30, self.def_passwd, self.incorrect_passwd_long,
                                                                 self.def_pin, self.incorrect_pin_long, "reboot")
        finally:
            self.securityImpl.remove_resume_screen_lock_type_all("Password", self.def_passwd)

        print "[RunTest]: %s" % self.__str__()
        self.securityImpl.logger.info("")
        # 2. PIN
        print "[Test_type]------Test lock type with <PIN> lock type------"
        self.securityImpl.set_screen_lock_type_all("PIN", self.def_passwd, self.def_pin)
        self.securityImpl.action_screen_lock_security("Unlock")
        try:
            self.securityImpl.resume_unlock_screen_lock_type_all("PIN", 30, self.def_passwd, self.incorrect_passwd_long,
                                                                 self.def_pin, self.incorrect_pin_long, "reboot")
        finally:
            self.securityImpl.remove_resume_screen_lock_type_all("PIN", self.def_passwd, self.def_pin)

        print "[RunTest]: %s" % self.__str__()
        self.securityImpl.logger.info("")
        # 3. Pattern
        time.sleep(3)
        print "[Test_type]------Test lock type with <Pattern> lock type------"
        self.securityImpl.set_screen_lock_type_all("Pattern")
        self.securityImpl.action_screen_lock_security("Unlock")
        try:
            self.securityImpl.resume_unlock_screen_lock_type_all("Pattern", 30, "incorrect_pattern", "reboot")
        finally:
            self.securityImpl.remove_resume_screen_lock_type_all("Pattern")


