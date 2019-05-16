from testlib.util.uiatestbase import UIATestBase
from testlib.security.security_impl import SecurityImpl

import time


class ScreenLockTypeAtBootTime(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        self.securityImpl = SecurityImpl()
        self.def_passwd = "qwer123"
        self.incorrect_passwd = "qwer"
        self.incorrect_passwd_long = "abcdefg123456"
        self.def_pin = 1234
        self.incorrect_pin = 1111
        self.incorrect_pin_long = 1234567890
        self.securityImpl.start_RPCServer()
        print
        print "[Setup]: %s" % self._test_name
        self.securityImpl.run_check_disk_encryption_type()
        super(ScreenLockTypeAtBootTime, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(ScreenLockTypeAtBootTime, self).tearDown()

    def test_boot_with_screen_lock_password(self):
        print "[RunTest]: %s" % self.__str__()
        self.securityImpl.logger.info("")
        self.securityImpl.set_screen_lock_type_all("Password", self.def_passwd)
        self.securityImpl.action_screen_lock_security("Reboot")
        self.securityImpl.unlock_screen_lock_type_all("Password", 10, self.def_passwd, self.incorrect_passwd_long,
                                                      self.def_pin, self.incorrect_pin_long)
        self.securityImpl.remove_screen_lock_type_all("Password", self.def_passwd)

    def test_boot_with_screen_lock_pin(self):
        print "[RunTest]: %s" % self.__str__()
        self.securityImpl.logger.info("")
        self.securityImpl.set_screen_lock_type_all("PIN", self.def_passwd, self.def_pin)
        self.securityImpl.action_screen_lock_security("Reboot")
        self.securityImpl.unlock_screen_lock_type_all("PIN", 10, self.def_passwd, self.incorrect_passwd_long,
                                                      self.def_pin, self.incorrect_pin_long)
        self.securityImpl.remove_screen_lock_type_all("PIN", self.def_passwd, self.def_pin)

    def test_boot_with_screen_lock_pattern(self):
        print "[RunTest]: %s" % self.__str__()
        self.securityImpl.logger.info("")
        self.securityImpl.set_screen_lock_type_all("Pattern")
        self.securityImpl.action_screen_lock_security("Reboot")
        self.securityImpl.unlock_screen_lock_type_all("Pattern", 10, "incorrect_pattern")
        self.securityImpl.remove_screen_lock_type_all("Pattern")
