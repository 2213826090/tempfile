from testlib.util.uiatestbase import UIATestBase
from testlib.security.security_impl import SecurityImpl

import time

class ScreenLockPasswdPinPatternUpdate(UIATestBase):

    # HP desc: need use user image
    def setUp(self):
        self._test_name = __name__
        self.securityImpl = SecurityImpl()
        print
        self.def_passwd = "qwer123"
        self.def_pin = 1234
        self.def_pattern = 14789
        print "[Setup]: %s" % self._test_name
        super(ScreenLockPasswdPinPatternUpdate, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(ScreenLockPasswdPinPatternUpdate, self).tearDown()

    def test_screen_lock_with_password_pin_pattern_update(self):
        for loop in range(10):
            self.securityImpl.logger.info("------<Start Test>loop input %d/10 password_pin_pattern update" %(loop + 1))
            print "[RunTest]: %s" % self.__str__()
            self.securityImpl.logger.info("")
            # 1. Password
            print "[Test_type]------Set lock type with <Password> lock type------"
            time.sleep(3)
            self.securityImpl.set_screen_lock_type_all("Password", self.def_passwd)
            self.securityImpl.action_screen_lock_security("Unlock")
            self.securityImpl.unlock_screen_lock_type_all("Password", 5, self.def_passwd, self.def_passwd)
            self.securityImpl.remove_screen_lock_type_all("Password", self.def_passwd)
            # 2. PIN
            print "[Test_type]------Set lock type with <PIN> lock type------"
            time.sleep(3)
            self.securityImpl.set_screen_lock_type_all("PIN", self.def_passwd, self.def_pin)
            self.securityImpl.action_screen_lock_security("Unlock")
            self.securityImpl.unlock_screen_lock_type_all("PIN", 5,self.def_passwd,self.def_passwd,self.def_pin,self.def_pin)
            self.securityImpl.remove_screen_lock_type_all("PIN", self.def_passwd, self.def_pin)
            # 3. Pattern
            time.sleep(3)
            print "[Test_type]------Set lock type with <Pattern> lock type------"
            self.securityImpl.set_screen_lock_type_all("Pattern")
            self.securityImpl.action_screen_lock_security("Unlock")
            self.securityImpl.unlock_screen_lock_type_all("Pattern", 5)
            self.securityImpl.remove_screen_lock_type_all("Pattern")
