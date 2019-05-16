from testlib.util.uiatestbase import UIATestBase
from testlib.security.security_impl import SecurityImpl

import time


class LockTypeUnlockMoreThan30times(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        self.securityImpl = SecurityImpl()
        self.def_passwd = "qwer123"
        self.incorrect_passwd = "qwer"
        self.incorrect_passwd_long = "abcdefg123456"
        self.def_pin = 1234
        self.incorrect_pin = 1111
        self.incorrect_pin_long = 1234567890
        if 1 == 1:
            raise Exception("Debug case on going, skip it, next time run this case")
        print
        print "[Setup]: %s" % self._test_name
        self.securityImpl.start_RPCServer()
        super(LockTypeUnlockMoreThan30times, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(LockTypeUnlockMoreThan30times, self).tearDown()

    def test_brute_force_password_more_than_30times(self):
        print "[RunTest]: %s" % self.__str__()
        self.securityImpl.logger.info("")
        # 1. Password
        print "[Test_type]------Test lock type with <Password> lock type------"
        self.securityImpl.set_screen_lock_type_all("Password", self.def_passwd)
        self.securityImpl.action_screen_lock_security("Unlock")
        try:
            self.securityImpl.resume_unlock_screen_lock_type_all("Password", 32, self.def_passwd, self.incorrect_passwd_long,
                                                          self.def_pin, self.incorrect_pin_long)
            self.securityImpl.remove_resume_screen_lock_type_all("Password", self.def_passwd)
        except Exception, e:
            print e
            if "RPC server not started" in e:
                self.securityImpl.check_RPC_server_not_start(self.def_passwd)
                self.securityImpl.remove_resume_screen_lock_type_all("Password", self.def_passwd)
            raise Exception(e)

    def test_brute_force_pin_more_than_30times(self):
        print "[RunTest]: %s" % self.__str__()
        self.securityImpl.logger.info("")
        # 2. PIN
        print "[Test_type]------Test lock type with <PIN> lock type------"
        self.securityImpl.set_screen_lock_type_all("PIN", self.def_passwd, self.def_pin)
        self.securityImpl.action_screen_lock_security("Unlock")
        try:
            self.securityImpl.resume_unlock_screen_lock_type_all("PIN", 32, self.def_passwd, self.incorrect_passwd_long,
                                                          self.def_pin, self.incorrect_pin_long)
            self.securityImpl.remove_resume_screen_lock_type_all("PIN", self.def_passwd, self.def_pin)
        except Exception, e:
            print e
            if "RPC server not started" in e:
                self.securityImpl.check_RPC_server_not_start(self.def_pin)
                self.securityImpl.remove_resume_screen_lock_type_all("PIN", self.def_passwd, self.def_pin)
            raise Exception(e)

    def test_brute_force_pattern_more_than_30times(self):
        print "[RunTest]: %s" % self.__str__()
        self.securityImpl.logger.info("")
        # 3. Pattern
        time.sleep(3)
        print "[Test_type]------Test lock type with <Pattern> lock type------"
        self.securityImpl.set_screen_lock_type_all("Pattern")
        self.securityImpl.action_screen_lock_security("Unlock")
        try:
            self.securityImpl.resume_unlock_screen_lock_type_all("Pattern", 32, "incorrect_pattern")
        finally:
            self.securityImpl.remove_resume_screen_lock_type_all("Pattern")

