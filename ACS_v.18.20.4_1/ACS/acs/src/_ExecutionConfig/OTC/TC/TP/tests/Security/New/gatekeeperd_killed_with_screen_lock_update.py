from testlib.util.uiatestbase import UIATestBase
from testlib.security.security_impl import SecurityImpl

import time

class GatekeeperdKilledWithScreenLock(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        self.securityImpl = SecurityImpl()
        print
        self.def_passwd = "qwer123"
        self.def_pin = 1234
        self.def_pattern = 14789
        print "[Setup]: %s" % self._test_name
        super(GatekeeperdKilledWithScreenLock, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(GatekeeperdKilledWithScreenLock, self).tearDown()


    def test_gatekeeperd_killed_with_screen_lock_update(self):
        print "[RunTest]: %s" % self.__str__()
        self.securityImpl.logger.info("")
        # 1. Password
        print "[Test_type]------Set lock type with <Password> lock type------"
        time.sleep(3)
        self.securityImpl.set_screen_lock_type_all("Password", self.def_passwd)
        self.securityImpl.action_screen_lock_security("Unlock")
        self.securityImpl.unlock_screen_lock_type_all("Password", 10, self.def_passwd, self.def_passwd)
        try:
            self.securityImpl.check_gatekeeperd_running()
        finally:
            self.securityImpl.remove_screen_lock_type_all("Password", self.def_passwd)
        # 2. PIN
        print "[Test_type]------Set lock type with <PIN> lock type------"
        time.sleep(3)
        self.securityImpl.set_screen_lock_type_all("PIN", self.def_passwd, self.def_pin)
        self.securityImpl.action_screen_lock_security("Unlock")
        self.securityImpl.unlock_screen_lock_type_all("PIN", 10,self.def_passwd,self.def_passwd,self.def_pin,self.def_pin)
        try:
            self.securityImpl.check_gatekeeperd_running()
        finally:
            self.securityImpl.remove_screen_lock_type_all("PIN", self.def_passwd, self.def_pin)
        # 3. Pattern
        time.sleep(3)
        print "[Test_type]------Set lock type with <Pattern> lock type------"
        self.securityImpl.set_screen_lock_type_all("Pattern")
        self.securityImpl.action_screen_lock_security("Unlock")
        self.securityImpl.unlock_screen_lock_type_all("Pattern", 10)
        try:
            self.securityImpl.check_gatekeeperd_running()
        finally:
            self.securityImpl.remove_screen_lock_type_all("Pattern")
