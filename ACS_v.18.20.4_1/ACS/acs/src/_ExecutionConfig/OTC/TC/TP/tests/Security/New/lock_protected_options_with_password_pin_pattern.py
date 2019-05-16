from testlib.util.uiatestbase import UIATestBase
from testlib.security.security_impl import SecurityImpl

import time

class LockProtectedOptionsPasswdPinPattern(UIATestBase):

    # HP desc: need use user image
    def setUp(self):
        self._test_name = __name__
        self.securityImpl = SecurityImpl()
        print
        self.def_passwd = "qwer123"
        self.def_pin = 1234
        self.def_pattern = 14789
        print "[Setup]: %s" % self._test_name
        super(LockProtectedOptionsPasswdPinPattern, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(LockProtectedOptionsPasswdPinPattern, self).tearDown()

    def test_lock_protected_options_with_password_pin_pattern(self):
        print "[RunTest]: %s" % self.__str__()
        self.securityImpl.logger.info("")
        # 1. Password
        print "[Test_type]------Set lock type with <Password> lock type------"
        time.sleep(3)
        self.securityImpl.set_screen_lock_type_all("Password", self.def_passwd)
        try:
            self.securityImpl.smart_lock_car_ble_trust_agent("Password", self.def_passwd)
            self.securityImpl.oem_unlocking_enable_or_password_lock(True, "Password", self.def_passwd)
            self.securityImpl.oem_unlocking_enable_or_password_lock(False, "Password", self.def_passwd)
        finally:
            self.securityImpl.remove_screen_lock_type_all("Password", self.def_passwd)
        # 2. PIN
        print "[Test_type]------Set lock type with <PIN> lock type------"
        time.sleep(3)
        self.securityImpl.set_screen_lock_type_all("PIN", self.def_passwd, self.def_pin)
        try:
            self.securityImpl.smart_lock_car_ble_trust_agent("PIN", self.def_pin)
            self.securityImpl.oem_unlocking_enable_or_password_lock(True, "PIN", self.def_pin)
            self.securityImpl.oem_unlocking_enable_or_password_lock(False, "PIN", self.def_pin)
        finally:
            self.securityImpl.remove_screen_lock_type_all("PIN", self.def_passwd, self.def_pin)
        # 3. Pattern
        time.sleep(3)
        print "[Test_type]------Set lock type with <Pattern> lock type------"
        self.securityImpl.set_screen_lock_type_all("Pattern")
        try:
            self.securityImpl.smart_lock_car_ble_trust_agent("Pattern")
            self.securityImpl.oem_unlocking_enable_or_password_lock(True, "Pattern")
            self.securityImpl.oem_unlocking_enable_or_password_lock(False, "Pattern")
        finally:
            self.securityImpl.remove_screen_lock_type_all("Pattern")

