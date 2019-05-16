from testlib.util.uiatestbase import UIATestBase
from testlib.security.security_impl import SecurityImpl

import time


class DiskEncryptionKeyUpdateLockType(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        self.securityImpl = SecurityImpl()
        self.def_passwd = "qwer123"
        self.def_pin = 1234
        self.def_pattern = 14789
        self.update_passwd = "qwer123456"
        self.update_pin = 123456
        self.update_pattern = 4789
        print
        print "[Setup]: %s" % self._test_name
        self.securityImpl.run_check_disk_encryption_type()
        super(DiskEncryptionKeyUpdateLockType, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        self.securityImpl.choose_screen_lock("None")
        super(DiskEncryptionKeyUpdateLockType, self).tearDown()


    def test_disk_update_password_key_on_without_lock_key(self):
        print "[RunTest]: %s" % self.__str__()
        self.securityImpl.logger.info("")
        self.securityImpl.choose_screen_lock("None")
        self.securityImpl.upate_lock_type_with_None_type_all("Password", self.def_passwd)

    def test_disk_update_pin_key_on_without_lock_key(self):
        print "[RunTest]: %s" % self.__str__()
        self.securityImpl.logger.info("")
        self.securityImpl.choose_screen_lock("None")
        self.securityImpl.upate_lock_type_with_None_type_all("PIN", self.def_passwd, self.def_pin)

    def test_disk_update_pattern_key_on_without_lock_key(self):
        print "[RunTest]: %s" % self.__str__()
        self.securityImpl.logger.info("")
        self.securityImpl.choose_screen_lock("None")
        self.securityImpl.upate_lock_type_with_None_type_all("Pattern", self.def_passwd, self.def_pin,self.def_pattern)


    def test_disk_update_lock_type_key_on_passwd_pin_pattern_switch_lock(self):
        print "[RunTest]: %s" % self.__str__()
        self.securityImpl.logger.info("")
        #1. Password
        print "[Test_type]------Update lock type with <Password> lock type------"
        time.sleep(3)
        self.securityImpl.set_screen_lock_type_all("Password", self.def_passwd)
        self.securityImpl.upate_lock_type_with_lock_type_all("Password", self.update_passwd)
        self.securityImpl.unlock_screen_lock_password(10, self.def_passwd)
        self.securityImpl.remove_screen_lock_type_all("Password", self.def_passwd)
        #2. PIN
        print "[Test_type]------Update lock type with <PIN> lock type------"
        time.sleep(3)
        self.securityImpl.set_screen_lock_type_all("PIN", self.def_passwd, self.def_pin)
        self.securityImpl.upate_lock_type_with_lock_type_all("PIN", self.update_passwd, self.update_pin)
        self.securityImpl.unlock_screen_lock_pin(10, self.def_pin)
        self.securityImpl.remove_screen_lock_type_all("PIN", self.def_passwd, self.def_pin)
        #3. Pattern
        time.sleep(3)
        print "[Test_type]------Update lock type with <Pattern> lock type------"
        self.securityImpl.set_screen_lock_type_all("Pattern")
        self.securityImpl.upate_lock_type_with_lock_type_all("Pattern", self.update_passwd, self.update_pin,self.update_pattern)
        self.securityImpl.unlock_screen_lock_type_all("Pattern", 10)
        self.securityImpl.remove_screen_lock_type_all("Pattern")