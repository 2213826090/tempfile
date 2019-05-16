from testlib.util.uiatestbase import UIATestBase
from testlib.security.security_impl import SecurityImpl

import time

class DiskEncryptionVoldDecryptStatus(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        self.securityImpl = SecurityImpl()
        print
        self.def_passwd = "qwer123"
        self.def_pin = 1234
        self.def_pattern = 14789
        print "[Setup]: %s" % self._test_name
        self.securityImpl.run_check_disk_encryption_type()
        super(DiskEncryptionVoldDecryptStatus, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(DiskEncryptionVoldDecryptStatus, self).tearDown()


    def test_disk_encryption_lock_type_check_vold_decrypt_status(self):
        print "[RunTest]: %s" % self.__str__()
        self.securityImpl.logger.info("")
        # 1. Password
        print "[Test_type]------Set lock type with <Password> lock type------"
        time.sleep(3)
        self.securityImpl.set_screen_lock_type_all("Password", self.def_passwd)
        self.securityImpl.reboot_devices()
        #reboot and check vold decrypt status
        self.securityImpl.check_vold_decrypt_status("Reboot")
        self.securityImpl.unlock_screen_lock_type_all("Password", 10, self.def_passwd, self.def_passwd)
        # login AOS and check vold decrypt status
        try:
            self.securityImpl.check_vold_decrypt_status("Unlock")
        finally:
            self.securityImpl.remove_screen_lock_type_all("Password", self.def_passwd)
        # 2. PIN
        print "[Test_type]------Set lock type with <PIN> lock type------"
        time.sleep(3)
        self.securityImpl.set_screen_lock_type_all("PIN", self.def_passwd, self.def_pin)
        self.securityImpl.reboot_devices()
        # reboot and check vold decrypt status
        self.securityImpl.check_vold_decrypt_status("Reboot")
        self.securityImpl.unlock_screen_lock_type_all("PIN", 10,self.def_passwd,self.def_passwd,self.def_pin,self.def_pin)
        # login AOS and check vold decrypt status
        try:
            self.securityImpl.check_vold_decrypt_status("Unlock")
        finally:
            self.securityImpl.remove_screen_lock_type_all("PIN", self.def_passwd, self.def_pin)
        # 3. Pattern
        time.sleep(3)
        print "[Test_type]------Set lock type with <Pattern> lock type------"
        self.securityImpl.set_screen_lock_type_all("Pattern")
        self.securityImpl.reboot_devices()
        # reboot and check vold decrypt status
        self.securityImpl.check_vold_decrypt_status("Reboot")
        self.securityImpl.unlock_screen_lock_type_all("Pattern", 10)
        # login AOS and check vold decrypt status
        try:
            self.securityImpl.check_vold_decrypt_status("Unlock")
        finally:
            self.securityImpl.remove_screen_lock_type_all("Pattern")
