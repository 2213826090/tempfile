from testlib.util.uiatestbase import UIATestBase
from testlib.security.security_impl import SecurityImpl


class FBEenableCheck(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        self.securityImpl = SecurityImpl()
        self.securityImpl.run_check_file_encryption_type()
        self.def_passwd = "qwer123"
        print
        print "[Setup]: %s" % self._test_name
        super(FBEenableCheck, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(FBEenableCheck, self).tearDown()


    def test_direct_boot_fbe_enable_check(self):
        print "[RunTest]: %s" % self.__str__()
        self.securityImpl.logger.info("")
        self.securityImpl.file_encryption_flag_check()

    def test_direct_boot_fbe_enable_check_for_encrypted_file(self):
        print "[RunTest]: %s" % self.__str__()
        self.securityImpl.logger.info("")
        self.securityImpl.fbe_enable_check_for_encrypted_file()
        # 1. Password
        print "[Test_type]------Test lock type with <Password> lock type------"
        self.securityImpl.set_screen_lock_type_all("Password", self.def_passwd)
        try:
            self.securityImpl.action_screen_lock_security("Reboot")
            self.securityImpl.fbe_enable_check_for_encrypted_file("Password")
            self.securityImpl.unlock_screen_lock_type_all("Password", 5, self.def_passwd, self.def_passwd)
        finally:
            self.securityImpl.remove_screen_lock_type_all("Password", self.def_passwd)

