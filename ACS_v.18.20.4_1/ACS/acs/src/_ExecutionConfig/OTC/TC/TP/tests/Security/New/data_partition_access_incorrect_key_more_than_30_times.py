from testlib.util.uiatestbase import UIATestBase
from testlib.security.security_impl import SecurityImpl
from testlib.security.init_flash_device import InitFlashDevices
import time

class DataPartitionAccess(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        self.securityImpl = SecurityImpl()
        self.def_passwd = "qwer123"
        self.incorrect_passwd = "qwer"
        self.incorrect_passwd_long = "abcdefg123456"
        self.def_pin = 1234
        self.incorrect_pin = 1111
        self.incorrect_pin_long = 1234567890
        self.init_dut = InitFlashDevices()
        self.securityImpl.start_RPCServer()
        print
        print "[Setup]: %s" % self._test_name
        self.securityImpl.run_check_disk_encryption_type()
        super(DataPartitionAccess, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(DataPartitionAccess, self).tearDown()

    def test_data_partition_access_incorrect_key_more_than_30_times(self):
        # HP desc: need use user image
        # after 30 times, dut will be wiped forcely
        print "[RunTest]: %s" % self.__str__()
        self.securityImpl.logger.info("")
        self.securityImpl.set_screen_lock_type_all("Password", self.def_passwd)
        for i in range(3):
            self.securityImpl.action_screen_lock_security("Reboot")
            if self.securityImpl.check_screen_unlock_need_reboot_at_boot("Password", 10, self.incorrect_passwd_long) is True:
                self.securityImpl.logger.info("------<Check Test> each 10 times wrong check, AOS requires a reboot")
            else:
                print "[Retry]------ Unlock screen password"
                self.securityImpl.reboot_require_unlock_screen_passwd_pin_to_mos(self.def_passwd)
                self.securityImpl.remove_screen_lock_password(self.def_passwd)
                assert False, "[Debug]---Unlock screen lock password is Failed"
        time.sleep(20)
        self.securityImpl.logger.info('After 30 times, dut has been wiped forcely')
        self.init_dut.init_main()
        self.securityImpl.remove_screen_lock_type_all("Password", self.def_passwd)

        print "[RunTest]: %s" % self.__str__()
        self.securityImpl.logger.info("")
        self.securityImpl.set_screen_lock_type_all("PIN", self.def_passwd, self.def_pin)
        for i in range(3):
            self.securityImpl.action_screen_lock_security("Reboot")
            if self.securityImpl.check_screen_unlock_need_reboot_at_boot("PIN", 10, self.def_passwd,self.incorrect_pin_long) is True:
                self.securityImpl.logger.info("------<Check Test> each 10 times wrong check, AOS requires a reboot")
            else:
                print "[Retry]------ Unlock screen PIN"
                self.securityImpl.reboot_require_unlock_screen_passwd_pin_to_mos(self.def_pin)
                self.securityImpl.remove_screen_lock_pin(self.def_pin)
                assert False, "[Debug]---Unlock screen lock PIN is Failed"
        time.sleep(20)
        self.securityImpl.logger.info('After 30 times, dut has been wiped forcely')
        self.init_dut.init_main()
        self.securityImpl.remove_screen_lock_type_all("PIN", self.def_passwd, self.def_pin)

        print "[RunTest]: %s" % self.__str__()
        self.securityImpl.logger.info("")
        self.securityImpl.set_screen_lock_type_all("Pattern")
        for i in range(2):
            self.securityImpl.action_screen_lock_security("Reboot")
            if self.securityImpl.unlock_screen_lock_incorrect_pattern(10) is True:
                self.securityImpl.logger.info("------<Check Test> each 10 times wrong check, AOS requires a reboot")
            else:
                print "[Retry]------ Unlock screen Pattern"
                self.securityImpl.reboot_unlock_reusme_unlock_pattern()
                self.securityImpl.remove_screen_lock_pattern()
                assert False, "[Debug]---Unlock screen lock Pattern is Failed"
        self.securityImpl.action_screen_lock_security("Reboot")
        self.securityImpl.unlock_screen_lock_incorrect_pattern(10)
        time.sleep(30)
        self.securityImpl.logger.info('After 30 times, dut has been wiped forcely')
        self.init_dut.init_main()
        self.securityImpl.remove_screen_lock_type_all("Pattern")

