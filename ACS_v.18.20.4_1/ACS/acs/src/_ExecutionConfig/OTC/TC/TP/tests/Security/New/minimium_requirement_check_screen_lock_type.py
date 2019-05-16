from testlib.util.uiatestbase import UIATestBase
from testlib.security.security_impl import SecurityImpl



class MinimiumRequirementCheckScreenLock(UIATestBase):

    # HP desc: need use user image
    def setUp(self):
        self._test_name = __name__
        self.securityImpl = SecurityImpl()
        self.def_passwd = "qwer123"
        self.min_passwd = ["1", "q", "q1", "qw1"]
        self.incorrect_passwd_long = "abcdefg123456"
        self.def_pin = 1234
        self.min_pin = [0, 1, 12, 123]
        self.incorrect_pin_long = 1234567890
        # (draw_point_top, draw_point_left), (5, 5)==1 ,(3, 5)==2, (1,5)==3
        self.min_pattern = [5, 3, 1]
        print
        self.securityImpl.choose_screen_lock("None")
        print "[Setup]: %s" % self._test_name
        super(MinimiumRequirementCheckScreenLock, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(MinimiumRequirementCheckScreenLock, self).tearDown()

    def test_minimium_requirement_check_screen_lock_password(self):
        print "[RunTest]: %s" % self.__str__()
        self.securityImpl.logger.info("")
        loop_input_times = len(self.min_passwd)
        for i in range(loop_input_times):
            self.securityImpl.logger.info("Loop input min Password %d/%d " % (i+1, loop_input_times))
            self.securityImpl.minimium_requirement_set_screen_lock_type("Password", self.min_passwd[i])

    def test_minimium_requirement_check_screen_lock_pin(self):
        print "[RunTest]: %s" % self.__str__()
        self.securityImpl.logger.info("")
        loop_input_times = len(self.min_pin)
        for i in range(loop_input_times):
            self.securityImpl.logger.info("Loop input min PIN %d/%d " % ((i + 1), loop_input_times))
            self.securityImpl.minimium_requirement_set_screen_lock_type("PIN", self.def_passwd, self.min_pin[i])

    def test_minimium_requirement_check_screen_lock_pattern(self):
        print "[RunTest]: %s" % self.__str__()
        self.securityImpl.logger.info("")
        loop_input_times = len(self.min_pattern)
        for i in range(loop_input_times):
            self.securityImpl.logger.info("Loop input min Pattern %d/%d " % ((i + 1), loop_input_times))
            self.securityImpl.minimium_requirement_set_screen_lock_type("Pattern", self.min_pattern[i])

