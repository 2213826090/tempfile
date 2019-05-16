from testlib.util.uiatestbase import UIATestBase
from testlib.security.security_impl import SecurityImpl


class SelinuxModeCheck(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        self.securityImpl = SecurityImpl()
        print
        print "[Setup]: %s" % self._test_name
        super(SelinuxModeCheck, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(SelinuxModeCheck, self).tearDown()

    def test_selinux_mode_check(self):
        print "[RunTest]: %s" % self.__str__()
        self.securityImpl.logger.info("")
        self.securityImpl.selinux_mode_check()