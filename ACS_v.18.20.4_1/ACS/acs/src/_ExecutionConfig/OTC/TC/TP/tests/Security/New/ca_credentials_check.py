from testlib.util.uiatestbase import UIATestBase
from testlib.security.security_impl import SecurityImpl


class CACredentialsCheck(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        self.securityImpl = SecurityImpl()
        print
        print "[Setup]: %s" % self._test_name
        super(CACredentialsCheck, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(CACredentialsCheck, self).tearDown()

    def test_ca_credentials_check(self):
        print "[RunTest]: %s" % self.__str__()
        self.securityImpl.logger.info("")
        self.securityImpl.check_CA_credentials_comm()
