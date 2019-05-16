from testlib.util.uiatestbase import UIATestBase
from testlib.security.security_impl import SecurityImpl


class GatekeeperdRunning(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        self.securityImpl = SecurityImpl()
        print
        print "[Setup]: %s" % self._test_name
        super(GatekeeperdRunning, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(GatekeeperdRunning, self).tearDown()


    def test_gatekeeperd_running_in_background(self):
        print "[RunTest]: %s" % self.__str__()
        self.securityImpl.logger.info("")
        self.securityImpl.check_gatekeeperd_running()

