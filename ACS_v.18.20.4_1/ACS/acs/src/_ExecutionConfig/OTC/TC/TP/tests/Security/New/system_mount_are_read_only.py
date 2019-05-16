from testlib.util.uiatestbase import UIATestBase
from testlib.security.security_impl import SecurityImpl


class SystemMountAreReadOnly(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        self.securityImpl = SecurityImpl()
        print
        print "[Setup]: %s" % self._test_name
        super(SystemMountAreReadOnly, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(SystemMountAreReadOnly, self).tearDown()

    def test_system_mount_are_read_only(self):
        print "[RunTest]: %s" % self.__str__()
        self.securityImpl.logger.info("")
        self.securityImpl.system_mount_are_read_only()