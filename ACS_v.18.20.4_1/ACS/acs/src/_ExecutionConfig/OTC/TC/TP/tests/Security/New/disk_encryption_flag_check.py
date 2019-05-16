from testlib.util.uiatestbase import UIATestBase
from testlib.security.security_impl import SecurityImpl


class DiskEncryptionFlagCheck(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        self.securityImpl = SecurityImpl()
        print
        print "[Setup]: %s" % self._test_name
        super(DiskEncryptionFlagCheck, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(DiskEncryptionFlagCheck, self).tearDown()


    def test_disk_encryption_flag_check(self):
        print "[RunTest]: %s" % self.__str__()
        self.securityImpl.logger.info("")
        self.securityImpl.disk_encryption_flag_check()
