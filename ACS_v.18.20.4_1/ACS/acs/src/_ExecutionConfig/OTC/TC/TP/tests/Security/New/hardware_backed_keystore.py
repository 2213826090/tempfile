from testlib.util.uiatestbase import UIATestBase
from testlib.security.security_impl import SecurityImpl


class HardwareBackedKeystore(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        self.securityImpl = SecurityImpl()
        print
        print "[Setup]: %s" % self._test_name
        super(HardwareBackedKeystore, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(HardwareBackedKeystore, self).tearDown()


    def test_hardware_backed_keystore(self):
        print "[RunTest]: %s" % self.__str__()
        self.securityImpl.logger.info("")
        self.securityImpl.hardware_backed_keystore()


    def test_hardware_backed_keystore_libary_check(self):
        print "[RunTest]: %s" % self.__str__()
        self.securityImpl.logger.info("")
        self.securityImpl.hardware_backed_keystore_libary_check()