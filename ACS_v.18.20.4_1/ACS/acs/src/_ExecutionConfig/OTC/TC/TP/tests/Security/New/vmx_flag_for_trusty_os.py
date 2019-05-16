from testlib.util.uiatestbase import UIATestBase
from testlib.security.security_impl import SecurityImpl


class VMXFlagForTrustyOs(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        self.securityImpl = SecurityImpl()
        print
        print "[Setup]: %s" % self._test_name
        super(VMXFlagForTrustyOs, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(VMXFlagForTrustyOs, self).tearDown()

    def test_vmx_flag_for_trusty_os(self):
        print "[RunTest]: %s" % self.__str__()
        self.securityImpl.logger.info("")
        self.securityImpl.vmx_flag_for_trusty_os()