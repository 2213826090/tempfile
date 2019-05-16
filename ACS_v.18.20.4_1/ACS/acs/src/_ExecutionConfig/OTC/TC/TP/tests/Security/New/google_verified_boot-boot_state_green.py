from testlib.util.uiatestbase import UIATestBase
from testlib.security.security_impl import SecurityImpl


class GoogleVerifiedBootStateGreen(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        self.securityImpl = SecurityImpl()
        print
        print "[Setup]: %s" % self._test_name
        super(GoogleVerifiedBootStateGreen, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(GoogleVerifiedBootStateGreen, self).tearDown()


    def test_google_verified_boot_boot_state_green(self):
        print "[RunTest]: %s" % self.__str__()
        self.securityImpl.logger.info("")
        #push file, In order to judge eb case whether flash image
        self.securityImpl.push_send_event_script("draw_pattern_lock.sh")
        self.securityImpl.google_verified_boot_boot_state_green()

