from testlib.util.uiatestbase import UIATestBase
from testlib.security.security_impl import SecurityImpl
from testlib.util.common import g_common_obj
import time


class AdbConnectionAndKey(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        self.securityImpl = SecurityImpl()
        print
        print "[Setup]: %s" % self._test_name
        super(AdbConnectionAndKey, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(AdbConnectionAndKey, self).tearDown()

    def test_check_adb_connection_and_key_userdebug_image(self):
        print "[RunTest]: %s" % self.__str__()
        self.securityImpl.logger.info("")
        g_common_obj.adb_cmd_common("devices")
        self.securityImpl.root()
        cmd = "ls -la /adb_keys"
        msg = g_common_obj.adb_cmd_capture_msg(cmd)
        print msg
        assert "adb_keys" in msg
        cat_cmd = "cat /adb_keys"
        cat_adb_keys = g_common_obj.adb_cmd_capture_msg(cat_cmd)
        if cat_adb_keys == None:
            assert False, "adb_keys is null"
