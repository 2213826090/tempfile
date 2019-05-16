from testlib.util.uiatestbase import UIATestBase
from testlib.util.common import g_common_obj

class GeteventNoTouch(UIATestBase):

    def test_getevent_output_no_touch(self):
        print "[RunTest]: %s" % self.__str__()
        wait_time = 60
        cmd = "timeout %s getevent | grep -e /dev/input/event[0-9]:" % wait_time
        msg = g_common_obj.adb_cmd_capture_msg(cmd, time_out = wait_time + 3)
        assert "" == msg

