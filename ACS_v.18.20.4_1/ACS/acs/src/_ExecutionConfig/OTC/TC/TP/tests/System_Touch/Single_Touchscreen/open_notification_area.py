from testlib.util.uiatestbase import UIATestBase
from testlib.system_touch.system_touch import SystemTouch

touch = SystemTouch()

class OpenNotification(UIATestBase):

    def test_open_notification(self):
        print "[RunTest]: %s" % self.__str__()
        touch.open_notification_by_swipe()

