from testlib.util.uiatestbase import UIATestBase
from testlib.system_touch.system_touch import SystemTouch

touch = SystemTouch()

class GestureLongPress(UIATestBase):

    def test_gesture_long_press_screen(self):
        print "[RunTest]: %s" % self.__str__()
        touch.long_press_build_comm()
