from testlib.util.uiatestbase import UIATestBase
from testlib.systemui.systemui_impl import SystemUI

class Widget(UIATestBase):
    """
    @summary: Test add and remove widget
    """
    def setUp(self):
        super(Widget, self).setUp()
        self._test_name = __name__
        print
        print "[Setup]: %s" % self._test_name
        self.systemui = SystemUI()

    def tearDown(self):
        super(Widget, self).tearDown()
        print "[Teardown]: %s" % self._test_name

    def testWidget(self):
        """
        This test case is to add and remove widget

        Test Case Step:
        1. Add a widget
        2. Remove the widget
        """
        print "[RunTest]: %s" % self.__str__()

        self.systemui.add_clock_to_widget()
        self.systemui.launch_clock_from_widget()
        self.systemui.rm_clock_widget()
