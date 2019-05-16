'''
@summary: Test gota low power can get reasonable prompt
@since: 5/20/2015
@author: Yang, NaX <nax.yang@intel.com>
'''
import os
from testlib.util.uiatestbase import UIATestBase
from testlib.gota.gota_impl import gotaImpl

class LowPowerGetReasonablePrompt(UIATestBase):
    """
    @summary: Test gota low power can get reasonable prompt
    """

    def setUp(self):
        super(LowPowerGetReasonablePrompt, self).setUp()
        cfg_file = os.path.join(os.environ.get('TEST_DATA_ROOT', ''),'tests.tablet.gota.conf')
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.cfg = self.config.read(cfg_file, 'gota')
        self.gota= gotaImpl(self.cfg)

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        self.gota.set_virtual_battery_level(8)
        super(LowPowerGetReasonablePrompt, self).tearDown()
        self.cfg = None

    def testLowPowerGetReasonablePrompt(self):
        """
        This test case is to test gota low power can get reasonable prompt

        Test Case Precondition:
        set low power, and get reasonable prompt

        Test Case Step:
        1. test gota low power can get reasonable prompt

        Expect Result:
        1. set low power
        2. get reasonable prompt

        The real implementation will be in SystemImpl class.
        """

        print "[RunTest]: %s" % self.__str__()

        self.gota.enter_system_updates()
        self.gota.set_virtual_battery_level(0)
        self.gota.low_power_get_reasonable_prompt()