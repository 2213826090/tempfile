'''
@summary: test profile work Sample MDM is successful after gota
@since: 25/5/2015
@author: Yang, NaX <nax.yang@intel.com>
'''
import os
from testlib.util.uiatestbase import UIATestBase
from testlib.gota.gota_impl import gotaImpl

class CheckProfileWorkAfterGOTA(UIATestBase):

    def setUp(self):
        super(CheckProfileWorkAfterGOTA, self).setUp()
        self._test_name = __name__
        cfg_file = os.path.join(os.environ.get('TEST_DATA_ROOT', ''), 'tests.tablet.gota.conf')
        self.cfg = self.config.read(cfg_file, 'gota')
        self.gota = gotaImpl(self.cfg)
        print "[Setup]: %s" % self._test_name

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(CheckProfileWorkAfterGOTA, self).tearDown()
        self.cfg = None

    def testCheckProfileWorkAfterGOTA(self):
        print "[RunTest]: %s" % self.__str__()
        self.gota.check_AFW_profile_work_after_GOTA()