'''
@summary: test contacts data exist in personal profile after gota
@since: 25/5/2015
@author: Yang, NaX <nax.yang@intel.com>
'''

import os
from testlib.util.uiatestbase import UIATestBase
from testlib.gota.gota_impl import gotaImpl

class CheckAFWDataInpersonalProfileAfterGOTA(UIATestBase):

    def setUp(self):
        super(CheckAFWDataInpersonalProfileAfterGOTA, self).setUp()
        self._test_name = __name__
        cfg_file = os.path.join(os.environ.get('TEST_DATA_ROOT', ''), 'tests.tablet.gota.conf')
        self.cfg = self.config.read(cfg_file, 'gota')
        self.gota = gotaImpl(self.cfg)
        print "[Setup]: %s" % self._test_name

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(CheckAFWDataInpersonalProfileAfterGOTA, self).tearDown()
        self.cfg = None

    def testAFWDataInPersonalProfile(self):
        print "[RunTest]: %s" % self.__str__()
        self.gota.check_AFW_data_in_personal_profile_after_GOTA()