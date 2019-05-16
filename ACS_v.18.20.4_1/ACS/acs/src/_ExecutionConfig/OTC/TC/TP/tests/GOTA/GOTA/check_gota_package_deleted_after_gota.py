'''
@summary: test gota package deteled after GOTA update
@since: 1/6/2015
@author: Yang, NaX <nax.yang@intel.com>
'''
import os
from testlib.util.uiatestbase import UIATestBase
from testlib.gota.gota_impl import gotaImpl

class GotaPackageDeletedAfterGOTA(UIATestBase):

    def setUp(self):
        super(GotaPackageDeletedAfterGOTA, self).setUp()
        self._test_name = __name__
        cfg_file = os.path.join(os.environ.get('TEST_DATA_ROOT', ''), 'tests.tablet.gota.conf')
        self.cfg = self.config.read(cfg_file, 'gota')
        self.gota = gotaImpl(self.cfg)
        print "[Setup]: %s" % self._test_name

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(GotaPackageDeletedAfterGOTA, self).tearDown()
        self.cfg = None

    def testGotaPackageDeletedAfterGOTA(self):
        print "[RunTest]: %s" % self.__str__()
        self.gota.check_gota_package_deleted()