'''
@summary: test gmail after gota
@since: 9/10/2015
@author: Su Chaonan(chaonanx.su@intel.com)
'''
import os
import string
import commands
from testlib.util.uiatestbase import UIATestBase
from testlib.gota.gota_impl import gotaImpl
from testlib.util.common import g_common_obj
from testlib.systemui.systemui_impl import SystemUI
from testlib.system.system_impl import SystemImpl
from testlib.domains.gmail_impl import GmailImpl
class CheckGmailAfterGOTA(UIATestBase):

    def setUp(self):
        super(CheckGmailAfterGOTA, self).setUp()
        cfg_file = os.path.join(os.environ.get('TEST_DATA_ROOT', ''), \
            'tests.tablet.gota.conf')
        self._test_name = __name__
        self.cfg = self.config.read(cfg_file, 'gota')
        self.gota= gotaImpl(self.cfg)
        self.system=SystemImpl(self.cfg)
        self.gmail=GmailImpl()
        print "[Setup]: %s" % self._test_name
    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(CheckGmailAfterGOTA, self).tearDown()
        self.cfg = None
    def testCheckGmailAfterGOTA(self):
        print "[RunTest]: %s" % self.__str__()
        self.gmail.launch_gmail()