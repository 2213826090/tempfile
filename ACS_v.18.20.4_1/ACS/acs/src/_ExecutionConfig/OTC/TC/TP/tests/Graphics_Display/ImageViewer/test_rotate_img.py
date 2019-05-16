'''
@summary: run GLBenchmark
'''
import os

from testlib.graphics.gits_impl import GitsImpl
from testlib.util.common import g_common_obj
from testlib.util.config import TestConfig
from testlib.util.repo import Artifactory
from testlib.graphics.tools import ConfigHandle
from testlib.util.uiatestbase import UIATestBase


class GitsTest(UIATestBase):

    @classmethod
    def setUpClass(self):
        """
        install apk
        """
        super(GitsTest, self).setUpClass()
        config = TestConfig()
        cfg_file = 'tests.tablet.artifactory.conf'
        GitsImpl.setup_enviroment(config.read(cfg_file, "content_gits"))
        case_cfg = 'tests.tablet.imageview.test_rotate_img.conf'
        self.gits = GitsImpl(config.read(case_cfg, "test_rotate"))
        self.gits.remove_file()
        self.gits.setup_stream()

    @classmethod
    def tearDownClass(self):
        """
        uninstall apk
        """
        super(GitsTest, self).tearDownClass()
        self.gits.remove_file()
        self.gits = None

    def setUp(self):
        super(GitsTest, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(GitsTest, self).tearDown()

    def testRotaeImg(self):
        '''
        @summary: test_rotate_img
        '''
        print "[RunTest]: %s" % self.__str__()
        self.gits.replay_stream(self.gits.cfg.get("count"))
        assert self.gits.diff(self.gits.basic_screenshot, os.path.join(
            os.path.expanduser("~"),
            self.gits.cfg.get("screenshot_folder")))
        self.gits.remove_screenshot_in_logdir()
