# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 12/24/2015
@author: Xiangyi Zhao
'''
import os
from testlib.graphics.glbenckmark_impl import GLBenchmarkImpl
from testlib.graphics.extend_glbenckmark_impl import GLBenchmarkExtendImpl
from testlib.util.repo import Artifactory
from testlib.graphics.tools import ConfigHandle
from testlib.util.common import g_common_obj
from testlib.util.uiatestbase import UIATestBase
from testlib.util.config import TestConfig


class BurstFrequency(UIATestBase):

    @classmethod
    def setUpClass(self):
        """
        install apk
        """
        super(BurstFrequency, self).setUpClass()
        g_common_obj.root_on_device()
        g_common_obj.remount_device()
        config = TestConfig()
        cfg_file = 'tests.tablet.artifactory.conf'
        cfg_arti = config.read(cfg_file, 'artifactory')
        config_handle = ConfigHandle()
        cfg_arti["location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat/', 'sys.conf')
        cfg = config.read(cfg_file, 'content_graphic')
        arti = Artifactory(cfg_arti.get('location'))
        # apk_name = cfg.get("name")
        apk_name = cfg.get("name_270")
        file_path = arti.get(apk_name)
        # g_common_obj.adb_cmd_common('install ' + file_path, 600)
        os.system("adb install -r -g %s" % file_path)

    @classmethod
    def tearDownClass(self):
        """
        uninstall apk
        """
        super(BurstFrequency, self).tearDownClass()

    def setUp(self):
        super(BurstFrequency, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self._glBenchmark = GLBenchmarkImpl()
        self.benchmark = GLBenchmarkExtendImpl()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(BurstFrequency, self).tearDown()
        self._glBenchmark.stop_app_am()
        self._glBenchmark = None
        self.benchmark.clean()

    def test_Burst_Frequency(self):
        """
        This test case is to Run GLBenchmark 2.5.1 Egypt HD ETC1 OnScreen test

        Test Case Precondition:
        GLBenchmark 2.5.1 apk installed on DUT

        Test Case Step:
        1. Launch GLBenchmark
        2. Open Performance Tests, select GLBenchmark 2.7 EgyptHD ETC1
        3. Start run with Onscreen

        Expect Result:
        1. GLBenchmark can run successfully and fluently

        The real implementation will be in GLBenchmarkImpl class.
        """

        print "[RunTest]: %s" % self.__str__()
        self.benchmark.launch()
        self.benchmark.run_performance_check_burstfrequency("test23", timeout=180)
        g_common_obj.assert_exp_happens()
