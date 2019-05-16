# -*- coding: utf-8 -*-
'''
@summary: Install basemark app and run Taiji subtest
@since: 09/18/2017
@author: Rui
'''
from testlib.graphics.glbenckmark_impl import GLBenchmarkImpl
from testlib.graphics.extend_glbenckmark_impl import GLBenchmarkExtendImpl
from testlib.util.repo import Artifactory
from testlib.graphics.tools import ConfigHandle
from testlib.util.common import g_common_obj
from testlib.util.config import TestConfig
from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.common import close_all_tasks


class RunBasemark(UIATestBase):
    '''
    Replaced with benchmark app since houdini not supported anymore.
    '''

    @classmethod
    def setUpClass(self):
        """
        install apk
        """
        super(RunBasemark, self).setUpClass()
        config = TestConfig()
        cfg_file = 'tests.tablet.artifactory.conf'
        cfg_arti = config.read(cfg_file, 'artifactory')
        config_handle = ConfigHandle()
        cfg_arti["location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat/', 'sys.conf')
        cfg = config.read(cfg_file, 'content_graphic')
        arti = Artifactory(cfg_arti.get('location'))
        apk_name = cfg.get("name")
        file_path = arti.get(apk_name)
        result = config_handle.check_apps("com.glbenchmark.glbenchmark27")
        if result == 0:
            g_common_obj.adb_cmd_common('install -r ' + file_path, 300)

    @classmethod
    def tearDownClass(self):
        """
        uninstall apk
        """
        super(RunBasemark, self).tearDownClass()

    def setUp(self):
        super(RunBasemark, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self._glBenchmark = GLBenchmarkImpl()
        self.benchmark = GLBenchmarkExtendImpl()
        close_all_tasks()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(RunBasemark, self).tearDown()
        self._glBenchmark.stop_app_am()
        self._glBenchmark = None
        self.benchmark.clean()

    def test_CheckTearing_BasemarkES2v1_Taiji_Hover(self):
        print "[RunTest]: %s" % self.__str__()
        self.benchmark.launch()
        self.benchmark.run_performance_test("test25")
        self.benchmark.check_all_results()
        g_common_obj.assert_exp_happens()
