# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 01/14/2015
@author: Yingjun Jin
'''

from testlib.graphics.test_template.render_app_testbase import RenderAppTestBase
from testlib.graphics.rsballs import RsBalls
from testlib.util.config import TestConfig
from testlib.util.repo import Artifactory
from testlib.graphics.tools import ConfigHandle
from testlib.util.common import g_common_obj

class RunRsBalls(RenderAppTestBase):

    @classmethod
    def setUpClass(self):
        """
        install apk
        """
        super(RunRsBalls, self).setUpClass()
        config = TestConfig()
        cfg_file = 'tests.tablet.artifactory.conf'
        cfg_arti = config.read(cfg_file, 'artifactory')
        config_handle = ConfigHandle()
        cfg_arti["location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat/', 'sys.conf')
        cfg = config.read(cfg_file, 'content_rsballs')
        arti = Artifactory(cfg_arti.get('location'))
        apk_name = cfg.get("name")
        file_path = arti.get(apk_name)
        result = config_handle.check_apps("com.example.android.rs.balls")
        if result == 0:
            g_common_obj.adb_cmd_common('install ' + file_path)

    def setUp(self):
        super(RunRsBalls, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self._runRsBalls = RsBalls()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(RunRsBalls, self).tearDown()

    def testRsBalls(self):
        self._runRsBalls.launch_app_am()
        self._runRsBalls.rotate_screen()
        self._runRsBalls.stop_app_am()