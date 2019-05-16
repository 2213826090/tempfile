# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 01/14/2015
@author: Yingjun Jin
'''
# from testlib.graphics.test_template.render_app_testbase import RenderAppTestBase
from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.basicrenderscript import BasicRenderScript
from testlib.util.config import TestConfig
from testlib.util.repo import Artifactory
from testlib.graphics.tools import ConfigHandle
from testlib.util.common import g_common_obj
# import os


class RenderScript(UIATestBase):

    @classmethod
    def setUpClass(self):
        """
        install apk
        """
        super(RenderScript, self).setUpClass()
        config = TestConfig()
        cfg_file = 'tests.tablet.artifactory.conf'
        cfg_arti = config.read(cfg_file, 'artifactory')
        config_handle = ConfigHandle()
        cfg_arti["location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat/', 'sys.conf')
        cfg = config.read(cfg_file, 'content_basicrenderscript')
        arti = Artifactory(cfg_arti.get('location'))
        apk_name = cfg.get("name")
        file_path = arti.get(apk_name)
        result = config_handle.check_apps("com.intel.sample.androidbasicrs")
        if result == 0:
            g_common_obj.adb_cmd_common('install ' + file_path)

    def setUp(self):
        super(RenderScript, self).setUp()
        self._test_name = __name__
        cfg_file = 'tests.tablet.basic_render_script.conf'
        print "[Setup]: %s" % self._test_name
        self._runRenderScript = BasicRenderScript(\
            self.config.read(cfg_file, "render"))

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(RenderScript, self).tearDown()

    def testBasicRenderScript(self):
        self._runRenderScript.launch_app_am()
        self._runRenderScript.press_render_screen_center()
        self._runRenderScript.change_render_via_take_photo()
        self._runRenderScript.press_render_screen_center()
        self._runRenderScript.stop_app_am()