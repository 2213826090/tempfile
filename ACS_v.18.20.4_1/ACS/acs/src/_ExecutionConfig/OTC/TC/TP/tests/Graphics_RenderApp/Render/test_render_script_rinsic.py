# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 01/14/2015
@author: Yingjun Jin
'''

from testlib.graphics.test_template.render_app_testbase import RenderAppTestBase
from testlib.graphics.renderscriptrinsic import RenderScriptIntrinsic
from testlib.util.config import TestConfig
from testlib.util.repo import Artifactory
from testlib.graphics.tools import ConfigHandle
from testlib.util.common import g_common_obj

class RenderScriptRinsic(RenderAppTestBase):

    @classmethod
    def setUpClass(self):
        """
        install apk
        """
        super(RenderScriptRinsic, self).setUpClass()
        config = TestConfig()
        cfg_file = 'tests.tablet.artifactory.conf'
        cfg_arti = config.read(cfg_file, 'artifactory')
        config_handle = ConfigHandle()
        cfg_arti["location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat/', 'sys.conf')
        cfg = config.read(cfg_file, 'content_renderscriptrinsic')
        arti = Artifactory(cfg_arti.get('location'))
        apk_name = cfg.get("name")
        file_path = arti.get(apk_name)
        result = config_handle.check_apps("com.example.android.renderscriptintrinsic")
        if result == 0:
            g_common_obj.adb_cmd_common('install ' + file_path)

    def setUp(self):
        super(RenderScriptRinsic, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self._runScriptRinsic = RenderScriptIntrinsic()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(RenderScriptRinsic, self).tearDown()

    def testRenderScriptRinsic(self):
        self._runScriptRinsic.launch_app_am()
        self._runScriptRinsic.swipe_the_slider()
        self._runScriptRinsic.select_emboss()
        self._runScriptRinsic.swipe_the_slider()
        self._runScriptRinsic.select_hue()
        self._runScriptRinsic.swipe_the_slider()
        self._runScriptRinsic.stop_app_am()

