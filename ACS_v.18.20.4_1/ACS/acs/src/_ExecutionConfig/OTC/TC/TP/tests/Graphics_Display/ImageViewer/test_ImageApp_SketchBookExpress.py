# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 05/26/2015
@author: Xiangyi Zhao
'''
from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.ImageApp_SketchBookExpress_impl import SketchBookX
from testlib.util.config import TestConfig
from testlib.util.repo import Artifactory
from testlib.graphics.tools import ConfigHandle
from testlib.util.common import g_common_obj
import os

class SketchBookExpress(UIATestBase):

    @classmethod
    def setUpClass(self):
        """
        install apk
        """
        super(SketchBookExpress, self).setUpClass()
        config = TestConfig()
        cfg_file = 'tests.tablet.artifactory.conf'
        cfg_arti = config.read(cfg_file, 'artifactory')
        config_handle = ConfigHandle()
        cfg_arti["location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat/', 'sys.conf')
        cfg_apk = config.read(cfg_file, 'content_sketchbook')
        arti = Artifactory(cfg_arti.get('location'))
        binary_name = cfg_apk.get("name")
        file_path = arti.get(binary_name)
        result = config_handle.check_apps("com.adsk.sketchbookhdexpress")
        if result == 0:
            g_common_obj.adb_cmd_common('install ' + file_path)
        SketchBookX.set_workaround()

    def setUp(self):
        super(SketchBookExpress, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self._sketchBook = SketchBookX()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(SketchBookExpress, self).tearDown()
        os.system('rm sketchbook*.png')
        self._sketchBook.uninstall_app()

    def test_sketchbookexpress(self):
        self._sketchBook.launch_app_am()
        self._sketchBook.choose_menu()
        self._sketchBook.choose_quicktour()
        self._sketchBook.scroll()
        self._sketchBook.stop_app_am()