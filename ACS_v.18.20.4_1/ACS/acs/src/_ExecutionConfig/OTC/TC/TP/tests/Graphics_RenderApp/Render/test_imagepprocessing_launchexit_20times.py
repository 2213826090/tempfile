# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 01/15/2015
@author: Yingjun Jin
'''

from testlib.graphics.test_template.render_app_testbase import RenderAppTestBase
from testlib.graphics.imageprocessing import ImageProcessingTest
from testlib.util.config import TestConfig
from testlib.util.repo import Artifactory
from testlib.graphics.tools import ConfigHandle
from testlib.util.common import g_common_obj
from testlib.graphics.common import osversion

class RunImageProcessing(RenderAppTestBase):

    @classmethod
    def setUpClass(self):
        """
        install apk
        """
        super(RunImageProcessing, self).setUpClass()
        config = TestConfig()
        cfg_file = 'tests.tablet.artifactory.conf'
        cfg_arti = config.read(cfg_file, 'artifactory')
        config_handle = ConfigHandle()
        cfg_arti["location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat/', 'sys.conf')
        cfg = config.read(cfg_file, 'content_imageprocessing')
        arti = Artifactory(cfg_arti.get('location'))
        version_array = osversion.get_android_version()
        androidversion = version_array[0]
        if androidversion == 7:
            print "osversion is N"
            apk_name = cfg.get("name")
        elif androidversion == 6:
            print "osversion is M"
            apk_name = cfg.get("name")
        elif androidversion == 5:
            print "osversion is L"
            apk_name = cfg.get("name_l")
        else:
            print "osversion is %s" % (androidversion)
            apk_name = cfg.get("name_l")
        file_path = arti.get(apk_name)
        result = config_handle.check_apps("com.android.rs.image")
        if result == 0:
            g_common_obj.adb_cmd_common('install ' + file_path)

    def setUp(self):
        super(RunImageProcessing, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self._runimageprocessing = ImageProcessingTest()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(RunImageProcessing, self).tearDown()
        self._runimageprocessing.uninstall_app()

    def imageprocessing_launchexit_20times(self):
        self._runimageprocessing.launch_app_am()
        self._runimageprocessing.launchexit_20times()
        self._runimageprocessing.stop_app_am()