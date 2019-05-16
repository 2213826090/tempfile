# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 06/09/2015
@author: Xiangyi Zhao
'''
from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.ImageApp_PicSay_impl import PicSay
from testlib.util.config import TestConfig
from testlib.util.repo import Artifactory
from testlib.graphics.tools import ConfigHandle
from testlib.util.common import g_common_obj
import os

class Picsay(UIATestBase):

    @classmethod
    def setUpClass(self):
        """
        install apk
        """
        super(Picsay, self).setUpClass()
        config = TestConfig()
        cfg_file = 'tests.tablet.artifactory.conf'
        cfg_arti = config.read(cfg_file, 'artifactory')
        config_handle = ConfigHandle()
        cfg_arti["location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat/', 'sys.conf')
        cfg_apk = config.read(cfg_file, 'content_picsay')
        arti = Artifactory(cfg_arti.get('location'))
        binary_name = cfg_apk.get("name")
        file_path = arti.get(binary_name)
        result = config_handle.check_apps("com.shinycore.picsayfree")
        if result == 0:
            g_common_obj.adb_cmd_common('install ' + file_path)
        cfg_pic = config.read(cfg_file, 'content_picture')
        arti = Artifactory(cfg_arti.get('location'))
        pic_name = cfg_pic.get("name")
        file_path = arti.get(pic_name)
        g_common_obj.adb_cmd_common('push ' + file_path + ' /sdcard/Pictures')
        PicSay.set_workaround()

    def setUp(self):
        super(Picsay, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self._picsay = PicSay()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(Picsay, self).tearDown()
        g_common_obj.adb_cmd('rm -rf /sdcard/Pictures/*')
        os.system('rm *.png')
        self._picsay.uninstall_app()

    def test_picsay(self):
        self._picsay.launch_app_am()
        self._picsay.choose_pic()
        self._picsay.edit_pic()
        self._picsay.check_pic()
        self._picsay.stop_app_am()