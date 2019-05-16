# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 06/2/2015
@author: Zhang RongX,Z
'''
from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.ImageApp_MeituPic_impl import MeituPicEdit
from testlib.util.config import TestConfig
from testlib.util.repo import Artifactory
from testlib.graphics.tools import ConfigHandle
from testlib.util.common import g_common_obj

class MeituPic(UIATestBase):

    @classmethod
    def setUpClass(self):
        """
        install apk
        """
        super(MeituPic, self).setUpClass()
        config = TestConfig()
        cfg_file = 'tests.tablet.artifactory.conf'
        cfg_arti = config.read(cfg_file, 'artifactory')
        config_handle = ConfigHandle()
        cfg_arti["location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat/', 'sys.conf')
        cfg_apk = config.read(cfg_file, 'content_meitupic')
        arti = Artifactory(cfg_arti.get('location'))
        binary_name = cfg_apk.get("name")
        file_path = arti.get(binary_name)
        result = config_handle.check_apps("com.mt.mtxx.mtxx")
        if result == 0:
            g_common_obj.adb_cmd_common('install ' + file_path)
        cfg_pic = config.read(cfg_file, 'content_picture')
        arti = Artifactory(cfg_arti.get('location'))
        pic_name = cfg_pic.get("name")
        file_path = arti.get(pic_name)
        g_common_obj.adb_cmd('rm /sdcard/Pictures/*')
        g_common_obj.adb_cmd_common('push ' + file_path + ' /sdcard/Pictures')

    def setUp(self):
        print "setup"
        super(MeituPic, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self._meitupic_edit = MeituPicEdit()
        self._meitupic_edit.fresh_sdcard()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(MeituPic, self).tearDown()
        g_common_obj.adb_cmd('rm /sdcard/Pictures/*')
        g_common_obj.adb_cmd('rm /sdcard/DCIM/Camera/*')
        self._meitupic_edit.uninstall_app()

    def test_meitupic(self):
        print "edit a picture using  apk metupic"
        self._meitupic_edit.launch_app_am()
        self._meitupic_edit.choose_pic()
        self._meitupic_edit.edit_pic()
        self._meitupic_edit.save_pic()
        g_common_obj.assert_exp_happens()
        self._meitupic_edit.stop_app_am()