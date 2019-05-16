# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 09/08/2015
@author: Xiangyi Zhao
'''

from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.ImageApp_DocsToGo_impl import DocsToGo
from testlib.util.config import TestConfig
from testlib.util.repo import Artifactory
from testlib.graphics.tools import ConfigHandle
from testlib.util.common import g_common_obj
from testlib.common.base import clearTmpDir


class ComposeUI(UIATestBase):

    def setUp(self):
        super(ComposeUI, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self._docstogo = DocsToGo()

        config = TestConfig()
        cfg_file = 'tests.tablet.composeui.conf'
        cfg_arti = config.read(cfg_file, 'artifactory')
        config_handle = ConfigHandle()
        cfg_arti["location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat/', 'sys.conf')
        cfg_apk = config.read(cfg_file, 'DocsToGo')
        arti = Artifactory(cfg_arti.get('location'))
        binary_name = cfg_apk.get("name")
        file_path = arti.get(binary_name)
        result = config_handle.check_apps("com.dataviz.docstogo")
        if result == 0:
            g_common_obj.adb_cmd_common('install ' + file_path)

        arti = Artifactory(cfg_arti.get('location'))
        cfg_docx = config.read(cfg_file, 'docx')
        docx_name = cfg_docx.get("name")
        docx_path = arti.get(docx_name)
        g_common_obj.adb_cmd_common('push ' + docx_path + ' /sdcard/')
        cmd = "am broadcast -a android.intent.action.MEDIA_MOUNTED\
         -d file:///sdcard/"
        g_common_obj.adb_cmd(cmd)
        self.docx = cfg_docx.get("filename")
        print self.docx

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(ComposeUI, self).tearDown()
        self._docstogo.uninstall_app()
        self._docstogo.clean_workaround(self.docx)
        clearTmpDir()

    def test_TextEdit_DocsToGo(self):
        ''' refer TC test_TextEdit_DocsToGo
        '''
        print "[RunTest]: %s" % self.__str__()
        self._docstogo.launch_app_am()
        self._docstogo.edit_docx()
        self._docstogo.check_docx()
        self._docstogo.stop_app_am()
