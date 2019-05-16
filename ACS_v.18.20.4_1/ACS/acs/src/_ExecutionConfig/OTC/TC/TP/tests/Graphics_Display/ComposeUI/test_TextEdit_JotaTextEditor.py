# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 08/31/2015
@author: Xiangyi Zhao
'''

from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.ImageApp_JotaTextEditor_impl import JotaTextEditor
from testlib.util.config import TestConfig
from testlib.util.repo import Artifactory
from testlib.graphics.tools import ConfigHandle
from testlib.util.common import g_common_obj


class ComposeUI(UIATestBase):

    def setUp(self):
        super(ComposeUI, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self._jotatexteditor = JotaTextEditor()
        config = TestConfig()
        cfg_file = 'tests.tablet.composeui.conf'
        cfg_arti = config.read(cfg_file, 'artifactory')
        config_handle = ConfigHandle()
        cfg_arti["location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat/', 'sys.conf')
        cfg_apk = config.read(cfg_file, 'JotaTextEditor')
        arti = Artifactory(cfg_arti.get('location'))
        binary_name = cfg_apk.get("name")
        file_path = arti.get(binary_name)
        result = config_handle.check_apps("jp.sblo.pandora.jota")
        if result == 0:
            g_common_obj.adb_cmd_common('install ' + file_path)
        self._jotatexteditor.clean_workaround()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(ComposeUI, self).tearDown()
        self._jotatexteditor.clean_workaround()
        self._jotatexteditor.uninstall_app()

    def test_TextEdit_JotaTextEditor(self):
        ''' refer TC test_TextEdit_JotaTextEditor
        '''
        print "[RunTest]: %s" % self.__str__()
        self._jotatexteditor.launch_app_am()
        self._jotatexteditor.edit_text()
        self._jotatexteditor.stop_app_am()
