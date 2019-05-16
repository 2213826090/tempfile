# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 5/23/2016
@author: Jin ChengjiangX
'''

import os
from testlib.util.repo import Artifactory
from testlib.graphics.tools import ConfigHandle
from testlib.util.common import g_common_obj
from testlib.util.uiatestbase import UIATestBase
from testlib.util.config import TestConfig
from testlib.graphics.gits_impl import GitsImpl


class DeleteOneImage(UIATestBase):

    @classmethod
    def setUpClass(self):
        """
        install apk
        """
        super(DeleteOneImage, self).setUpClass()
        self.config = TestConfig()
        cfg_file = 'tests.tablet.artifactory.conf'
        self.case_cfg = 'tests.tablet.gits.conf'
        GitsImpl.setup_enviroment(self.config.read(cfg_file, "content_gits"))

    def setUp(self):
        super(DeleteOneImage, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.gits = GitsImpl()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(DeleteOneImage, self).tearDown()
        self.gits.remove_file()

    def test_imagedelete_press_menu_select_photo_delete(self):
        """
        refer TC test_ImageEdit_AddSaturationEffect
        """
        print "[RunTest]: %s" % self.__str__()
        self.gits.handle_test(self.case_cfg, 'test_imagedelete_press_menu_select_photo_delete')
