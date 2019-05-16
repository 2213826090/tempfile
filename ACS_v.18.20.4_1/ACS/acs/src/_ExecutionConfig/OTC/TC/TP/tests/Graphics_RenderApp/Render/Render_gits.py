# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 04/17/2015
@author: Yingjun Jin
'''

import os
from testlib.util.repo import Artifactory
from testlib.graphics.tools import ConfigHandle
from testlib.util.common import g_common_obj
from testlib.graphics.test_template.render_app_testbase import RenderAppTestBase
from testlib.util.config import TestConfig
from testlib.graphics.gits_impl import GitsImpl


class Render(RenderAppTestBase):

    @classmethod
    def setUpClass(self):
        """
        init enviroment
        """
        super(Render, self).setUpClass()
        self.config = TestConfig()
        cfg_file = 'tests.tablet.artifactory.conf'
        self.case_cfg = 'tests.tablet.gits.conf'
        GitsImpl.setup_enviroment(self.config.read(cfg_file, "content_gits"))

    def setUp(self):
        super(Render, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.gits = GitsImpl()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(Render, self).tearDown()
        self.gits.remove_file()

    def test_imageprocessing_anyvalues(self):
        """
        refer TC test_ImageProcessing_AnyValues
        """
        print "[RunTest]: %s" % self.__str__()
        self.gits.handle_test(self.case_cfg, 'AnyValues')

    def test_basicrenderscript(self):
        """
        refer TC test_BasicRenderScript
        """
        print "[RunTest]: %s" % self.__str__()
        self.gits.handle_test(self.case_cfg, 'BRS')
