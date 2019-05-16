# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 09/25/2015
@author: Xiangyi Zhao
'''

from testlib.graphics.test_template.render_app_testbase import RenderAppTestBase
from testlib.util.config import TestConfig
from testlib.graphics.gits_impl import GitsImpl


class AppBasedOnOgl(RenderAppTestBase):

    @classmethod
    def setUpClass(self):
        """
        install apk
        """
        super(AppBasedOnOgl, self).setUpClass()
        self.config = TestConfig()
        cfg_file = 'tests.tablet.artifactory.conf'
        self.case_cfg = 'tests.tablet.gits.conf'
        GitsImpl.setup_enviroment(self.config.read(cfg_file, "content_gits"))

    def setUp(self):
        super(AppBasedOnOgl, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.gits = GitsImpl()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(AppBasedOnOgl, self).tearDown()
        self.gits.remove_file()

    def test_3DGame_FruitNinja(self):
        """
        refer TC test_3DGame_FruitNinja
        """
        print "[RunTest]: %s" % self.__str__()
        self.gits.handle_test(self.case_cfg, 'FruitNinja')