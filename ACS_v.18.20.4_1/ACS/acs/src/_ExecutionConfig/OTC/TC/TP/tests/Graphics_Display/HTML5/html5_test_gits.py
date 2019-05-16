# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 01/05/2016
@author: Xiangyi Zhao
'''

from testlib.util.uiatestbase import UIATestBase
from testlib.util.config import TestConfig
from testlib.graphics.gits_impl import GitsImpl


class Html5(UIATestBase):

    @classmethod
    def setUpClass(self):
        """
        setup enviroment
        """
        super(Html5, self).setUpClass()
        self.config = TestConfig()
        cfg_file = 'tests.tablet.artifactory.conf'
        self.case_cfg = 'tests.tablet.html5.conf'
        GitsImpl.setup_enviroment(self.config.read(cfg_file, "content_gits"))

    def setUp(self):
        super(Html5, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.gits = GitsImpl()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(Html5, self).tearDown()
        self.gits.remove_file()

    def test_html5demos_canvas(self):
        """
        refer TC test_html5demos_canvas
        """
        print "[RunTest]: %s" % self.__str__()
        self.gits.handle_test(self.case_cfg, 'demos_canvas')

    def test_html5performance_html5test(self):
        """
        refer TC test_html5performance_html5test
        """
        print "[RunTest]: %s" % self.__str__()
        self.gits.handle_test(self.case_cfg, 'performance_html5test')

    def test_html5_css3_selectors(self):
        """
        refer TC test_html5_css3_selectors
        """
        print "[RunTest]: %s" % self.__str__()
        self.gits.handle_test(self.case_cfg, 'css3_selectors')

    def test_html5canvassupport_html5test(self):
        """
        refer TC test_html5canvassupport_html5test
        """
        print "[RunTest]: %s" % self.__str__()
        self.gits.handle_test(self.case_cfg, 'canvassupport_html5test')

    def test_html5performance_flyingimages(self):
        """
        refer TC test_html5performance_flyingimages
        """
        print "[RunTest]: %s" % self.__str__()
        self.gits.handle_test(self.case_cfg, 'performance_flyingimages')

    def test_html5elements_html5test(self):
        """
        refer TC test_html5elements_html5test
        """
        print "[RunTest]: %s" % self.__str__()
        self.gits.handle_test(self.case_cfg, 'elements_html5test')

    def test_asteroidshtml5canvas_2drendering_javascriptbenchmark(self):
        """
        refer TC test_asteroidshtml5canvas_2drendering_javascriptbenchmark
        """
        print "[RunTest]: %s" % self.__str__()
        self.gits.handle_test(self.case_cfg, 'javascriptbenchmark')

