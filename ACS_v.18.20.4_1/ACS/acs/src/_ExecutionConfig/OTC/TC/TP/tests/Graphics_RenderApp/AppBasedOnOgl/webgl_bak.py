# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 1/29/2016
@author: Zhao Xiangyi
'''

import os
from testlib.graphics.test_template.render_app_testbase import RenderAppTestBase
from testlib.util.config import TestConfig
from testlib.graphics.gits_impl import GitsImpl


class Webgl(RenderAppTestBase):

    @classmethod
    def setUpClass(self):
        """
        install apk
        """
        super(Webgl, self).setUpClass()
        self.config = TestConfig()
        cfg_file = 'tests.tablet.artifactory.conf'
        self.case_cfg = 'tests.tablet.webgl.conf'
        GitsImpl.setup_enviroment(self.config.read(cfg_file, "content_gits"))

    def setUp(self):
        super(Webgl, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.gits = GitsImpl()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(Webgl, self).tearDown()
        self.gits.remove_file()

    def test_WebGL_Performance_ToonShading(self):
        """
        refer TC test_WebGL_Performance_ToonShading
        """
        print "[RunTest]: %s" % self.__str__()
        self.gits.handle_test(self.case_cfg, 'webgl_toon_shading')

    def test_webgl_performance_imagesphere(self):
        """
        refer TC test_webgl_performance_imagesphere
        """
        print "[RunTest]: %s" % self.__str__()
        self.gits.handle_test(self.case_cfg, 'webgl_imagesphere')

    def test_WebGL_Performance_MultipleViews(self):
        """
        refer TC test_WebGL_Performance_MultipleViews
        """
        print "[RunTest]: %s" % self.__str__()
        self.gits.handle_test(self.case_cfg, 'webgl_multiple_views')

    def test_WebGL_Performance_Matrix(self):
        """
        refer TC test_WebGL_Performance_Matrix
        """
        print "[RunTest]: %s" % self.__str__()
        self.gits.handle_test(self.case_cfg, 'webgl_performance_matrix')

    def test_webgl_Iceberg(self):
        """
        refer TC test_webgl_Iceberg
        """
        print "[RunTest]: %s" % self.__str__()
        self.gits.handle_test(self.case_cfg, 'webgl_iceberg')

    def test_webgl_performance_blob(self):
        """
        refer TC test_webgl_performance_blob
        """
        print "[RunTest]: %s" % self.__str__()
        self.gits.handle_test(self.case_cfg, 'webgl_blob')

    def test_WebGL_Performance_Spacerocks(self):
        """
        refer TC test_WebGL_Performance_Spacerocks
        """
        print "[RunTest]: %s" % self.__str__()
        self.gits.handle_test(self.case_cfg, 'webgl_performance_spacerocks')

    def test_webgl_performance_fishtank(self):
        """
        refer TC test_webgl_performance_fishtank
        """
        print "[RunTest]: %s" % self.__str__()
        self.gits.handle_test(self.case_cfg, 'webgl_fishtank')

    def test_webgl_performance_aquarium(self):
        """
        refer TC test_webgl_performance_aquarium
        """
        print "[RunTest]: %s" % self.__str__()
        self.gits.handle_test(self.case_cfg, 'webgl_aquarium')

    def test_webgl_performance_field(self):
        """
        refer TC test_webgl_performance_field
        """
        print "[RunTest]: %s" % self.__str__()
        self.gits.handle_test(self.case_cfg, 'webgl_field')

    def test_webgl_performance_dynamiccubemap(self):
        """
        refer TC test_webgl_performance_dynamiccubemap
        """
        print "[RunTest]: %s" % self.__str__()
        self.gits.handle_test(self.case_cfg, 'webgl_dynamic_cubemap')

    def test_WebGL_html5code_gallery(self):
        """
        refer TC test_WebGL_html5code_gallery
        """
        print "[RunTest]: %s" % self.__str__()
        self.gits.handle_test(self.case_cfg, 'webgl_html5code_gallery')

