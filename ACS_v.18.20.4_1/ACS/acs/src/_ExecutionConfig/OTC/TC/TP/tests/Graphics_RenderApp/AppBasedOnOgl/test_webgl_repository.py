# -*- coding: utf-8 -*-
"""
Created on 2014-11-5

@author: yusux
"""
from testlib.browser.browser_impl import BrowserImpl
from testlib.graphics.test_template.render_app_testbase import RenderAppTestBase


class WebGLrepotest(RenderAppTestBase):

    def setUp(self):
        super(WebGLrepotest, self).setUp()
        cfg_file = 'tests.tablet.browser.conf'
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.browser = BrowserImpl(self.config.read(cfg_file, 'webglrepo'))
        self.browser.browser_setup()

    def tearDown(self):
        super(WebGLrepotest, self).tearDown()
        print "[Teardown]: %s" % self._test_name
        self.browser.clear_data()
        self.browser = None

    def test_webgl_demorepo_san_angeles(self):
        """
        check if the webgl page with san_angeles is browserable
        """
        print "[RunTest]: %s" % self.__str__()
        self.browser.open_website(self.browser.cfg.get("webglrepotest_san_angeles"))
        self.browser.web_check("web_gif_check", 160)

    def test_webgl_demorepo_particles(self):
        """
        check if the webgl page with particles is browserable
        """
        print "[RunTest]: %s" % self.__str__()
        self.browser.open_website(self.browser.cfg.get("webglrepotest_particles"))
        self.browser.web_check("web_gif_check", 160)

    def test_webgl_demorepo_shiny_teapot(self):
        """
        check if the webgl page with shiny_teapot is browserable
        """
        print "[RunTest]: %s" % self.__str__()
        self.browser.open_website(self.browser.cfg.get("webglrepotest_shiny_teapot"))
        self.browser.web_check("web_gif_check", 160)

    def test_webgl_demorepo_high_dpi(self):
        """
        check if the webgl page with high_dpi is browserable
        """
        print "[RunTest]: %s" % self.__str__()
        self.browser.open_website(self.browser.cfg.get("webglrepotest_high_dpi"))
        self.browser.web_check("web_gif_check", 160)

    def test_webgl_demorepo_image_texture_test(self):
        """
        check if the webgl page with image_texture_test is browserable
        """
        print "[RunTest]: %s" % self.__str__()
        self.browser.open_website(self.browser.cfg.get("webglrepotest_image_texture_test"))
        self.browser.web_check("web_gif_check", 160)

    def test_webgl_demorepo_procedural_texture_test(self):
        """
        check if the webgl page with procedural_texture_test is browserable
        """
        print "[RunTest]: %s" % self.__str__()
        self.browser.open_website(self.browser.cfg.get("webglrepotest_procedural_texture_test"))
        self.browser.web_check("web_gif_check", 160)

