# -*- coding: utf-8 -*-
"""
Created on 2014-11-5

@author: yusux
"""
from testlib.util.uiatestbase import UIATestBase
from testlib.util.config import TestConfig
from testlib.graphics.html5_impl import Html5Impl
from testlib.graphics.extend_chrome_impl import ChromeExtendImpl


class OpenWebpageWithGif(UIATestBase):

    def setUp(self):
        super(OpenWebpageWithGif, self).setUp()
        config = TestConfig()
        cfg_file = 'tests.tablet.artifactory.conf'
        cfg = config.read(cfg_file, 'content_chrome')
        web_gif = cfg.get("web_gif")
        self.web_gif = web_gif
        web_gif_check = cfg.get("webgif_check")
        self.web_gif_check = web_gif_check
        self._Html5Impl = Html5Impl()
        self._Html5Impl.check_chrome_installed()
        self.extendchrome = ChromeExtendImpl()
        self.extendchrome.clear_data()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name

    def tearDown(self):
        super(OpenWebpageWithGif, self).tearDown()
        print "[Teardown]: %s" % self._test_name
        self.extendchrome.clear_data()

    def testLaunchBrowseWebPageWithGif(self):
        """
        check if the web page with gif anmiation is open    
        """
        print "[RunTest]: %s" % self.__str__()
        self.extendchrome.launch()
        self.extendchrome.chrome_setup()
        self.extendchrome.open_website(self.web_gif)
        self.extendchrome.web_check(self.web_gif_check, 160)
