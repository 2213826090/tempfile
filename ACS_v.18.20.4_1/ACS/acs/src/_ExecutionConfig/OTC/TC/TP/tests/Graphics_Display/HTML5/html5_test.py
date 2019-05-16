# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 03/12/2015
@author: Yingjun Jin
'''

from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.html5_impl import Html5Impl


class Html5(UIATestBase):

    @classmethod
    def setUpClass(self):
        super(Html5, self).setUpClass()
        self.case_cfg = 'tests.html5_and_webgl.conf'

    def setUp(self):
        super(Html5, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self._html5 = Html5Impl()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(Html5, self).tearDown()

    def test_html5_css3_selectors(self):
        ''' refer TC test_HTML5_CSS3_Selectors
        '''
        print "[RunTest]: %s" % self.__str__()
        self._html5.run_case(self.case_cfg, 'html5_CSS3')

    def test_html5performance_html5test(self):
        ''' refer TC test_HTML5Performance_html5test
        '''
        print "[RunTest]: %s" % self.__str__()
        self._html5.run_case(self.case_cfg, 'html5_test')

    def test_html5performance_galactic(self):
        ''' refer TC test_HTML5Performance_Galactic
        '''
        print "[RunTest]: %s" % self.__str__()
        self._html5.run_case(self.case_cfg, 'html5_Galactic')

    def test_html5performance_canvaspinball(self):
        ''' refer TC test_HTML5Performance_CanvasPinball
        '''
        print "[RunTest]: %s" % self.__str__()
        self._html5.run_case(self.case_cfg, 'html5_CanvasPinball')

    def test_html5performance_flickrexplorer(self):
        ''' refer TC test_HTML5Performance_FlickrExplorer
        '''
        print "[RunTest]: %s" % self.__str__()
        self._html5.run_case(self.case_cfg, 'html5_FlickrExplorer')

    def test_html5demos_canvas(self):
        ''' refer TC test_HTML5Demos_Canvas
        '''
        print "[RunTest]: %s" % self.__str__()
        self._html5.run_case(self.case_cfg, 'html5_Canvas')

    def test_html5canvassupport_html5test(self):
        ''' refer TC test_Html5CanvasSupport_Html5test
        '''
        print "[RunTest]: %s" % self.__str__()
        self._html5.run_case(self.case_cfg, 'html5_CanvasSupport')

    def test_asteroidshtml5canvas_2drendering_javascriptbenchmark(self):
        ''' refer TC test_AsteroidsHTML5Canvas_2DRendering_JavaScriptBenchmark
        '''
        print "[RunTest]: %s" % self.__str__()
        self._html5.run_case_benchmark(self.case_cfg, 'html5_JavaScriptBenchmark')

    def test_html5performance_fishbowl(self):
        ''' refer TC test_html5performance_FishBowl
        '''
        print "[RunTest]: %s" % self.__str__()
        self._html5.run_case(self.case_cfg, 'html5_FishBowl')

    def test_html5performance_speedreading(self):
        ''' refer TC test_HTML5Performance_SpeedReading
        '''
        print "[RunTest]: %s" % self.__str__()
        self._html5.run_case(self.case_cfg, 'html5_SpeedReading')

    def test_html5_html5rocks(self):
        ''' refer TC test_HTML5_html5rocks
        '''
        print "[RunTest]: %s" % self.__str__()
        self._html5.run_case(self.case_cfg, 'html5_rocks')

    def test_html5performance_flyingimages(self):
        ''' refer TC test_HTML5Performance_FlyingImages
        '''
        print "[RunTest]: %s" % self.__str__()
        self._html5.run_case(self.case_cfg, 'html5_flyingimages')

    def test_html5elements_html5test(self):
        ''' refer TC test_html5elements_html5test
        '''
        print "[RunTest]: %s" % self.__str__()
        self._html5.run_case(self.case_cfg, 'html5_elements')
