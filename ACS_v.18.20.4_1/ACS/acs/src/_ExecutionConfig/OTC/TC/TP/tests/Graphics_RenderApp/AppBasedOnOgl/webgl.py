# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 03/16/2015
@author: Yingjun Jin
'''

from testlib.graphics.test_template.render_app_testbase import RenderAppTestBase
from testlib.graphics.html5_impl import Html5Impl

class Webgl(RenderAppTestBase):

    @classmethod
    def setUpClass(self):
        super(Webgl, self).setUpClass()
        self.case_cfg = 'tests.html5_and_webgl.conf'

    def setUp(self):
        super(Webgl, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self._webgl = Html5Impl()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(Webgl, self).tearDown()

    def test_webgl_demorepository(self):
        ''' refer TC test_WebGL_DemoRepository
        '''
        print "[RunTest]: %s" % self.__str__()
        self._webgl.run_case(self.case_cfg, 'webgl_DemoRepository')

    def test_webgl_performance_lotsofobjects(self):
        ''' refer TC test_WebGL_Performance_LotsOfObjects
        '''
        print "[RunTest]: %s" % self.__str__()
        self._webgl.run_case(self.case_cfg, 'webgl_LotsOfObjects')

    def test_webgl_playgame(self):
        ''' refer TC test_WebGL_PlayGame
        '''
        print "[RunTest]: %s" % self.__str__()
        self._webgl.run_case(self.case_cfg, 'webgl_PlayGame')

    def test_webgl_Iceberg(self):
        ''' refer TC test_WebGL_Iceberg
        '''
        print "[RunTest]: %s" % self.__str__()
        self._webgl.run_case(self.case_cfg, 'webgl_Iceberg')

    def test_webgl_performance_aquarium(self):
        ''' refer TC test_WebGL_Performance_Aquarium
        '''
        print "[RunTest]: %s" % self.__str__()
        self._webgl.run_case(self.case_cfg, 'webgl_aquarium')

    def test_webgl_performance_blob(self):
        ''' refer TC test_WebGL_Performance_Blob
        '''
        print "[RunTest]: %s" % self.__str__()
        self._webgl.run_case(self.case_cfg, 'webgl_Blob')

    def test_webgl_performance_dynamiccubemap(self):
        ''' refer TC test_WebGL_Performance_DynamicCubemap
        '''
        print "[RunTest]: %s" % self.__str__()
        self._webgl.run_case(self.case_cfg, 'webgl_dynamic_cubemap')

    def test_webgl_performance_field(self):
        ''' refer TC test_WebGL_Performance_Field
        '''
        print "[RunTest]: %s" % self.__str__()
        self._webgl.run_case(self.case_cfg, 'webgl_field')

    def test_webgl_performance_fishtank(self):
        ''' refer TC test_WebGL_Performance_FishTank
        '''
        print "[RunTest]: %s" % self.__str__()
        self._webgl.run_case(self.case_cfg, 'webgl_fishtank')

    def test_webgl_performance_imagesphere(self):
        ''' refer TC test_WebGL_Performance_Imagesphere
        '''
        print "[RunTest]: %s" % self.__str__()
        self._webgl.run_case(self.case_cfg, 'webgl_imagesphere')

    def test_WebGL_Performance_ToonShading(self):
        ''' refer TC test_WebGL_Performance_ToonShading
        '''
        print "[RunTest]: %s" % self.__str__()
        self._webgl.run_case(self.case_cfg, 'webgl_toon_shading')

    def test_WebGL_Performance_MultipleViews(self):
        ''' refer TC test_WebGL_Performance_MultipleViews
        '''
        print "[RunTest]: %s" % self.__str__()
        self._webgl.run_case(self.case_cfg, 'webgl_multiple_views')

    def test_WebGL_Performance_Matrix(self):
        ''' refer TC test_WebGL_Performance_Matrix
        '''
        print "[RunTest]: %s" % self.__str__()
        self._webgl.run_case(self.case_cfg, 'webgl_performance_matrix')

    def test_WebGL_Performance_Spacerocks(self):
        ''' refer TC test_WebGL_Performance_Spacerocks
        '''
        print "[RunTest]: %s" % self.__str__()
        self._webgl.run_case(self.case_cfg, 'webgl_performance_spacerocks')

    def test_WebGL_html5code_gallery(self):
        ''' refer TC test_WebGL_html5code_gallery
        '''
        print "[RunTest]: %s" % self.__str__()
        self._webgl.run_case(self.case_cfg, 'webgl_html5code_gallery')
