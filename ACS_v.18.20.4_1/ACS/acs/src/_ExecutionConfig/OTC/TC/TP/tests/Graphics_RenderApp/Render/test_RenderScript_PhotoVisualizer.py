# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 03/26/2015
@author: Yingjun Jin
'''
from testlib.graphics.test_template.render_app_testbase import RenderAppTestBase
from testlib.graphics.render_impl import RenderImpl


class Render(RenderAppTestBase):

    def setUp(self):
        super(Render, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self._render = RenderImpl()
        self._render.install_apk("Photo_visualizer")
        self._render.init_photos()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(Render, self).tearDown()
        self._render.delete_photos()

    def test_renderscript_photovisualizer(self):
        ''' refer TC test_RenderScript_PhotoVisualizer
        '''
        print "[RunTest]: %s" % self.__str__()
        self._render.launch_photovisualizer_am()
        self._render.set_photovisualizer_settings_1st()
        self._render.run_photovisualizer()
        self._render.set_photovisualizer_settings_2st()
        self._render.run_photovisualizer()
        self._render.stop_photovisualizer_am()
