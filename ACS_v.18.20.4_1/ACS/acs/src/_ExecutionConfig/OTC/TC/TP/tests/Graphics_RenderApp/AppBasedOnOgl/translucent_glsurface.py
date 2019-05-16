# -*- coding: utf-8 -*-
'''
Created on Dec 8, 2014

@author: yusux
'''
from testlib.graphics.sample_apidemo import SampleApiDemoImpl
from testlib.graphics.test_template.render_app_testbase import RenderAppTestBase
from testlib.util.common import g_common_obj

class TestGLSurface(RenderAppTestBase):
    @classmethod
    def setUpClass(cls):
        super(TestGLSurface, cls).setUpClass()
        _runPresentation = SampleApiDemoImpl()
        _runPresentation.install_apk()

    def setUp(self):
        super(TestGLSurface, self).setUp()
        self._test_name = __name__
        print "[Setup]:  %s" % self._test_name
        self._runPresentation = SampleApiDemoImpl()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(TestGLSurface, self).tearDown()
        g_common_obj.shell_cmd("ps -ef | grep "'adb'" | awk '{print $2}' | xargs killall")

    def testsTranslucent_GLSurface(self, timeout=1800):
        self._runPresentation.stop_app_am()
        self._runPresentation.countdown_t0_translucent_glsurfaceview(timeout)

    def test_countdown_t0_test_compresstexture(self, timeout=1800):
        self._runPresentation.stop_app_am()
        self._runPresentation.countdown_t0_test_compresstexture(timeout)

    def test_countdown_t0_test_cubemap(self, timeout=1800):
        self._runPresentation.stop_app_am()
        self._runPresentation.countdown_t0_test_cubemap(timeout)

    def test_countdown_t0_test_framebufferobject(self, timeout=1800):
        self._runPresentation.stop_app_am()
        self._runPresentation.countdown_t0_test_framebufferobject(timeout)

    def test_countdown_t0_test_kube(self, timeout=1800):
        self._runPresentation.stop_app_am()
        self._runPresentation.countdown_t0_test_kube(timeout)

    def test_countdown_t0_test_matrixpaletteskining(self, timeout=1800):
        self._runPresentation.stop_app_am()
        self._runPresentation.countdown_t0_test_matrixpaletteskining(timeout)

    def test_countdown_t0_test_opengles20(self, timeout=1800):
        self._runPresentation.stop_app_am()
        self._runPresentation.countdown_t0_test_opengles20(timeout)

    def test_countdown_t0_test_glsurfaceview(self, timeout=1800):
        self._runPresentation.stop_app_am()
        self._runPresentation.countdown_t0_test_glsurfaceview(timeout)

    def test_countdown_t0_test_spritetextactivity(self, timeout=1800):
        self._runPresentation.stop_app_am()
        self._runPresentation.countdown_t0_test_spritetextactivity(timeout)

    def test_countdown_t0_test_texturedtriangle(self, timeout=1800):
        self._runPresentation.stop_app_am()
        self._runPresentation.countdown_t0_test_texturedtriangle(timeout)

    def test_countdown_t0_test_touchrotate(self, timeout=1800):
        self._runPresentation.stop_app_am()
        self._runPresentation.countdown_t0_test_touchrotate(timeout)