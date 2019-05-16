# -*- coding: utf-8 -*-
'''
Created on 060/05/2015
@author: Ding, JunnanX
'''

from testlib.util.common import g_common_obj
from testlib.graphics.test_template.render_app_testbase import RenderAppTestBase
from testlib.graphics.common import logcat
from testlib.graphics.APIdemo_impl import APIDemoImpl
from testlib.graphics.extend_systemui_impl import SystemUiExtendImpl


class APIDemosGraphicsOpenGLESTest(RenderAppTestBase):

    @classmethod
    def setUpClass(cls):
        print "[setUpClass]: %s" % cls.__name__
        super(APIDemosGraphicsOpenGLESTest, cls).setUpClass()
        cls.d = g_common_obj.get_device()
        g_common_obj.root_on_device()
        g_common_obj.remount_device()

    def setUp(self):
        print "[Setup]:%s" % self.__class__.__name__
        super(APIDemosGraphicsOpenGLESTest, self).setUp()

        self.systemui = SystemUiExtendImpl()
        self.apidemo = APIDemoImpl()

        self.apidemo.clean()
        self.apidemo.setup()

        self.systemui.unlock_screen()
        self.d.screen.on()
        self.d.press.menu()

        self.mark_time = logcat.get_device_time_mark()

    def tearDown(self):
        print "[tearDown]:%s" % self.__class__.__name__
        app_pid = g_common_obj.adb_cmd_capture_msg("ps | grep 'com.example.android.apis' |awk '{print $2}'")
        fatal_msg = logcat.get_device_log(self.mark_time, filters='*:F', pid=app_pid)
        assert len(fatal_msg) == 0, "occurred Fatal error during testing:\n%s" % (fatal_msg)

        self.apidemo.clean()
        super(APIDemosGraphicsOpenGLESTest, self).tearDown()

    @classmethod
    def tearDownClass(cls):
        print "[tearDownClass]: %s" % cls.__name__
        super(APIDemosGraphicsOpenGLESTest, cls).tearDownClass()

    def test_APIDemos_CompressedTexture_30minutes(self):
        """
        test_APIDemos_CompressedTexture_30minutes

        Steps:
        1. Launch API Demo app and enter into "Graphics -> OpenGL ES". Test "Compressed Texture" for 30 minutes
        No error or crash occurs. Display is always correct no artifcat or freeze.
        """
        print "[RunTest]: %s" % self.__str__()

        print """[Step] 1. Launch API Demo app and enter into "Graphics -> OpenGL ES". Test "Compressed Texture" for 30 minutes
        No error or crash occurs. Display is always correct no artifcat or freeze."""
        self.apidemo.run_compressed_texture(count=1, step_time=60)

    def test_APIDemos_CubeMap_30minutes(self):
        """
        test_APIDemos_CubeMap_30minutes

        Steps:
        1. Launch API Demo app and enter into "Graphics -> OpenGL ES". Test "Cube Map" for 30 minutes
        No error or crash occurs. Display is always correct no artifcat or freeze.
        """
        print "[RunTest]: %s" % self.__str__()

        print """[Step] 1. Launch API Demo app and enter into "Graphics -> OpenGL ES". Test "Cube Map" for 30 minutes
        No error or crash occurs. Display is always correct no artifcat or freeze."""
        self.apidemo.run_cube_map(count=1, step_time=60)

    def test_APIDemos_FrameBufferObject_30minutes(self):
        """
        test_APIDemos_FrameBufferObject_30minutes

        Steps:
        1. Launch API Demo app and enter into "Graphics -> OpenGL ES". Test "Frame Buffer Object" for 30 minutes
        No error or crash occurs. Display is always correct no artifcat or freeze.
        """
        print "[RunTest]: %s" % self.__str__()

        print """[Step] 1. Launch API Demo app and enter into "Graphics -> OpenGL ES". Test "Frame Buffer Object" for 30 minutes
        No error or crash occurs. Display is always correct no artifcat or freeze."""
        self.apidemo.run_frame_buffer_object(count=1, step_time=60)

    def test_APIDemos_GLSurfaceView_30minutes(self):
        """
        test_APIDemos_FrameBufferObject_30minutes

        Steps:
        1. Launch API Demo app and enter into "Graphics -> OpenGL ES". Test "GLSurfaceView" for 30 minutes
        No error or crash occurs. Display is always correct no artifcat or freeze.
        """
        print "[RunTest]: %s" % self.__str__()

        print """[Step] 1. Launch API Demo app and enter into "Graphics -> OpenGL ES". Test "GLSurfaceView" for 30 minutes
        No error or crash occurs. Display is always correct no artifcat or freeze."""
        self.apidemo.run_gl_surface_view(count=1, step_time=60)

    def test_APIDemos_Kube_30minutes(self):
        """
        test_APIDemos_Kube_30minutes

        Steps:
        1. Launch API Demo app and enter into "Graphics -> OpenGL ES". Test "Kube" for 30 minutes
        No error or crash occurs. Display is always correct no artifcat or freeze.
        """
        print "[RunTest]: %s" % self.__str__()

        print """[Step] 1. Launch API Demo app and enter into "Graphics -> OpenGL ES". Test "Kube" for 30 minutes
        No error or crash occurs. Display is always correct no artifcat or freeze."""
        self.apidemo.run_kube(count=1, step_time=60)

    def test_APIDemos_MatrixPaletteSkinning_30minutes(self):
        """
        test_APIDemos_MatrixPaletteSkinning_30minutes

        Steps:
        1. Launch API Demo app and enter into "Graphics -> OpenGL ES". Test "Matrix Palette Skinning" for 30 minutes
        No error or crash occurs. Display is always correct no artifcat or freeze.
        """
        print "[RunTest]: %s" % self.__str__()

        print """[Step] 1. Launch API Demo app and enter into "Graphics -> OpenGL ES". Test "Matrix Palette Skinning" for 30 minutes
        No error or crash occurs. Display is always correct no artifcat or freeze."""
        self.apidemo.run_matrix_palette_skinning(count=1, step_time=60)

    def test_APIDemos_OpenGLES20(self):
        """
        test_APIDemos_OpenGLES20

        Steps:
        1. Launch API Demo app and enter into "Graphics -> OpenGL ES". Test "OpenGL ES 2.0" for 30 minutes
        No error or crash occurs. Display is always correct no artifcat or freeze.
        """
        print "[RunTest]: %s" % self.__str__()

        print """[Step] 1. Launch API Demo app and enter into "Graphics -> OpenGL ES". Test "OpenGL ES 2.0" for 30 minutes
        No error or crash occurs. Display is always correct no artifcat or freeze."""
        self.apidemo.run_opengles20(count=1, step_time=60)

    def test_APIDemos_SpriteText_30minutes(self):
        """
        test_APIDemos_SpriteText_30minutes

        Steps:
        1. Launch API Demo app and enter into "Graphics -> OpenGL ES". Test "Sprite Text" for 30 minutes
        No error or crash occurs. Display is always correct no artifcat or freeze.
        """
        print "[RunTest]: %s" % self.__str__()

        print """[Step] 1. Launch API Demo app and enter into "Graphics -> OpenGL ES". Test "Sprite Text" for 30 minutes
        No error or crash occurs. Display is always correct no artifcat or freeze."""
        self.apidemo.run_sprite_text(count=1, step_time=60)

    def test_APIDemos_TexturedTriangle_30minutes(self):
        """
        test_APIDemos_TexturedTriangle_30minutes

        Steps:
        1. Launch API Demo app and enter into "Graphics -> OpenGL ES". Test "Textured Triangle" for 30 minutes
        No error or crash occurs. Display is always correct no artifcat or freeze.
        """
        print "[RunTest]: %s" % self.__str__()

        print """[Step] 1. Launch API Demo app and enter into "Graphics -> OpenGL ES". Test "Textured Triangle" for 30 minutes
        No error or crash occurs. Display is always correct no artifcat or freeze."""
        self.apidemo.run_textured_triangle(count=1, step_time=60)

    def test_APIDemos_TouchRotate_30minutes(self):
        """
        test_APIDemos_TouchRotate_30minutes

        Steps:
        1. Launch API Demo app and enter into "Graphics -> OpenGL ES". Test "Touch Rotate" for 30 minutes
        No error or crash occurs. Display is always correct no artifcat or freeze.
        """
        print "[RunTest]: %s" % self.__str__()

        print """[Step] 1. Launch API Demo app and enter into "Graphics -> OpenGL ES". Test "Touch Rotate" for 30 minutes
        No error or crash occurs. Display is always correct no artifcat or freeze."""
        self.apidemo.run_touch_rotate(count=1, step_time=60)
