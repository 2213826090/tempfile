# -*- coding: utf-8 -*-
"""
@author: yusux
"""
from datetime import datetime
import sys
import time
import thread
from multiprocessing import Process
from testlib.util.common import g_common_obj
from testlib.util.config import TestConfig
from testlib.util.repo import Artifactory
from testlib.graphics.tools import ConfigHandle
from testlib.dut_init.dut_init_impl import Function
import pdb


class SampleApiDemoImpl(object):
    pkg_name = "com.example.android.apis"
    activity_name = "com.example.android.apis.ApiDemos"
    current_time = datetime.now()

    def __init__(self):
        self.device = g_common_obj.get_device()

    def unlock(self, raiseError=False):
        """
            Unlock screen by via input keyevent 82
        """
        self.device.wakeup()
        time.sleep(0.5)
        if self.device(resourceId="com.android.keyguard:id/keyguard_selector_view_frame").exists:
            self.device(resourceId="com.android.keyguard:id/keyguard_selector_view_frame").swipe.right()
        if self.device(resourceId="com.android.systemui:id/lock_icon").exists:
            w = self.device.info[u'displayWidth']
            h = self.device.info[u'displayHeight']
            self.device.swipe(w / 2, h, w / 2, 0)

    @staticmethod
    def install_apk():
        """
            install from Artifactory
        """
        config = TestConfig()
        cfg_file = 'tests.tablet.artifactory.conf'
        cfg_arti = config.read(cfg_file, 'artifactory')
        config_handle = ConfigHandle()
        cfg_arti["location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat/', 'sys.conf')
        cfg = config.read(cfg_file, 'content_apidemos')
        arti = Artifactory(cfg_arti.get('location'))
        apk_name = cfg.get("name")
        file_path = arti.get(apk_name)
        result = config_handle.check_apps(SampleApiDemoImpl.pkg_name)
        if result == 0:
            g_common_obj.adb_cmd_common('install -r ' + str(file_path))

    def restart_jsonrpc_server(self):

        """
        try to start jsonrpc server
        """
        try:
            self.device = g_common_obj.get_device()
            self.device.info
        except Exception as e:
            print "start json rpc server failed at: " + e

    @staticmethod
    def launch_app_am():
        """
        launchApp by pkgname/activityname
        """
        g_common_obj.stop_app_am(
            SampleApiDemoImpl.pkg_name)
        g_common_obj.launch_app_am(
            SampleApiDemoImpl.pkg_name, SampleApiDemoImpl.activity_name)
        print "Launched APIDemo by adb am"

    @staticmethod
    def stop_app_am():
        print "stop APIDemo by adb am"
        g_common_obj.stop_app_am(SampleApiDemoImpl.pkg_name)

    def perform_presentation_through_mediarouter(self):
        """
            start Presentation_through_mediarouter by UIObject plz plugin \
            HDMI cable if  case necessary & DUT was ready
        """
        self.restart_jsonrpc_server()
        self.device(text="App").click()
        time.sleep(2)
        self.device(text="Activity").click()
        time.sleep(2)
        self.device().scroll.to(
            text="Presentation with Media Router")
        self.device(text="Presentation with Media Router").click()
        time.sleep(5)
        start_time = self.current_time
        print start_time

    def countdown_t0_perform_presentation_through_mediarouter(self, timeout):
        """
            perform_presentation_through_wirelessHDMI last count down
        """
        runcast = SampleApiDemoImpl()
        runcast.perform_presentation_through_mediarouter()
        count = 0
        cmd = "dumpsys window|grep mCurrentFocus"
        errormessage = "PresentationWithMediaRouterActivity is interrupted"
        while timeout > count:
            ncount = timeout - count
            if ncount > 1:
                print("there were %d seconds left " % ncount)
            else:
                print("there were %d second left " % ncount)
            mcurrent_focus = g_common_obj.adb_cmd_capture_msg(cmd)
            sys.stdout.flush()
            time.sleep(1)
            assert mcurrent_focus.__contains__("PresentationWithMediaRouterActivity"), errormessage
            count += 1
            if count >= timeout:
                self.device.press.home()
                print "\rcase has finished succuessfully!"
        assert count == timeout

    def switch_camera_preview(self, switch_times=24):
        """
            perform unexist menu press to switch front camera and rear camera in APIDemos--->Graphics--->CameraPreview
        """
        self.unlock(self)
        g_common_obj.launch_app_from_home_sc("API Demos")
        self.device(text="Graphics").click()
        self.device(
            text="CameraPreview", className="android.widget.TextView").click()
        # here you can change switch times
        for d in xrange(switch_times):
            if self.device.press.menu():
                True
                assert self.device(text="Switch Camera").exists is True
                self.device(text="Switch Camera").click()

    def translucent_glsurfaceview(self):
        """
            locat APIDemos--->Graphics--->OpenGL ES--->Translucent GLSurfaceView
        """
        self.unlock(self)
        self.launch_app_am()
        self.device(text="Graphics").click()
        self.device(scrollable=True).scroll.vert.forward(steps=50)
        self.device(text="OpenGL ES").click()
        self.device(text="Translucent GLSurfaceView").click()

    @staticmethod
    def countdown_t0_translucent_glsurfaceview(timeout):
        runcast = SampleApiDemoImpl()
        runcast.translucent_glsurfaceview()
        count = 0
        cmd = "dumpsys window|grep mCurrentFocus"

        errormessage = "TranslucentGLSurfaceView is interrupted"

        while timeout > count:
            ncount = timeout - count
            if ncount > 1:
                print("there were %d seconds left " % ncount)
            else:
                print("there were %d second left " % ncount)
            mcurrent_focus = g_common_obj.adb_cmd_capture_msg(cmd).strip('\n\t\r\f')
            sys.stdout.flush()
            time.sleep(1)
            assert mcurrent_focus.__contains__("TranslucentGLSurfaceViewActivity"), errormessage
            count += 1
            if count >= timeout:
                print "\rcase has finished succuessfully!"
        assert count == timeout

    def start_presentation(self, checkboxtime=15):
        """
            locate presentation in apidemos,
            :param checkboxtime is switch times
        """
        self.unlock(self)
        self.device.press.home()
        print "start_presentation by UIObject,plz plugin HDMI cable if  case necessary &  DUT was ready"
        self.launch_app_am()
        self.device(text="App").click()
        self.device(text="Activity").click()
        self.device(scrollable=True).scroll.to(text="Presentation")
        self.device(
            text="Presentation", className="android.widget.TextView").click()
        if not self.device(className="android.widget.CheckBox",
                           resourceId="com.example.android.apis:id/show_all_displays") \
                .checked:
            self.device(className="android.widget.CheckBox", resourceId="com.example.android.apis:id/show_all_displays") \
                .click()
            # for i in range(checkboxtime):
            time.sleep(2)
            if self.device(textStartsWith='Display #1').exists:
                self.device(textStartsWith='Display #1').left(className="android.widget.CheckBox").click()
                # self.device.press.back()
        print "presentations is successfully done!."

    def locate_texturecompress(self):
        """
            locate_texturecompress
        """
        self.unlock(self)
        self.launch_app_am()
        self.device(text="Graphics").click()
        self.device(scrollable=True).scroll.vert.forward(steps=50)
        self.device(text="OpenGL ES").click()
        self.device(text="Compressed Texture").click()

    @staticmethod
    def countdown_t0_test_compresstexture(timeout):
        """
            countdown_t0_test_compresstexture
            :param timeout usually for 30minutss
        """
        runcast = SampleApiDemoImpl()
        runcast.locate_texturecompress()
        count = 0
        cmd = "dumpsys window|grep mCurrentFocus"

        errormessage = "CompressedTextureActivity is interrupted"

        while timeout > count:
            ncount = timeout - count
            if ncount > 1:
                print("there were %d seconds left " % ncount)
            else:
                print("there were %d second left " % ncount)
            mcurrent_focus = g_common_obj.adb_cmd_capture_msg(cmd).strip('\n\t\r\f')
            sys.stdout.flush()
            time.sleep(1)
            assert mcurrent_focus.__contains__(".CompressedTextureActivity"), errormessage
            count += 1
            if count >= timeout:
                print "\rcase has finished succuessfully!"
        assert count == timeout

    def locate_cubemap(self):
        """
            locate_cubemap
        """
        self.unlock(self)
        self.launch_app_am()
        self.device(text="Graphics").click()
        self.device(scrollable=True).scroll.vert.forward(steps=50)
        self.device(text="OpenGL ES").click()
        self.device(text="Cube Map").click()

    @staticmethod
    def countdown_t0_test_cubemap(timeout):
        """
            countdown_t0_test_cubemap
            :param timeout usually for 30minutss
        """
        runcast = SampleApiDemoImpl()
        runcast.locate_cubemap()
        count = 0
        cmd = "dumpsys window|grep mCurrentFocus"

        errormessage = "CubeMapActivity is interrupted"

        while timeout > count:
            ncount = timeout - count
            if ncount > 1:
                print("there were %d seconds left " % ncount)
            else:
                print("there were %d second left " % ncount)
            mcurrent_focus = g_common_obj.adb_cmd_capture_msg(cmd).strip('\n\t\r\f')
            sys.stdout.flush()
            time.sleep(1)
            assert mcurrent_focus.__contains__(".CubeMapActivity"), errormessage
            count += 1
            if count >= timeout:
                print "\rcase has finished succuessfully!"
        assert count == timeout

    def locate_framebuffer_object(self):
        """
            locate_framebuffer_object
        """
        self.unlock(self)
        self.launch_app_am()
        self.device(text="Graphics").click()
        self.device(scrollable=True).scroll.vert.forward(steps=50)
        self.device(text="OpenGL ES").click()
        self.device(text="Frame Buffer Object").click()

    @staticmethod
    def countdown_t0_test_framebufferobject(timeout):
        """
            countdown_t0_test_framebufferobject
            :param:timeout
        """
        runcast = SampleApiDemoImpl()
        runcast.locate_framebuffer_object()
        count = 0
        cmd = "dumpsys window|grep mCurrentFocus"

        errormessage = "FrameBufferObjectActivity is interrupted"

        while timeout > count:
            ncount = timeout - count
            if ncount > 1:
                print("there were %d seconds left " % ncount)
            else:
                print("there were %d second left " % ncount)
            mcurrent_focus = g_common_obj.adb_cmd_capture_msg(cmd).strip('\n\t\r\f')
            sys.stdout.flush()
            time.sleep(1)
            assert mcurrent_focus.__contains__(".FrameBufferObjectActivity"), errormessage
            count += 1
            if count >= timeout:
                print "\rcase has finished succuessfully!"
        assert count == timeout

    def locate_kube(self):
        """
            locate_kube
        """
        self.unlock(self)
        self.launch_app_am()
        self.device(text="Graphics").click()
        self.device(scrollable=True).scroll.vert.forward(steps=50)
        self.device(text="OpenGL ES").click()
        self.device(text="Kube").click()

    @staticmethod
    def countdown_t0_test_kube(timeout):
        """
        countdown_t0_test_kube
        :param:timeout
        """
        runcast = SampleApiDemoImpl()
        runcast.locate_kube()
        count = 0
        cmd = "dumpsys window|grep mCurrentFocus"

        errormessage = "kube is interrupted"

        while timeout > count:
            ncount = timeout - count
            if ncount > 1:
                print("there were %d seconds left " % ncount)
            else:
                print("there were %d second left " % ncount)
            mcurrent_focus = g_common_obj.adb_cmd_capture_msg(cmd).strip('\n\t\r\f')
            sys.stdout.flush()
            time.sleep(1)
            assert mcurrent_focus.__contains__(".kube"), errormessage
            count += 1
            if count >= timeout:
                print "\rcase has finished succuessfully!"
        assert count == timeout

    def locate_matrixpalettesking(self):
        """
            locate_kube
        """
        self.unlock(self)
        self.launch_app_am()
        self.device(text="Graphics").click()
        self.device(scrollable=True).scroll.vert.forward(steps=50)
        self.device(text="OpenGL ES").click()
        self.device(text="Matrix Palette Skinning").click()

    @staticmethod
    def countdown_t0_test_matrixpaletteskining(timeout):
        """
            test_matrixpaletteskining
        """
        runcast = SampleApiDemoImpl()
        runcast.locate_matrixpalettesking()
        count = 0
        cmd = "dumpsys window|grep mCurrentFocus"

        errormessage = "MatrixPaletteActivity is interrupted"

        while timeout > count:
            ncount = timeout - count
            if ncount > 1:
                print("there were %d seconds left " % ncount)
            else:
                print("there were %d second left " % ncount)
            mcurrent_focus = g_common_obj.adb_cmd_capture_msg(cmd).strip('\n\t\r\f')
            sys.stdout.flush()
            time.sleep(1)
            assert mcurrent_focus.__contains__(".MatrixPaletteActivity"), errormessage
            count += 1
            if count >= timeout:
                print "\rcase has finished succuessfully!"
        assert count == timeout

    def locate_opengles(self):
        """
        locate ogles20
        """
        self.unlock(self)
        self.launch_app_am()
        self.device(text="Graphics").click()
        self.device(scrollable=True).scroll.vert.forward(steps=50)
        self.device(text="OpenGL ES").click()
        self.device(text="OpenGL ES 2.0").click()

    @staticmethod
    def countdown_t0_test_opengles20(timeout):
        """
            test_matrixpaletteskining
        """
        runcast = SampleApiDemoImpl()
        runcast.locate_opengles()
        count = 0
        cmd = "dumpsys window|grep mCurrentFocus"

        errormessage = "MatrixPaletteActivity is interrupted"

        while timeout > count:
            ncount = timeout - count
            if ncount > 1:
                print("there were %d seconds left " % ncount)
            else:
                print("there were %d second left " % ncount)
            mcurrent_focus = g_common_obj.adb_cmd_capture_msg(cmd).strip('\n\t\r\f')
            sys.stdout.flush()
            time.sleep(1)
            assert mcurrent_focus.__contains__(".GLES20Activity"), errormessage
            count += 1
            if count >= timeout:
                print "\rcase has finished succuessfully!"
        assert count == timeout

    def locate_glsurfaceview(self):
        """
        locate ogles20
        """
        self.unlock(self)
        self.launch_app_am()
        self.device(text="Graphics").click()
        self.device(scrollable=True).scroll.vert.forward(steps=50)
        self.device(text="OpenGL ES").click()
        self.device(text="GLSurfaceView").click()

    @staticmethod
    def countdown_t0_test_glsurfaceview(timeout):
        """
            test_glsurfaceview
            :param timeout:
        :return:
        """
        runcast = SampleApiDemoImpl()
        runcast.locate_glsurfaceview()
        count = 0
        cmd = "dumpsys window|grep mCurrentFocus"

        errormessage = "GLSurfaceViewActivity is interrupted"

        while timeout > count:
            ncount = timeout - count
            if ncount > 1:
                print("there were %d seconds left " % ncount)
            else:
                print("there were %d second left " % ncount)
            mcurrent_focus = g_common_obj.adb_cmd_capture_msg(cmd).strip('\n\t\r\f')
            sys.stdout.flush()
            time.sleep(1)
            assert mcurrent_focus.__contains__(".GLSurfaceViewActivity"), errormessage
            count += 1
            if count >= timeout:
                print "\rcase has finished succuessfully!"
        assert count == timeout

    def locate_spritetext(self):
        """
        locate ogles20
        """
        self.unlock(self)
        self.launch_app_am()
        self.device(text="Graphics").click()
        self.device(scrollable=True).scroll.vert.forward(steps=50)
        self.device(text="OpenGL ES").click()
        self.device(text="Sprite Text").click()

    @staticmethod
    def countdown_t0_test_spritetextactivity(timeout):
        """
            countdown_t0_test_spritetextactivity
        :param timeout:
        :return:
        """
        runcast = SampleApiDemoImpl()
        runcast.locate_spritetext()
        count = 0
        cmd = "dumpsys window|grep mCurrentFocus"

        errormessage = "SpriteTextActivity is interrupted"

        while timeout > count:
            ncount = timeout - count
            if ncount > 1:
                print("there were %d seconds left " % ncount)
            else:
                print("there were %d second left " % ncount)
            mcurrent_focus = g_common_obj.adb_cmd_capture_msg(cmd).strip('\n\t\r\f')
            sys.stdout.flush()
            time.sleep(1)
            assert mcurrent_focus.__contains__(".SpriteTextActivity"), errormessage
            count += 1
            if count >= timeout:
                print "\rcase has finished succuessfully!"
        assert count == timeout

    def locate_texturedtriangle(self):
        """
        locate spritetext
        """
        self.unlock(self)
        self.launch_app_am()
        self.device(text="Graphics").click()
        self.device(scrollable=True).scroll.vert.forward(steps=50)
        self.device(text="OpenGL ES").click()
        self.device(text="Textured Triangle").click()

    @staticmethod
    def countdown_t0_test_texturedtriangle(timeout):
        """
            countdown_t0_test_texturedtriangle
        :param timeout:
        :return:
        """
        runcast = SampleApiDemoImpl()
        runcast.locate_texturedtriangle()
        count = 0
        cmd = "dumpsys window|grep mCurrentFocus"

        errormessage = "Textured Triangle is interrupted"

        while timeout > count:
            ncount = timeout - count
            if ncount > 1:
                print("there were %d seconds left " % ncount)
            else:
                print("there were %d second left " % ncount)
            mcurrent_focus = g_common_obj.adb_cmd_capture_msg(cmd).strip('\n\t\r\f')
            sys.stdout.flush()
            time.sleep(1)
            assert mcurrent_focus.__contains__(".TriangleActivity"), errormessage
            count += 1
            if count >= timeout:
                print "\rcase has finished succuessfully!"
        assert count == timeout

    def locate_touchrotate(self):

        """
            locate touch rotate
        """
        self.unlock(self)
        self.launch_app_am()
        self.device(text="Graphics").click()
        self.device(scrollable=True).scroll.vert.forward(steps=50)
        self.device(text="OpenGL ES").click()
        self.device(text="Touch Rotate").click.wait()
        config = TestConfig()
        cfg_file = 'tests.tablet.artifactory.conf'
        cfg_arti = config.read(cfg_file, 'artifactory')
        config_handle = ConfigHandle()
        cfg_arti["location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat/', 'sys.conf')
        cfg = config.read(cfg_file, 'content_touchrotateevents')
        arti = Artifactory(cfg_arti.get('location'))
        name = cfg.get("name")
        file_path = arti.get(name)
        g_common_obj.shell_cmd('chmod 777 ' + file_path)
        return file_path

    @staticmethod
    def countdown_t0_test_touchrotate(timeout):
        """
            countdown_t0_test_texturedtriangle
        :param timeout:
        :return:
        """

        runcast = SampleApiDemoImpl()
        file_path = runcast.locate_touchrotate()
        count = 0
        cmd = "dumpsys window|grep mCurrentFocus"
        errormessage = "TouchRotateActivity is interrupted"
        g_common_obj.shell_cmd('adb root ||adb remount')
        g_common_obj.shell_cmd('bash ' + file_path)
        while timeout > count:
            ncount = timeout - count
            if ncount > 1:
                print("there were %d seconds left " % ncount)
            else:
                print("there were %d second left " % ncount)
            mcurrent_focus = g_common_obj.adb_cmd_capture_msg(cmd).strip('\n\t\r\f')
            sys.stdout.flush()
            time.sleep(1)
            assert mcurrent_focus.__contains__(".TouchRotateActivity"), errormessage
            count += 1
            if count >= timeout:
                print "\rcase has finished succuessfully!"
        assert count == timeout
