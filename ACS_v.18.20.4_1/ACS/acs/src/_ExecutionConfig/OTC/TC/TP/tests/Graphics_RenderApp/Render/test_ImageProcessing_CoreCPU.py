# -*- coding: utf-8 -*-
'''
Created on 04/22/2015
@author: Ding, JunnanX
'''

from testlib.util.common import g_common_obj
from testlib.graphics.test_template.render_app_testbase import RenderAppTestBase
from testlib.graphics.common import logcat
from testlib.graphics.extend_systemui_impl import SystemUiExtendImpl
from testlib.graphics.system_settings_impl import SystemSettingsImpl
from testlib.graphics.extend_imageprocessing_impl import ImageProcessingImpl


class ImageProcessingCoreCPUTest(RenderAppTestBase):

    @classmethod
    def setUpClass(cls):
        print "[setUpClass]: %s" % cls.__name__
        super(ImageProcessingCoreCPUTest, cls).setUpClass()
        cls.d = g_common_obj.get_device()
        g_common_obj.root_on_device()
        g_common_obj.remount_device()

    def setUp(self):
        print "[Setup]:%s" % self.__class__.__name__
        super(ImageProcessingCoreCPUTest, self).setUp()
        self.mark_time = logcat.get_device_time_mark()

        self.sys_setting = SystemSettingsImpl()
        self.systemui = SystemUiExtendImpl()
        self.imageprocessing = ImageProcessingImpl()

        self.imageprocessing.setup()

        self.systemui.unlock_screen()
        self.d.screen.on()

    def tearDown(self):
        print "[tearDown]:%s" % self.__class__.__name__

        app_pid = g_common_obj.adb_cmd_capture_msg("ps | grep '%s' |awk '{print $2}'" % (ImageProcessingImpl.PKG_NAME))
        fatal_msg = logcat.get_device_log(self.mark_time, filters='*:F', pid=app_pid)
        assert len(fatal_msg) == 0, "occurred Fatal error during testing:\n%s" % (fatal_msg)

        self.imageprocessing.clean()

    @classmethod
    def tearDownClass(cls):
        print "[tearDownClass]: %s" % cls.__name__
        super(ImageProcessingCoreCPUTest, cls).tearDownClass()

    def test_ImageProcessing_CoreCPU(self):
        """
        test_ImageProcessing_CoreCPU

        Steps:
        1. Launch ImageProcessing.
            The application starts and run without issues. Check message like “Computer Device: CPU” in logcat
        2. change some settings.
            Applying settings to the image successfully
        """
        print "[RunTest]: %s" % self.__str__()

        print """[Step] 1. Launch ImageProcessing.
            The application starts and run without issues. Check message like “Computer Device: CPU” in logcat"""
        time_mark = logcat.get_device_time_mark()
        self.imageprocessing.launch()
        self.imageprocessing.verify_log_core_cpu(time_mark)

        print """[Step] 2. change some settings.
            Applying settings to the image successfully
            """
        self.imageprocessing.change_levels_vec4_relaxed_settings()
        self.imageprocessing.change_levels_vec3_relaxed_settings()
