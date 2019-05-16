# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 06/11/2015
@author: Xiangyi Zhao
'''
import time
from testlib.graphics.common import logcat
from testlib.graphics.extend_systemui_impl import SystemUiExtendImpl
# from testlib.graphics.ImageApp_MeituPic_impl import MeituPicEdit
from testlib.graphics.ImageApp_PhotoGrid_impl import PhotoGridEdit
from testlib.graphics.ImageApp_PicsArtPhotoStudio_impl import PicsArtStudio
from testlib.graphics.ImageApp_PicSay_impl import PicSay
from testlib.graphics.ImageApp_SketchBookExpress_impl import SketchBookX
from testlib.graphics.ImageApp_Skitch_impl import Skitch
from testlib.util.uiatestbase import UIATestBase
from testlib.util.config import TestConfig
from testlib.util.repo import Artifactory
from testlib.graphics.tools import ConfigHandle
from testlib.util.common import g_common_obj


class DisplayAllAppsOnHomeScreen(UIATestBase):

    @classmethod
    def setUpClass(cls):
        print "[setUpClass]: %s" % cls.__name__
        super(DisplayAllAppsOnHomeScreen, cls).setUpClass()
        cls.d = g_common_obj.get_device()
        g_common_obj.root_on_device()
        g_common_obj.remount_device()
        config = TestConfig()
        cfg_file = 'tests.tablet.artifactory.conf'
        cfg_arti = config.read(cfg_file, 'artifactory')
        config_handle = ConfigHandle()
        cfg_arti["location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat/', 'sys.conf')
        # Change to photoGrid app since meitupic is not support for x86 cpu
        cfg_picasatool = config.read(cfg_file, 'content_photogrid')
        arti = Artifactory(cfg_arti.get('location'))
        binary_name = cfg_picasatool.get("name")
        file_path = arti.get(binary_name)
        print "[Setup]: install photogrid"
        result = config_handle.check_apps("com.roidapp.photogrid")
        if result == 0:
            g_common_obj.adb_cmd_common('install ' + file_path)

        cfg_picsart = config.read(cfg_file, 'content_picsart')
        arti = Artifactory(cfg_arti.get('location'))
        binary_name = cfg_picsart.get("name")
        file_path = arti.get(binary_name)
        print "[Setup]: install picsart"
        result = config_handle.check_apps("com.picsart.studio")
        if result == 0:
            g_common_obj.adb_cmd_common('install ' + file_path)

        cfg_picsay = config.read(cfg_file, 'content_picsay')
        arti = Artifactory(cfg_arti.get('location'))
        binary_name = cfg_picsay.get("name")
        file_path = arti.get(binary_name)
        print "[Setup]: install picsay"
        result = config_handle.check_apps("com.shinycore.picsayfree")
        if result == 0:
            g_common_obj.adb_cmd_common('install ' + file_path)

        cfg_sketchbook = config.read(cfg_file, 'content_sketchbook')
        arti = Artifactory(cfg_arti.get('location'))
        binary_name = cfg_sketchbook.get("name")
        file_path = arti.get(binary_name)
        print "[Setup]: install sketchbook"
        result = config_handle.check_apps("com.adsk.sketchbookhdexpress")
        if result == 0:
            g_common_obj.adb_cmd_common('install ' + file_path)

        cfg_skitch = config.read(cfg_file, 'content_skitch')
        arti = Artifactory(cfg_arti.get('location'))
        binary_name = cfg_skitch.get("name")
        file_path = arti.get(binary_name)
        print "[Setup]: install skitch"
        result = config_handle.check_apps("com.evernote.skitch")
        if result == 0:
            g_common_obj.adb_cmd_common('install ' + file_path)

    def setUp(self):
        print "[Setup]:%s" % self.__class__.__name__
        super(DisplayAllAppsOnHomeScreen, self).setUp()

        self.systemui = SystemUiExtendImpl()
        # self.meitu_impl = MeituPicEdit()
        self.systemui.unlock_screen()
        self.d.screen.on()
        self.d.press.menu()

        self.mark_time = logcat.get_device_time_mark()

    def tearDown(self):
        print "[tearDown]:%s" % self.__class__.__name__
        super(DisplayAllAppsOnHomeScreen, self).tearDown()
        # self.meitu_impl.uninstall_app()
        PhotoGridEdit().uninstall_app()
        PicsArtStudio().uninstall_app()
        PicSay().uninstall_app()
        SketchBookX().uninstall_app()
        Skitch().uninstall_app()

    @classmethod
    def tearDownClass(cls):
        print "[tearDownClass]: %s" % cls.__name__
        super(DisplayAllAppsOnHomeScreen, cls).tearDownClass()

    def test_Display_AllAppsOnHomeScreen(self):
        """
        test_Display_AllAppsOnHomeScreen
        """
        # self.meitu_impl.launch_app_am()
        PhotoGridEdit().launch_app_am()
        app_pid = g_common_obj.adb_cmd_capture_msg("ps | grep '%s' |awk '{print $2}'" % (PhotoGridEdit().pkg_name))

        for i in range(6):
            time.sleep(5)
            _, activity = self.systemui.get_current_focus()
            if not activity == '':
                break
        assert activity == PhotoGridEdit().activity_name, "Check MeituPicEdit launch fail"

        fatal_msg = logcat.get_device_log(self.mark_time, filters='*:F', pid=app_pid)
        assert len(fatal_msg) == 0, "occurred Fatal error during testing:\n%s" % (fatal_msg)

        PhotoGridEdit().stop_app_am()
        time.sleep(2)

        PicsArtStudio().launch_app_am()
        _, activity = self.systemui.get_current_focus()
        assert activity == PicsArtStudio().activity_name, "Check PicsArtStudio launch fail"
#         fatal_msg = logcat.get_device_log(self.mark_time, filters='*:F')
#         assert len(fatal_msg) == 0, \
#             "occurred Fatal error during testing:\n%s" % (fatal_msg)
        PicsArtStudio().stop_app_am()
        time.sleep(2)

        PicSay().launch_app_am()
        _, activity = self.systemui.get_current_focus()
        assert activity == PicSay().activity_name, "Check PicSay launch fail"

        fatal_msg = logcat.get_device_log(self.mark_time, filters='*:F', pid=app_pid)
        assert len(fatal_msg) == 0, "occurred Fatal error during testing:\n%s" % (fatal_msg)

        PicSay().stop_app_am()
        time.sleep(2)

        SketchBookX().launch_app_am()
        _, activity = self.systemui.get_current_focus()
        assert activity == SketchBookX().activity_name, "Check SketchBookX launch fail"
#         fatal_msg = logcat.get_device_log(self.mark_time, filters='*:F')
#         assert len(fatal_msg) == 0, \
#             "occurred Fatal error during testing:\n%s" % (fatal_msg)
        SketchBookX().stop_app_am()
        time.sleep(2)

        Skitch().launch_app_am()
        _, activity = self.systemui.get_current_focus()
        assert activity == Skitch().activity_name, "Check Skitch launch fail"
#         fatal_msg = logcat.get_device_log(self.mark_time, filters='*:F')
#         assert len(fatal_msg) == 0, \
#             "occurred Fatal error during testing:\n%s" % (fatal_msg)
        Skitch().stop_app_am()
        time.sleep(2)
