"""
Created on Jan 26, 2015
@author: yusux
"""
import os
import time
from photosMgrImpl import PhotosPPImpl
from testlib.util.config import TestConfig
from testlib.util.repo import Artifactory
from testlib.graphics.tools import ConfigHandle
from testlib.util.common import g_common_obj
from testlib.dut_init.dut_init_impl import Function


class SwitchFolderImpl(PhotosPPImpl):
    def __init__(self, cfg={}):
        self.cfg = cfg
        self._device = g_common_obj.get_device()
        super.__init__

    @staticmethod
    def get_pictures():
        """
            download pictures tar folders from artifactory
        """
        if not os.path.isdir("~/Pictures") is True:
            g_common_obj.shell_cmd(
                "mkdir -p ~/Pictures")
        config = TestConfig()
        cfg_file = 'tests.tablet.artifactory.conf'
        cfg_arti = config.read(cfg_file, 'artifactory')
        config_handle = ConfigHandle()
        cfg_arti["location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat/', 'sys.conf')
        cfg = config.read(cfg_file, 'content_picture')
        arti = Artifactory(cfg_arti.get('location'))

        pic_name1 = cfg.get("pngtar")
        file_path1 = arti.get(pic_name1)
        g_common_obj.shell_cmd("mv " + file_path1 + " ~/Pictures/PNG.tar.gz ")

        pic_name2 = cfg.get("webptar")
        file_path2 = arti.get(pic_name2)
        g_common_obj.shell_cmd("mv " + file_path2 + " ~/Pictures/WEBP.tar.gz ")

        # assert os.path.isdir("~/Pictures") is True
        os.system("cd ~/Pictures ")
        os.system("echo change to dir ~/pictures $?")

        os.system("tar xzvf ~/Pictures/PNG.tar.gz -C ~/Pictures/ ")
        os.system("echo unarchive png $?")
        os.system("tar xzvf ~/Pictures/WEBP.tar.gz -C ~/Pictures/ ")
        os.system("echo unarchive webp $?")

    @staticmethod
    def push_content():
        """
            push_media_contents_into_two_level_different_folder
        """
        os.system("cd ~/Pictures")
        os.system("echo change to dir ~/pictures $?")
        g_common_obj.adb_cmd_common("shell mkdir -p /sdcard/DCIM/png")
        g_common_obj.push_file(
            '~/Pictures/PNG', '/sdcard/DCIM/png/')
        g_common_obj.adb_cmd_common("shell mkdir -p /sdcard/Pictures/webp")
        g_common_obj.push_file(
            '~/Pictures/WEBP', '/sdcard/Pictures/webp/')

    @staticmethod
    def pull_and_push_thumbernail():
        """
            pull mediaProvider generates thumbernail to Host
        """
        os.system("cd ~/Pictures")
        PhotosPPImpl().refresh_sdcard()
        os.system("echo change to dir ~/Pictures $?")
        os.system("mkdir -p ~/Pictures/thumbnails")
        g_common_obj.pull_file("~/Pictures/thumbnails", "/sdcard/DCIM/.thumbnails")
        time.sleep(2)
        g_common_obj.push_file("~/Pictures/thumbnails", "/sdcard/thumbnails")
        time.sleep(2)
        PhotosPPImpl().refresh_sdcard()

    def browse_thumbernail(self, switchtimes):
        """
            push_thumbnailsintoPhotostocheck
        """
        for x in xrange(switchtimes):
            PhotosPPImpl().navigate_to_ondevice()
            if self._device(text="Camera").exists:
                self._device(text="Camera").click()
            else:
                self._device(scrollable=True).scroll.to(text="Camera")
            self._device(text="Camera").click()
            self._device(resourceId="com.google.android.apps.plus:id/grid").swipe.down(steps=100)
            self.navigator_up()
            if self._device(text="webp").exists:
                self._device(text="webp").click()
            else:
                self._device(scrollable=True).scroll.to(text="webp")
            self._device(text="webp").click()
            self.navigator_up()
        if self._device(text="thumbnails").exists:
            self._device(text="thumbnails").click()
        else:
            self._device(scrollable=True).scroll.to(text="thumbnails")
        self._device(text="thumbnails").click()
        self._device(resourceId="com.google.android.apps.plus:id/grid").swipe.down(steps=100)
        assert self._device(resourceId="com.google.android.apps.plus:id/grid") \
            .child(className="android.view.View").exists is True

    def openpngfolders(self):
        """
            navigate_to_png_folder
        """
        if self._device(text="Camera").exists:
            self._device(text="Camera").click()
        else:
            self._device(scrollable=True).scroll.to(text="Camera")
        self._device(text="Camera").click()
        self._device(resourceId="com.google.android.apps.plus:id/grid") \
            .child(index='1', className="android.view.View").click()
        time.sleep(3)
        self._device(description="Navigate up").click()

    def openwebpfolders(self):
        """
            navigate_to_webp_folder
        """
        if self._device(text="webp").exists:
            self._device(text="webp").click()
        else:
            self._device(scrollable=True).scroll.to(text="webp")
        self._device(text="webp").click()
        self._device(resourceId="com.google.android.apps.plus:id/grid") \
            .child(className="android.view.View").click()
        time.sleep(3)
        self._device(description="Navigate up").click()

    def navigator_up(self):
        """
            navigatorUp_level_folder
        """
        Function().push_uiautomator_jar()
        assert self._device(description="Navigate up", className="android.widget.ImageButton").exists
        self._device(description="Navigate up", className="android.widget.ImageButton").click()

    @staticmethod
    def rm_installed_content():
        """
            remove installed push Content
        """
        g_common_obj.adb_cmd_common(
            "shell rm -rf /sdcard/DCIM/png /sdcard/Pictures/webp")
