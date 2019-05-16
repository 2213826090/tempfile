"""
Created on Jan 21, 2015
@author: yusux
"""
import os
import time
from testlib.dut_init.dut_init_impl import Function
from testlib.util.config import TestConfig
from testlib.util.repo import Artifactory
from testlib.graphics.tools import ConfigHandle
from testlib.util.common import g_common_obj


class PhotosPPImpl:
    def __init__(self):
        self._device = g_common_obj.get_device()

    def init_photos_signin(self):
        self._device.press.home()
        g_common_obj.adb_cmd(
            "am start -S com.google.android.apps.plus/com.google.android.apps.photos.phone.PhotosLauncherActivity")
        time.sleep(2)
        if self._device(text="Later").exists:
            self._device(text="Later").click()
            time.sleep(2)
        if self._device(text="ALL").exists and not self._device(text="No thanks").exists:
            g_common_obj.adb_cmd("am force-stop com.google.android.apps.photos")
            return
        for i in range(10):
            if self._device(text="ALL").exists and not self._device(text="No thanks").exists:
                # if not self._device(resourceId="android:id/progress").exists:
                break
            if self._device(text="Cancel").exists:
                self._device(text="Cancel").click.wait()
            if self._device(text="No thanks").exists:
                self._device(text="No thanks").click.wait()
            if self._device(text="All").exists:
                self._device(text="ALL").click.wait()
            time.sleep(2)
        assert not self._device(text="No thanks").exists
        self._device.press.back()
        self._device(description="Photos").click.wait()
        time.sleep(2)
        if self._device(text="No thanks").exists:
            self._device(text="No thanks").click.wait()
        assert self._device(text="Photos").exists and not self._device(text="No thanks").exists
        g_common_obj.adb_cmd("am force-stop com.google.android.apps.photos")
        g_common_obj.adb_cmd(
            "am start -S -n com.google.android.apps.plus/com.google.android.apps.photos.phone.PhotosHomeActivity")
        time.sleep(2)
        if self._device(text="Later").exists:
            self._device(text="Later").click()
            time.sleep(2)

    def unlock_screen(self):
        initdut = Function()
        initdut.wake_up()
        self._device.press.menu

    @staticmethod
    def accquire_photos_from_internet():
        """
            accquire_photos_from_internet,will require curl binary
        """
        config = TestConfig()
        cfg_file = 'tests.tablet.artifactory.conf'
        cfg_arti = config.read(cfg_file, 'artifactory')
        config_handle = ConfigHandle()
        cfg_arti["location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat/', 'sys.conf')
        cfg = config.read(cfg_file, 'content_picture')
        arti = Artifactory(cfg_arti.get('location'))
        pic_name = cfg.get("name")
        file_path = arti.get(pic_name)
        g_common_obj.adb_cmd_common('push ' + file_path + ' /sdcard/Pictures/tmpfolders/email_attach.jpg')

    @staticmethod
    def refresh_sdcard():
        """
            update SDCARD img content provider
        """
        g_common_obj.adb_cmd_common(
            "shell am broadcast -a android.intent.action.MEDIA_MOUNTED -d file:///sdcard")

    def navigate_to_ondevice(self):
        """
            openNavigatorPanel to browse devices files sequence
        """
        if self._device(text="Later").exists:
            self._device(text="Later").click()
            time.sleep(2)
        self._device(description="Open navigation drawer").click()
        time.sleep(2)
        self._device(
            resourceId="com.google.android.apps.plus:id/text", text="On device").click()

    def remove_folder_within_oneimg(self):
        """
            use optionMenu button remove the whole folder sequence
        """
        if self._device(text="tmpfolders").exists:
            self._device(text="tmpfolders").click()
        else:
            self._device(scrollable=True).scroll.to(text="tmpfolders")
        self._device(text="tmpfolders").click()
        self._device(description="More options").click()
        self._device(text="Delete all").click()
        self._device.wait.update()
        self._device(text="Delete").click()
        assert not self._device(text="tmpfolders").exists
