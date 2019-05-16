# -*- coding: utf-8 -*-
"""
Created on 2014-11-5

@author: yusux
"""
from testlib.util.common import g_common_obj
from testlib.util.config import TestConfig
from testlib.util.repo import Artifactory
from testlib.graphics.tools import ConfigHandle
from testlib.systemui.systemui_impl import SystemUI
from testlib.dut_init.dut_init_impl import Function
import time


class ZEDGEImpl(object):
    is_launched_first_time = False

    def __init__(self):
        self._device = g_common_obj.get_device()

    @staticmethod
    def wait_exist():
        """ wait until the ui object exist """

        def _wait(uiobj, timeout=10):
            return uiobj.wait("exists", timeout * 1000)

        return _wait

    def init_zedge(self):
        g_common_obj.launch_app_am("com.android.settings", "com.android.settings.Settings")
        time.sleep(3)
        if self._device(text="Apps").exists:
            self._device(text="Apps").click()
            self._device(scrollable=True).scroll.to(text="Zedge", resourceId="com.android.settings:id/app_name")
            if self._device(text="Zedge", resourceId="com.android.settings:id/app_name").exists:
                self._device(text="Zedge", resourceId="com.android.settings:id/app_name").click()
            if self._device(text="Clear data").enabled:
                return self.is_launched_first_time is False
            else:
                return self.is_launched_first_time is True
        print "is_Zedge_is_first_launch is :," + str(self.is_launched_first_time)

    def launch_zedge(self):
        if self.init_zedge() is True:
            g_common_obj.launch_app_from_home_sc("Zedge")
            time.sleep(3)
            if self._device(text="Terms of Service").exists is True:
                self._device(text="Continue", className="android.widget.Button").click()
        else:
            g_common_obj.launch_app_am("net.zedge.android", "net.zedge.android.activity.ControllerActivity")

    @staticmethod
    def install_zedge():
        """
            install from Artifactory
        """
        config = TestConfig()
        cfg_file = 'tests.tablet.artifactory.conf'
        cfg_arti = config.read(cfg_file, 'artifactory')
        config_handle = ConfigHandle()
        cfg_arti["location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat/', 'sys.conf')
        cfg = config.read(cfg_file, 'content_zedge')
        arti = Artifactory(cfg_arti.get('location'))
        apk_name = cfg.get("name")
        file_path = arti.get(apk_name)
        result = config_handle.check_apps("net.zedge.android")
        if result == 0:
            g_common_obj.adb_cmd_common('install ' + str(file_path))

    def remove_promote_ad(self):
        while self._device(resourceId="net.zedge.android:id/ad_container").exists:
            Function().push_uiautomator_jar()
            if not self._device(resourceId="net.zedge.android:id/ad_container").exists:
                break
            else:
                self._device(resourceId="net.zedge.android:id/close_button").click()
            break

    def set_static_wallpapper(self):
        """
            download_and_set_wallpaper
        """
        cmd = "logcat -d AcitivityManager:I net.zedge.android:W *:S -v time"

        errormessage = "Download is failure or RPCServer timeout,check your network and retry again"
        SystemUI().unlock_screen()
        self.launch_zedge()
        time.sleep(17)
        if self._device(resourceId="net.zedge.android:id/splash_screen_logo").exists:
            time.sleep(3)
        # if self._device(resourceId="net.zedge.android:id/ad_container").exists:
        # self.remove_promote_ad()
        # time.sleep(5)
        if self._device(text="Rate ZEDGE").exists:
            self._device(text="Don't ask me again").click.wait()

        if self._device(resourceId="net.zedge.android:id/menu_search").exists:
            self._device(resourceId="net.zedge.android:id/menu_search").click.wait()
            self._device(resourceId="net.zedge.android:id/abs__search_src_text").set_text("punisher")
            self._device.press("enter")
            time.sleep(3)
            self._device(resourceId="net.zedge.android:id/list_item_title", text="Wallpapers").click.wait()
            self._device(resourceId="net.zedge.android:id/item_list_grid").child(index='0').click()
        else:
            if self.slide_out_drawer() is True:
                self._device(resourceId="net.zedge.android:id/menu_item_text", text="Wallpapers").click.wait()
                self._device(resourceId="net.zedge.android:id/item_list_grid").child().click()
        if self._device(resourceId="net.zedge.android:id/ad_container").exists:
            self.remove_promote_ad()
        if self._device(text="Download").exists and not self._device(
                resourceId="net.zedge.android:id/connection_error_layout").exists:
            self._device(text="Download").click()
            time.sleep(3)
            # if self._device(resourceId="net.zedge.android:id/ad_container").exists:
            #     self.remove_promote_ad()
            assert not self._device(text="Download").exists, errormessage
        if self._device(text="Set").exists:
            self._device(text="Set").click.wait()

        grablogcat = g_common_obj.adb_cmd_capture_msg(cmd).strip()
        assert not grablogcat.__contains__("io.ErrnoException") or grablogcat.__contains__(
            "OutOfMemoryError") or grablogcat.__contains__("dirty regions requested") or grablogcat.__contains__(
            "rsGetAllocation, failed to find") or grablogcat.__contains__("Fatal signal 11")

    def set_live_wallpaper(self):
        """
            Download and set livewallpapper
        """
        # errormessage = "due network lag,livewppAPK was not finishing download,plz do try again"
        SystemUI().unlock_screen()
        g_common_obj.launch_app_am("net.zedge.android", "net.zedge.android.activity.ControllerActivity")
        if self._device(resourceId="net.zedge.android:id/splash_screen_logo").exists:
            time.sleep(3)
        if self._device(text="Rate ZEDGE").exists:
            self._device(text="Don't ask me again").click.wait()
        if self._device(resourceId="net.zedge.android:id/menu_search").exists:
            self._device(resourceId="net.zedge.android:id/menu_search").click.wait()
            self._device(resourceId="net.zedge.android:id/abs__search_src_text").set_text("weed")
            self._device.press("enter")
            time.sleep(3)
            self._device(resourceId="net.zedge.android:id/list_item_title", text="Live Wallpapers").click.wait()
            self._device(scrollable=True, resourceId="net.zedge.android:id/item_list_grid").scroll.to(text="Weed")
            self._device(text="Weed").click.wait()
            if self._device(text="Install", resourceId="net.zedge.android:id/button").exists:
                self._device(text="Install", className="android.widget.Button").click.wait()
                time.sleep(3)
                if self._device(text="INSTALL", resourceId="com.android.vending:id/buy_button").exists:
                    self._device(text="INSTALL", resourceId="com.android.vending:id/buy_button").click.wait()
                if self._device(text="ACCEPT", resourceId="com.android.vending:id/continue_button_label").exists:
                    self._device(text="ACCEPT", resourceId="com.android.vending:id/continue_button_label").click.wait()
                    time.sleep(25)
                self._device(description="Navigate up", className="android.widget.ImageButton").click()
            time.sleep(5)
            # assert self._device(textContains="Set", className="android.widget.Button").exists, errormessage
            self._device(textContains="Set", className="android.widget.Button").click.wait()
            self._device(scrollable=True).scroll.to(
                text="Weed Live Wallpaper", resourceId="com.android.wallpaper.livepicker:id/title")
            self._device(text="Weed Live Wallpaper", resourceId="com.android.wallpaper.livepicker:id/title").click()
            assert self._device(text="Set wallpaper").exists
            self._device(text="Set wallpaper").click()

    def slide_out_drawer(self):
        """
            slide_out_drawer
        """
        if self._device(resourceId="net.zedge.android:id/menu_fragment").exists:
            return True
        else:
            self._device.swipe(0, 650, 555, 650, steps=33)
            return True

    def set_ringtone(self):
        """
            setringtone
        """
        # errormessage = "due network lag,livewppAPK was not finishing download,plz do try again"
        SystemUI().unlock_screen()
        g_common_obj.launch_app_am("net.zedge.android", "net.zedge.android.activity.ControllerActivity")
        if self._device(resourceId="net.zedge.android:id/splash_screen_logo").exists:
            time.sleep(3)
        if self._device(text="Rate ZEDGE").exists:
            self._device(text="Don't ask me again").click.wait()
        if self._device(resourceId="net.zedge.android:id/menu_search").exists:
            self._device(resourceId="net.zedge.android:id/menu_search").click.wait()
            self._device(resourceId="net.zedge.android:id/abs__search_src_text").set_text("banshee")
            self._device.press("enter")
        self._device(resourceId="net.zedge.android:id/list_item_title", text="Ringtones").click.wait()
        self._device(scrollable=True, resourceId="net.zedge.android:id/item_list_grid").scroll.to(text="Banshee TV II")
        self._device(text="Banshee TV II").click.wait()
        if self._device(text="Download").exists:
            self._device(text="Download").click.wait()
        time.sleep(5)
        if self._device(text="Set").exists:
            self._device(text="Set").click.wait()
        if self._device(text="Set as..").exists:
            self._device(text="Notification ringtone").click()
        g_common_obj.launch_app_am("com.android.settings", "com.android.settings.Settings")
        if self._device(text="Sound & notification").exists:
            self._device(text="Sound & notification").click.wait()
        assert self._device(text="Banshee TV II").exists

def main():
    """
        testallfunction
    """
    # ZEDGEImpl().install_zedge()
    # ZEDGEImpl().slide_out_drawer()
    # ZEDGEImpl().set_static_wallpapper()
    # ZEDGEImpl().set_live_wallpaper()
    ZEDGEImpl().set_ringtone()
    pass


if __name__ == '__main__':
    main()