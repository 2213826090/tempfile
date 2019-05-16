# -*- coding: utf-8 -*-
__author__ = 'yusux'
import time
from testlib.dut_init.dut_init_impl import Function
from testlib.graphics.sample_apidemo import SampleApiDemoImpl
from testlib.util.common import g_common_obj
from testlib.util.config import TestConfig


class AospAppsInit(object):
    def __init__(self):
        self.success = False
        self.d = g_common_obj.get_device()
        self.config = TestConfig()
        self.func = Function()
        cfg_file = 'tests.tablet.dut_init.conf'
        self.username = self.config.read(cfg_file, 'google_account').get("username")
        self.passwd = self.config.read(cfg_file, 'google_account').get("password")

    def check_if_app_not_first_init(self, applaunchname):
        """
        Not fit for Photos,because it's in google plus package.welcome to use check others and report defect on this methods
        :param applaunchname:
        :return:
        """
        g_common_obj.launch_app_am('com.android.settings', 'com.android.settings.Settings')
        self.d(text="Apps").click.wait()
        self.d(resourceId="com.android.settings:id/tabs").swipe.left()
        self.d(resourceId="com.android.settings:id/tabs").swipe.left()
        self.d(scrollable=True).scroll.to(textContains=applaunchname)
        self.d(textContains=applaunchname).click()
        print self.d(text="Clear data").enabled
        return self.d(text="Clear data").enabled

    def init_playstore(self):
        g_common_obj.adb_cmd("am start -S com.android.vending/.AssetBrowserActivity")
        time.sleep(3)
        if self.d(textStartsWith="Just a sec").exists or self.d(text="Couldn't sign in").exists:
            self.d.press.back()
        for i in range(30):
            if self.d(text="Accept").exists:
                self.d(text="Accept").click.wait()
            if self.d(className="android.widget.ImageButton").exists:
                break
            time.sleep(2)
        self.d(className="android.widget.ImageButton").click.wait()
        self.d(text="Settings").click.wait()
        self.d(text="Auto-update apps").click.wait()
        self.d(text="Do not auto-update apps").click.wait()
        g_common_obj.adb_cmd("am force-stop com.android.vending")

    def init_chrome(self):
        # init chrome's two options
        g_common_obj.launch_app_from_home_sc("Chrome")
        time.sleep(5)
        while self.d(text="Accept & continue").exists:
            self.d(text="Accept & continue").click.wait()
        if self.d(text="Done").exists:
            self.d(text="Done").click.wait()
        if self.d(text="No thanks").exists:
            self.d(text="No thanks").click.wait()
        if self.d(text="No Thanks").exists:
            self.d(text="No Thanks").click.wait()
        if self.d(description="More options").exists:
            self.d(description="More options").click.wait()
        if self.d(text="Cancel").exists:
            self.d(text="Cancel").click.wait()
        if self.d(resourceId="com.android.chrome:id/menu_button").exists:
            self.d(resourceId="com.android.chrome:id/menu_button").click.wait()
        if self.d(text="Settings").exists:
            self.d(text="Settings").click.wait()
        if self.d(text="Sign in to Chrome").exists:
            self.d(text="Sign in to Chrome").click.wait()
            if self.d(text="Add a Google Account").exists:
                self.d(text="Cancel").click.wait()
            else:
                self.d(text="Sign in").click.wait()
        if self.check_if_app_not_first_init("Chrome") is False:
            self.d.press.recent()
            self.d(text="Chrome", resourceId="com.android.systemui:id/activity_description").click.wait()
            if self.d(text="Basics").down(resourceId="com.android.chrome:id/header_title",
                                          textContains="@gmail.com") != None:
                self.d(text="Basics").down(resourceId="com.android.chrome:id/header_title",
                                           textContains="@gmail.com").click.wait()
            if self.d(textContains="Auto sign").right(resourceId="android:id/checkbox").checked:
                self.d(textContains="Auto sign").right(resourceId="android:id/checkbox").click()
        else:
            self.d.press.recent()
            self.d(text="Chrome", resourceId="com.android.systemui:id/activity_description").click.wait()
            pass
        if self.d(resourceId="com.android.chrome:id/menu_button").exists:
            self.d(resourceId="com.android.chrome:id/menu_button").click.wait()
        self.d(text="Privacy").click.wait()
        if self.d(textContains="Navigation").right(resourceId="android:id/checkbox").checked:
            self.d(textContains="Navigation").right(resourceId="android:id/checkbox").click.wait()
        if self.d(textContains="Search and").right(resourceId="android:id/checkbox").checked:
            self.d(textContains="Search and").right(resourceId="android:id/checkbox").click.wait()
        if self.d(textContains="Usage and crash reports").right(resourceId="android:id/checkbox").checked:
            self.d(textContains="Usage and crash reports").right(resourceId="android:id/checkbox").click.wait()
        if self.d(text="CLEAR BROWSING DATA...").exists:
            self.d(text="CLEAR BROWSING DATA...").click.wait()
        # if self.d(text="Clear browsing data").exists:
        if self.d(text="Clear browsing history").exists:
            if not self.d(text="Clear browsing history").checked:
                self.d(text="Clear browsing history").click.wait()
        if self.d(text="Clear the cache").exists:
            if not self.d(text="Clear the cache").checked:
                self.d(text="Clear the cache").click.wait()
        if self.d(textContains="Clear cookies").exists:
            if not self.d(textContains="Clear cookies").checked:
                self.d(textContains="Clear cookies").click.wait()
        if self.d(text="Clear").exists:
            self.d(text="Clear").click.wait()
        if self.d(text="Accept").exists:
            self.d(text="Accept").click.wait()
        self.d.press.back()
        self.d.press.back()

        url = "http://www.youtube.com/watch?v=OYotwvVzBm4"
        for i in range(3):
            self.d(resourceId="com.android.chrome:id/url_bar").clear_text()
            if self.d(resourceId="com.android.chrome:id/website_settings_connection_message").exists:
                self.d.press.back()
            self.d(resourceId="com.android.chrome:id/url_bar").set_text(url)
            self.d.press.enter()
        time.sleep(2)
        for i in range(10):
            if self.d(resourceId="com.android.chrome:id/url_bar", text="m.youtube.com").exists:
                g_common_obj.adb_cmd("am force-stop com.android.chrome")
                return
            if self.d(textStartsWith="Open with", resourceId="android:id/title").exists:
                self.d(textContains="Chrome").click.wait()
                if self.d(text="Always").exists:
                    self.d(text="Always").click.wait()
                    if self.d(description="Web View").exists:
                        self.d.press.back()
                time.sleep(5)
                if self.d(text="OK").exists:
                    self.d(text="OK").click.wait()
                g_common_obj.adb_cmd("am force-stop com.android.chrome")
                return
            time.sleep(3)
        g_common_obj.adb_cmd("am force-stop com.android.chrome")

    def enable_developer_option(self):
        # enable developer option
        g_common_obj.adb_cmd("am start -S com.android.settings/.Settings")
        self.d(scrollable=True).scroll.vert.to(textContains="About tablet")
        if self.d(textContains="Developer options").exists:
            return
        self.d(textContains="About tablet").click()
        for i in range(8):
            self.d(textContains="Build number").click()
        self.d.press.back()
        self.d(scrollable=True).scroll.vert.to(textContains="Developer options")
        assert self.d(textContains="Developer options").exists

    def init_hangout(self):
        g_common_obj.adb_cmd("am start com.google.android.talk/com.google.android.talk.SigningInActivity")
        time.sleep(5)
        if self.d(text="Sign in").exists:
            time.sleep(5)
            self.d.wait.update()
            # self.d(text="Sign in").click.wait()
        if self.d(text="Skip").exists:
            self.d(text="Skip").click.wait()
        elif self.d(text="Got it").exists:
            self.d(text="Got it").click.wait()

    def init_googlemap(self):
        g_common_obj.adb_cmd("am start -a android.intent.action.VIEW com.google.android.apps.maps/"
                             "\com.google.android.maps.MapsActivity -d \"geo:47.6,-122.3\"")
        if self.d(text="Accept & continue").exists:
            self.d(text="Accept & continue").click.wait()
        if self.d(text="Sign in").exists:
            time.sleep(5)
            self.d(text="Sign in").click.wait()

    def init_playmusic(self):
        g_common_obj.adb_cmd(
            "am start -S com.google.android.music/com.android.music.activitymanagement.TopLevelActivity")
        for i in range(30):
            time.sleep(2)
            if not self.d(resourceId="com.google.android.music:id/tutorial_logo").exists:
                break
        assert not self.d(resourceId="com.google.android.music:id/tutorial_logo").exists
        if self.d(text="Skip").exists:
            self.d(text="Skip").click.wait()
        if self.d(text="Use Standard").exists:
            self.d(text="Use Standard").click.wait()
        if self.d(text="Done").exists:
            self.d(text="Done").click.wait()
        if self.d(resourceId="com.google.android.music:id/play_drawer_list").exists:
            self.d.press.back()
        if self.d(text="Got it").exists:
            self.d(text="Got it").click.wait()

    def keep_awake(self):
        """# keep screen awake"""
        if g_common_obj.adb_cmd("dumpsys power | grep 'mStayOn=true'") == 0:
            if g_common_obj.adb_cmd("dumpsys power | grep 'mScreenOffTimeoutSetting=1800000'") == 0:
                return
        g_common_obj.adb_cmd("am start -S com.android.settings/.Settings")
        self.d(scrollable=True).scroll.vert.to(textContains="Developer options")
        if self.d(textContains="Developer options").exists:
            self.d(textContains="Developer options").click.wait()
        if self.d(text="Stay awake").right(resourceId="android:id/checkbox") is not None:
            if not self.d(text="Stay awake").right(resourceId="android:id/checkbox").checked:
                self.d(text="Stay awake").right(resourceId="android:id/checkbox").click()
        else:
            if not self.d(text="Stay awake").right(resourceId="android:id/switchWidget").checked:
                self.d(text="Stay awake").right(resourceId="android:id/switchWidget").click()
        self.d.press.back()
        self.d(scrollable=True).scroll.vert.to(text="Display")
        self.d(text="Display").click.wait()
        if not self.d(text="After 30 minutes of inactivity").exists:
            self.d(text="Sleep").click.wait()
            self.d(text="30 minutes").click.wait()
        assert self.d(text="After 30 minutes of inactivity").exists


    def add_google_account(self):
        """
            add_google_account
        """
        for i in range(5):
            try:
                self.func.add_google_account(self.username, self.passwd)
                break
            except Exception as e:
                print e

    def init_dut_squeence(self):
        """
            seqeueence of predebugplan
        """
        self.__init__()
        SampleApiDemoImpl().unlock()
        self.func.push_uiautomator_jar()
        g_common_obj.set_vertical_screen()
        self.enable_developer_option()
        self.keep_awake()
        self.func.wake_up()
        self.add_google_account()
        self.init_playstore()
        self.func.init_camera()
        self.init_chrome()
        self.func.init_photo()
        self.init_googlemap()
        self.init_hangout()
        self.init_playmusic()
        self.func.init_youtube()