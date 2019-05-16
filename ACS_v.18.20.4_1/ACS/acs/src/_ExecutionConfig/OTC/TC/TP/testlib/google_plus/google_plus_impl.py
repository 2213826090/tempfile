"""
@summary: Test google plus
@since: 10/1/2014
@author: Mingmin Liu (mingminx.liu@intel.com)
"""
from testlib.util.common import g_common_obj
import time
import os


class GooglePlusImpl:
    """
        @summary: class for google plus application Home UI
    """
    googleplus_package = "com.google.android.apps.plus"
    googleplus_activity = ".phone.HomeActivity"
    setting_package = "com.android.settings"
    setting_activity = ".Settings"

#--------- begin locator -------------
    class Locator(object):
        """
            Helper for locator UI Object
        """

        def __init__(self, device):
            self.d = device

        @property
        def btn_next(self):
            """ UI button next """
            return self.d(text = "NEXT")

        @property
        def btn_get_start(self):
            """ UI button get start """
            return self.d(text = "Get Started")

        @property
        def btn_more_option(self):
            """ UI button more option """
            return self.d(description = "More options")

        @property
        def btn_refresh(self):
            """ UI button refresh """
            return self.d(text = "Refresh")

        @property
        def btn_wifi(self):
            """ UI button wifi """
            return self.d(textMatches="Wi.*Fi")

        @property
        def btn_account(self):
            """ UI button account """
            return self.d(resourceId = "com.google.android.apps.plus:id/account_layout")

        @property
        def btn_Connected(self):
            """ UI button Connected """
            return self.d(text = "Connected")

        @property
        def layout_grid(self):
            """ UI layout grid """
            return self.d(resourceId = "com.google.android.apps.plus:id/grid")

    def __init__ (self, cfg = None):
        self.d = g_common_obj.get_device()
        self.cfg = cfg
        self._locator = GooglePlusImpl.Locator(self.d)

    def set_orientation_n(self):
        """
        @summary: set orientation as n
        """
        self.d.orientation = "n"

    def launch_from_am(self,app_name):
        """
            Launch google plus from am
        """
        print "[INFO] Launch app from am"
        if (app_name == "googleplus") :
            g_common_obj.launch_app_am(GooglePlusImpl.googleplus_package, \
                                       GooglePlusImpl.googleplus_activity)
        elif (app_name == "settings") :
            g_common_obj.launch_app_am(GooglePlusImpl.setting_package, \
                                       GooglePlusImpl.setting_activity)
        time.sleep(5)

    def stop_from_am(self,app_name):
        """
            Stop Chrome from am
        """
        print "[INFO] Stop Chrome from am"
        if (app_name == "googleplus") :
            g_common_obj.launch_app_am(GooglePlusImpl.googleplus_package)
        elif (app_name == "settings") :
            g_common_obj.launch_app_am(GooglePlusImpl.setting_package)

    @staticmethod
    def startApp():
        """
        Skip Google Plus first launch screen
        """
        d = g_common_obj.get_device()
        g_common_obj.launch_app_from_home_sc("Google+")
        time.sleep(10)
        while d(resourceId="com.google.android.apps.plus:id/warm_welcome_button").exists:
            d(resourceId="com.google.android.apps.plus:id/warm_welcome_button").click()
            d.wait.update()
        g_common_obj.back_home()

    def checkConnect(self):
        self._locator.btn_wifi.click.wait()
        time.sleep(5)
        assert self._locator.btn_Connected.exists,"ERROR:text not found!"
        self.d.press.back()

    def quit_app(self):
        for i in range(3):
            self.d.press.back()

    def google_plus_view(self):
        """
        This test used to test google plus function.
        The test case spec is following:
        1. Launch the "google+" .
        """
        self.launch_from_am("settings")
        self.checkConnect()
        self.quit_app()
        self.launch_from_am("googleplus")
        time.sleep(20)
        if self._locator.btn_account.exists:
            self._locator.btn_account.click.wait()
            time.sleep(10)
        if self._locator.btn_get_start.exists:
            self._locator.btn_get_start.click.wait()
            time.sleep(20)
        if self._locator.btn_next.exists:
            print "[INFO] click next button"
            self._locator.btn_next.click.wait()
            time.sleep(3)
        if self._locator.btn_next.exists:
            print "[INFO] click next button 2"
            self._locator.btn_next.click.wait()
            time.sleep(3)
        if self._locator.btn_more_option.exists:
            self._locator.btn_more_option.click.wait()
            self._locator.btn_refresh.click.wait()
        time.sleep(5)
        self._locator.layout_grid.scroll.vert()
        self.quit_app()