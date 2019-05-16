import os
import time
from testlib.util.common import g_common_obj
from testlib.util.device import TestDevice


class Url:
    """
    Url definition
    """

    def __init__(self, cfg):
        """Get value from config"""

        self.url1 = cfg.get("url1")
        self.url2 = cfg.get("url2")
        self.url3 = cfg.get("url3")
        self.index1 = cfg.get("index1")
        self.index2 = cfg.get("index2")
        self.index3 = cfg.get("index3")


class BrowserSettingImpl:
    """
    Implements Browser UI actions.

    """

    browser_pkg_name = "com.ksmobile.cb"
    browser_activity_name = ".Main"

    class Locator(object):
        """
        Helper class to locate UI object on screen
        """

        def __init__(self, device):
            """ Init environment """
            self.d = device

        @property
        def browser_logo(self):
            """ Browser logo """
            return self.d(
                resourceId="com.ksmobile.cb:id/address_bar_search_logo")

        @property
        def package_name(self):
            """ Browser package name """
            return self.d(packageName="com.ksmobile.cb")

        @property
        def toolbar_home(self):
            """ Browser home UI toolbar """
            return self.d(resourceId="com.ksmobile.cb:id/toolbar_home")

        @property
        def search(self):
            """ Browser UI search """
            return self.d(text="Search or type a URL")

        @property
        def btn_refresh(self):
            """ Browser refresh button """
            return self.d(resourceId='com.ksmobile.cb:id/stop_refresh_btn')

        def addr_title(self, title):
            """ Browser address bar title """
            return self.d(
                text=title, resourceId='com.ksmobile.cb:id/address_bar_hint')

        @property
        def addr_bar(self):
            """ Browser address bar """
            return self.d(
                resourceId='com.ksmobile.cb:id/address_bar_hint')
    #------------------------ end of class Locator ----------------------

    def __init__(self, cfg):
        """Init environment"""
        self.d = g_common_obj.get_device()
        self.cfg = cfg
        self._locator = BrowserSettingImpl.Locator(self.d)

    def launch_by_home(self):
        """Launch browser by home"""
        print "[INFO] Launch browser by home"
        self.d.press.home()
        g_common_obj.launch_app_from_home_sc("CM Browser", "Apps")
        if self._locator.browser_logo.exists:
            self.d.press.back()
        assert self._locator.package_name.exists, True

    def open_tab(self):
        """Open a tab in browser. (Work with browser_close_all_tabs)"""
        print "[INFO] Clear the input history"
        self._locator.toolbar_home.click()
        assert self._locator.search.exists, True

    def close_tabs(self):
        """Close all the tabs in browser"""
        pass

    def __check_wepage_title(self, title, status=True):
        """Check the status of web page in browser"""
        print "[INFO] Check if the status of webpage is right"
        for i in range(5):
            if self._locator.btn_refresh.exists:
                break
            time.sleep(4)
        assert str(self._locator.addr_title(title).exists), str(status)

    def __open_url(self, address):
        """Open website in browser"""
        print "[INFO] Open in browser"
        self._locator.addr_bar.set_text(address)
        self.d.press("enter")
        time.sleep(10)

    def check_content(self, content, status=True):
        """Verify website is opened in browser"""
        print "[INFO]: Check if the status of content is %s" % content
        assert str(self.d(description=content).exists), str(status)

    def open_and_check(self, web, num):
        """ Open URL and have a check"""
        if num.number == "one":
            self.open_tab()
            self.__open_url(web.url1)
            self.__check_wepage_title(web.index1)
        else:
            self.open_tab()
            self.__open_url(web.url1)
            self.__check_wepage_title(web.index1)
            self.open_tab()
            self.__open_url(web.url2)
            self.__check_wepage_title(web.index2)
            self.open_tab()
            self.__open_url(web.url3)
            self.__check_wepage_title(web.index3)
