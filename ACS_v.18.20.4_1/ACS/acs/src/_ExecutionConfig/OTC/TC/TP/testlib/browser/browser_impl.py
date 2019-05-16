# Copyright (C) 2014  Yi, GraceX <gracex.yi@intel.com>
# Intel Corporation All Rights Reserved.

# The source code contained or described herein and
# all documents related to the source code ("Material") are owned by
# Intel Corporation or its suppliers or licensors.

# Title to the Material remains with Intel Corporation or
# its suppliers and licensors.
# The Material contains trade secrets and proprietary and
# confidential information of Intel or its suppliers and licensors.
# The Material is protected by worldwide copyright and
# trade secret laws and treaty provisions.
# No part of the Material may be used, copied, reproduced, modified,
# published, uploaded, posted, transmitted, distributed
# or disclosed in any way without Intel's prior express written permission.
# No license under any patent, copyright, trade secret or
# other intellectual property right is granted to
# or conferred upon you by disclosure or delivery of the Materials,
# either expressly, by implication, inducement, estoppel or otherwise.

# Any license under such intellectual property rights must be express
# and approved by Intel in writing.

"""
@summary: This file implements for browser
@since: 07/14/2014
@author: Grace Yi (gracex.yi@intel.com)
"""
import time
from testlib.util.common import g_common_obj
from testlib.graphics.common import osversion

class BrowserImpl(object):
    """
    Chrome Test Impl Class
    """ 
#--------- begin locator -------------
    class Locator(object):
        """
            Helper for locator UI Object
        """

        def __init__(self, device):
            self.d = device

        @property
        def btn_apps(self):
            """ UI button apps """
            return self.d(description="Apps")

        @property
        def btn_inactive(self):
            """ UI button inactive """
            return self.d(resourceId=\
                "com.google.android.googlequicksearchbox:id/inactive")

        @property
        def btn_chrome_accept(self):
            """ UI button accept in Welcome popup """
            return self.d(textContains="Accept & continue", \
                resourceId="com.android.chrome:id/terms_accept")

        @property
        def btn_chrome_setup(self):
            """ UI button setup title in Welcome pop up """
            return self.d(textContains="Set up Chrome", \
                resourceId="com.android.chrome:id/title")

        @property
        def btn_chrome_account_skip(self):
            """ UI button no thanks in signin pop up """
            return self.d(textContains="No thanks", \
                resourceId="com.android.chrome:id/negative_button")

        @property
        def btn_chrome_account_done(self):
            """ UI button done in signin pop up """
            return self.d(textContains="Done", \
                resourceId="com.android.chrome:id/positive_button")

        @property
        def btn_tab_close(self):
            """ UI button browser tab close """
            return self.d(resourceId="com.android.browser:id/close")

        @property
        def btn_tab_stop(self):
            """ UI button browser tab stop loading """
            return self.d(resourceId="com.android.browser:id/stop")

        @property
        def btn_new_tab(self):
            """ UI button browser new tab """
            return self.d(description="New tab")

        @property
        def btn_url_bar(self):
            """ UI button browser url bar """
            return self.d(resourceId="com.android.chrome:id/url_bar")

        @property
        def btn_app_name(self):
            """ UI button browser app name """
            return self.d(text="Chrome")

        @property
        def btn_clear_cache(self):
            """ UI button browser clear cache """
            return self.d(\
                resourceId="com.android.settings:id/clear_cache_button", \
                text="Clear cache")

        @property
        def btn_force_stop(self):
            """ UI button browser force stop """
            version_array = osversion.get_android_version()
            androidversion = version_array[0]
            if androidversion == 6:
                return self.d(text="Force stop")
            return self.d(text="FORCE STOP")

        @property
        def btn_ok(self):
            """ UI button ok """
            return self.d(resourceId="android:id/button1", text="OK")

        @property
        def btn_web_notavailable(self):
            """ UI button web is not available """
            return self.d(descriptionContains="is not available")

        @property
        def btn_web_timeout(self):
            """ UI button web is time out """
            return self.d(descriptionContains="Request Timeout")

        @property
        def btn_infobar_close(self):
            """ UI button infobar close """
            return self.d(\
                resourceId="com.android.browser:id/infobar_close_button")

        @property
        def btn_chrome_option(self):
            """ UI button more option """
            return self.d(resourceId="com.android.chrome:id/menu_button")


        @property
        def btn_chrome_refresh(self):
            """ UI button chrome refresh """
            return self.d(resourceId="com.android.chrome:id/refresh_button")

        @property
        def btn_chrome_setting(self):
            """ UI button more option """
            return self.d(\
                resourceId="com.android.chrome:id/menu_item_text", \
                text="Settings")

        @property
        def btn_chrome_usage_true(self):
            """ UI button more option """
            return self.d(textContains="Usage").right(\
                resourceId="android:id/checkbox", checked="true")

        @property
        def btn_chrome_usage_false(self):
            """ UI button more option """
            return self.d(textContains="Usage").right(\
                resourceId="android:id/checkbox", checked="false")

        @property
        def btn_browser_bookmark(self):
            """ UI button bookmark """
            return self.d(resourceId="com.android.chrome:id/bookmark_button")

        @property
        def btn_browser_addbookmark(self):
            """ UI button add bookmark """
            return self.d(\
                resourceId="com.android.chrome:id/bookmark_action_title", \
                text="Add bookmark")

        @property
        def btn_browser_editbookmark(self):
            """ UI button edit bookmark """
            return self.d(\
                resourceId="com.android.chrome:id/bookmark_action_title", \
                text="Edit bookmark")

        @property
        def btn_browser_bookmarktitle(self):
            """ UI button bookmark title """
            return self.d(\
                resourceId="com.android.chrome:id/bookmark_title_input")

        @property
        def btn_browser_bookmarksave(self):
            """ UI button save bookmark """
            return self.d(resourceId="com.android.chrome:id/ok")

        @property
        def btn_browser_bookmarkremove(self):
            """ UI button remove bookmark """
            return self.d(resourceId="com.android.chrome:id/remove")

        @property
        def btn_browser_tab0(self):
            """ UI button tab 0 """
            return self.d(resourceId="com.android.chrome:id/tab_title", \
                instance="0")

        @property
        def btn_browser_tab1(self):
            """ UI button tab 1 """
            return self.d(resourceId="com.android.chrome:id/tab_title", \
                instance="1")

        @property
        def btn_browser_newtab(self):
            """ UI button new tab """
            return self.d(resourceId="com.android.chrome:id/new_tab_button")

        @property
        def btn_browser_menu(self):
            """ UI button menu """
            return self.d(resourceId="com.android.chrome:id/menu_button")

        @property
        def btn_browser_bookmarks_list(self):
            """ UI button menu """
            return self.d(resourceId="com.android.chrome:id/menu_item_text", \
                text="Bookmarks")

        def bookmark_title(self, title):
            """ UI bookmark which contain title """
            if self.d(resourceId="com.android.chrome:id/bookmarks_list_view").\
            child_by_text(title, allow_scroll_search=True) != None:
                return self.d(textContains=title, instance="0")
            return None

        @property
        def bookmark_fist_title(self):
            """ UI bookmark the first bookmark in bookmark list """
            return self.d(\
                resourceId="com.android.chrome:id/bookmarks_list_view")\
            .child(className="android.widget.TextView", instance="0")

        @property
        def btn_browser_bookmarkdelete(self):
            """ UI button remove bookmark """
            return self.d(resourceId="android:id/title", text="Delete bookmark")

        @property
        def btn_refresh(self):
            """ UI button Refresh """
            return self.d(description="Refresh page")

        @property
        def btn_stop_page_loading(self):
            """ UI button Refresh """
            return self.d(description="Stop page loading")

    packagename = "com.android.chrome"
    activityname = "com.google.android.apps.chrome.Main"
    info_packagename = "com.android.settings"
    info_activity = "com.android.settings.applications.InstalledAppDetails"
    info_intent = "android.intent.action.MAIN"
    view_intent = "android.intent.action.VIEW"

    def __init__ (self, cfg={}):
        self.d = g_common_obj.get_device()
        self._locator = BrowserImpl.Locator(self.d)
        self.cfg = cfg

    @staticmethod
    def launch_from_am():
        """
            Launch Browser from am
        """
        print "[INFO] Launch Chrome from am"
        g_common_obj.launch_app_am(BrowserImpl.packagename, \
            BrowserImpl.activityname)
        time.sleep(2)

    def launchChrome(self):
        self.launch_from_am()
        if self.d(resourceId="com.android.chrome:id/terms_accept").exists:
            self.d(resourceId="com.android.chrome:id/terms_accept").click()
        time.sleep(2)
        if self.d(text="No thanks").exists:
            self.d(text="No thanks").click()

    def launch_app_from_home_sc(self, appname):
        """
            restrute for there is no switch widget/apps in app screen
        """
        iffind = False
        self.d.press.home()
        time.sleep(2)
        self._locator.btn_apps.click()
        time.sleep(2)
        count = int(self._locator.btn_inactive.count) + 1
        for i in range(0, count * 2):
            time.sleep(2)
            if self.d(text=appname).exists:
                self.d(text=appname).click()
                iffind = True
                break
            if i < count:
                self.d(scrollable=True).scroll.horiz()
            else:
                self.d(scrollable=True).scroll.horiz.backward()
        assert iffind == True

    @staticmethod
    def stop_from_am():
        """
            Stop Browser from am
        """
        print "[INFO] Stop Chrome from am"
        g_common_obj.stop_app_am(BrowserImpl.packagename)

    @staticmethod
    def startApp():
        """
        Skip the welcome screen while launch app for the first time
        """
        # init chrome's two options
        g_common_obj.launch_app_from_home_sc("Chrome")
        time.sleep(10)
        d = g_common_obj.get_device()
        time.sleep(5)
        while d(text="Accept & continue").exists:
            d(text="Accept & continue").click.wait()
        if d(text="Done").exists:
            d(text="Done").click.wait()
        if d(text="No thanks").exists:
            d(text="No thanks").click.wait()
        if d(text="No Thanks").exists:
            d(text="No Thanks").click.wait()
        if d(description="More options").exists:
            d(description="More options").click.wait()
        if d(text="Settings").exists:
            d(text="Settings").click.wait()
        if d(text="Sign in to Chrome").exists:
            d(text="Sign in to Chrome").click.wait()
            if d(text="Add a Google Account").exists:
                d(text="Cancel").click.wait()
            else:
                d(text="Sign in").click.wait()
        if d(text="Basics").exists:
            if d(text="Basics").\
            down(resourceId="com.android.chrome:id/header_title", \
                textContains="@gmail.com") != None:
                d(text="Basics").down(
                    resourceId="com.android.chrome:id/header_title", \
                    textContains="@gmail.com").click.wait()
        if d(textContains="Auto sign").exists:
            if d(textContains="Auto sign").\
            right(resourceId="android:id/checkbox").checked:
                d(textContains="Auto sign").\
                right(resourceId="android:id/checkbox").click()
                d.press.back()
        if d(text="Privacy").exists:
            d(text="Privacy").click.wait()
        if d(textContains="Navigation").exists:
            if d(textContains="Navigation").\
            right(resourceId="android:id/checkbox").checked:
                d(textContains="Navigation").\
                right(resourceId="android:id/checkbox").click.wait()
        if d(textContains="Search and").exists:
            if d(textContains="Search and").\
            right(resourceId="android:id/checkbox").checked:
                d(textContains="Search and").\
                right(resourceId="android:id/checkbox").click.wait()
        if d(textContains="Usage and crash").exists:
            if d(textContains="Usage and crash").\
            right(resourceId="android:id/checkbox").checked:
                d(textContains="Usage and crash").\
                right(resourceId="android:id/checkbox").click.wait()
        if d(text="Clear browsing data").exists:
            d(text="Clear browsing data").click.wait()
        if not d(text="Clear browsing history").checked:
            d(text="Clear browsing history").click.wait()
        if not d(text="Clear the cache").checked:
            d(text="Clear the cache").click.wait()
        if not d(textContains="Clear cookies").checked:
            d(textContains="Clear cookies").click.wait()
        if d(text="Clear").exists:
            d(text="Clear").click.wait()
        if d(text="Accept").exists:
            d(text="Accept").click.wait()
        g_common_obj.back_home()

    @staticmethod
    def launch_browser_info_from_am():
        """
        @summary: launch browser app info
        """
        cmd = "am start -a " + BrowserImpl.info_intent + \
        " -n " + BrowserImpl.info_packagename + "/" + \
        BrowserImpl.info_activity + " -d " + BrowserImpl.packagename
        g_common_obj.adb_cmd(cmd)

    @staticmethod
    def focus_window(package):
        """
        @summary: check focus package
        """
        cmd = "dumpsys window|grep mCurrentFocus"
        message = g_common_obj.adb_cmd_capture_msg(cmd)
        print "[INFO] Current Focus window is %s" % message
        if package in message:
            return True
        return False

    def browser_setup(self):
        """
        @summary: close all browser tab and new a empty one
        """
        self.d.watcher("SKIP_WELCOME").when(text="No Thanks").click(text="No Thanks")
        g_common_obj.back_home()
        self.launch_app_from_home_sc("Chrome")
        time.sleep(3)
        if self._locator.btn_chrome_accept.exists:
            self._locator.btn_chrome_accept.click()
            time.sleep(2)
        if self._locator.btn_chrome_setup.exists:
            self._locator.btn_chrome_account_skip.click()
            time.sleep(2)
        if self._locator.btn_chrome_account_done.exists:
            self._locator.btn_chrome_account_done.click()
            time.sleep(2)
        self.swipe_screen_down()
        assert self._locator.btn_url_bar.exists or \
        self._locator.btn_new_tab.exists, \
        "[FAILURE] Skip Welcome dialog failed!"
        self.open_new_tab()
        # self.skip_crash_report()
        self.d.watcher("SKIP_WELCOME").remove()

    def open_new_tab(self, timeout=600):
        """
        @summary: closed all tab and launch a new one
        """
        self.swipe_screen_down()
        count = self.get_tab_count()
        s_time = time.time()
        info = self.d.info
        displayWidth = int(info["displayWidth"])
        displayHeight = int(info["displayHeight"])
        while count > 0:
            if count > 2:
                print "[Debug] count > 2"
                vert_x = self.cfg.get("third_x")
                hori_y = self.cfg.get("third_y")
            elif count == 2:
                print "[Debug] count = 2"
                vert_x = self.cfg.get("second_x")
                hori_y = self.cfg.get("second_y")
            else:
                print "[Debug] count < 2"
                vert_x = self.cfg.get("first_x")
                hori_y = self.cfg.get("first_y")
            vert_x = int(vert_x) * displayWidth / int(self.cfg.get("display_x"))
            hori_y = int(hori_y) * displayHeight / int(self.cfg.get("display_y"))
            self.d.click(vert_x, hori_y)
            count = self.get_tab_count()
            last_time = int(time.time() - s_time)
            print "[Debug] Time last [%d]" % last_time
            assert last_time < timeout, \
            "[FAILURE] Closed all tab failed!"
        assert self.d(description="New tab").exists, \
        "[FAILURE] The New tab icon does not come out "
        self._locator.btn_new_tab.click.wait(timeout=3000)

    def open_tab(self, timeout=600):
        """
        @summary: launch a new one
        """
        self.swipe_screen_down()
        count = x_count = self.get_tab_count()
        s_time = time.time()
        while count >= x_count:
            if count > 2:
                print "[Debug] count > 2"
                vert_x = self.cfg.get("third_newtab_x")
                hori_y = self.cfg.get("third_newtab_y")
            elif count == 2:
                print "[Debug] count = 2"
                vert_x = self.cfg.get("second_newtab_x")
                hori_y = self.cfg.get("second_newtab_y")
            else:
                print "[Debug] count < 2"
                vert_x = self.cfg.get("first_newtab_x")
                hori_y = self.cfg.get("first_newtab_y")
            self.d.click(int(vert_x), int(hori_y))
            x_count = self.get_tab_count()
            last_time = int(time.time() - s_time)
            print "[Debug] Time last [%d]" % last_time
            assert last_time < timeout, \
            "[FAILURE] Closed all tab failed!"

    @staticmethod
    def get_tab_count():
        """
        @summary: get tab count
        """
        time.sleep(3)
        cmd = "top -n 1|grep com.android.chrome:sandboxed_process"
        message = g_common_obj.adb_cmd_capture_msg(cmd)
        count = message.count("sandboxed_process")
        print "[INFO] There is [%d] tab here" % count
        return count

    def open_website(self, url):
        """
        @summary: open website
        @paramater:
            url : the web address
        @return: None
        """
        self.swipe_screen_down()
        assert self._locator.btn_url_bar.exists, \
        "[FAILURE] Launched Chrome Failed"
        self._locator.btn_url_bar.set_text(url)
        self.d.press("enter")

    def open_website_phone(self, url):
        """
        @summary: open website by phone chrome
        @paramater:
            url : the web address
        @return: None
        """
        cmd = "am start -a " + BrowserImpl.view_intent + \
        " -d " + url
        g_common_obj.adb_cmd(cmd)

    def clear_data(self):
        """
        @summary: clean browser data
        """

        self.launch_browser_info_from_am()
        time.sleep(10)
        assert self._locator.btn_app_name.exists, \
        "ERROR:Launch appinfo of chrome failed"
        if self._locator.btn_clear_cache.exists:
            self._locator.btn_clear_cache.click()
        if self._locator.btn_force_stop.exists:
            self._locator.btn_force_stop.click.wait()
        if self._locator.btn_ok.exists:
            self._locator.btn_ok.click()
        self.d.press.home()

    def web_check(self, key, timeout=None):
        """
        @summary: check if open a website successfully
        @return: last time
        """
        if timeout == None:
            timeout = self.cfg.get("url_timeout")
        time_start = time.time()
        time.sleep(2)
        time_last = time.time()
        print("INFO: Start at %d" % time_start)
        while True:
            time_last = time.time()
            assert not self._locator.btn_web_notavailable\
            .exists, "ERROR:The webpage is not available"
            assert not self._locator.btn_web_timeout\
            .exists, "ERROR:The webpage is timeout"
            assert int(time_last - time_start) <= int(timeout), \
            "ERROR: The webpage does not loading successfully in %d s" \
            % int(timeout)
            if self._locator.btn_infobar_close.exists:
                self._locator.btn_infobar_close.click()
            if self.d(descriptionMatches=key, \
                className="android.view.View").exists or \
            self._locator.btn_refresh.exists:
                time.sleep(2)
                break
            time.sleep(10)
        print("INFO: Time last %d" % (time_last - time_start))
        return time_last - time_start

    def skip_crash_report(self):
        """
            skip crash report
        """
        if not self._locator.btn_chrome_option.exists:
            print ""
        self._locator.btn_chrome_option.click()
        self._locator.btn_chrome_setting.click()
        self.d(text="Privacy").click()
        if self._locator.btn_chrome_usage_true != None:
            self._locator.btn_chrome_usage_true.click()
        time.sleep(2)
        assert self._locator.btn_chrome_usage_false != None, \
        "ERROR: skip crash report failed"
        self.d.press.back()
        self.d.press.back()

    def click_link_button(self, key):
        """
        @summary: click link button
        """
        loop_count = 10
        while (not self.d(descriptionStartsWith=key).exists) and\
        (loop_count >= 0):
            self._locator.btn_chrome_refresh.click()
            time.sleep(20)
        assert loop_count >= 0, "The %s icon cannot be click" % key

        self.d(descriptionStartsWith=key).click()

    def show_bookmark(self):
        """
        @summary: goto browser bookmark settings show all the bookmark
        """
        print "[INFO] Show all the bookmark"
        if not self.focus_window(BrowserImpl.packagename):
            self.launch_app_from_home_sc("Chrome")
        time.sleep(2)
        self._locator.btn_chrome_option.click()
        time.sleep(2)
        self._locator.btn_browser_bookmarks_list.click()

    def delete_all_bookmark(self):
        """
        @summary: delete all bookmark
        """
        print "[INFO] Delete all bookmark"
        self.show_bookmark()
        while self._locator.bookmark_fist_title.exists:
            self._locator.bookmark_fist_title.long_click()
            if self._locator.btn_browser_bookmarkdelete.exists:
                self._locator.btn_browser_bookmarkdelete.click()

    def check_bookmark_exist(self, title):
        """
        @summary: check if the bookmark is exists
        """
        print "[INFO] Check if bookmark is existed"
        self.show_bookmark()
        if self._locator.bookmark_title(title):
            print "[INFO] Find bookmark[%s]" % title
            return True
        print "[WARNING] Not find bookmark [%s]" % title
        return False

    def add_bookmark_and_check(self, title):
        """
        @summary: add website to bookmark and check it in bookmark list
        """
        print "[INFO] Add bookmark[%s] and check" % title
        if not self.focus_window(BrowserImpl.packagename):
            self.launch_from_am()
        self._locator.btn_browser_bookmark.click()
        for _ in range(0, 30):
            self._locator.btn_browser_bookmarktitle.clear_text()
        self._locator.btn_browser_bookmarktitle.set_text(title)
        self._locator.btn_browser_bookmarksave.click()
        assert self.check_bookmark_exist(title), "ERROR: Add bookmark error!"

    def switch_tab(self, url, urlcheck, xurl, xurlcheck, count):
        """
            switch tab
        """
        print "[INFO] Open website :%s" % url
        self.launch_app_from_home_sc("Chrome")
        self.swipe_screen_down()
        self.open_website(url)
        self.web_check(urlcheck)
        print "[INFO] Open website :%s" % xurl
        self.open_tab()
        self.open_website(xurl)
        self.web_check(xurlcheck)
        for _ in (0, int(count)):
            print "[INFO] Swith tab"
            self.d.click(180, 57)
            self.web_check(urlcheck)
            self.d.click(490, 57)
            self.web_check(xurlcheck)

    def download_verify(self, status, timeout=20):
        """
        @summary: verify download status
        @parameter:
            status: download status need to verify
            {In progress, Queued, packageName}
        @return: None
        """
        print("INFO: Verify Download %s" % status)
        self.d.press.home()
        self.launch_app_from_home_sc("Downloads")
        while not self.d(textContains=status, \
        packageName="com.android.documentsui").exists:
            time.sleep(2)
            timeout -= 2
            if timeout <= 0:
                break
        verify_download = self.d(textContains=status, \
        packageName="com.android.documentsui").exists
        assert verify_download, "ERROR:Downloads recovery error!"
        print("INFO: Verify Download PASS")


    def refresh(self):
        """
        @summary: Refresh active page
        """
        print "[INFO] Refresh"
        timeout = 3 * 60
        _timeout = timeout
        self._locator.btn_refresh.click()
        while True:
            if self._locator.btn_stop_page_loading.exists is False:
                print "[INFO] Page load finish"
                break
            print "[INFO] Page loading ... "
            if timeout <= 0:
                print "[INFO] Page not finish load within %d seconds" % _timeout
                break
            time.sleep(6)
            timeout -= 6

    def longtime_refresh(self, checkpoint, duration):
        """
        @summary: continuous refresh
            use this way to check page can keep connection for a duration
        @param checkpoint: a text for checking page load success
        @param duration: (string) minutes of refresh duration
        1. click refresh button
        2. wait page load success
        3. repeat step 1,2 until duration timeout
        """
        print "[INFO] Long time refresh for %s minutes" % duration
        loop = 1
        interval = int(self.cfg.get("interval"))
        if interval is None:
            interval = 1
        duration = int(duration) * 60
        start_time = time.time()
        print "[INFO] Begin at: %s" % time.strftime('%Y-%m-%d %H:%M:%S')
        print "[INFO] Start %d refresh" % loop
        while True:
            self.refresh()
            self.web_check(checkpoint)
            elapse = time.time() - start_time
            print "[INFO] Elapse time: %d seconds" % elapse
            if elapse > duration:
                print "[INFO] End at: %s" % time.strftime('%Y-%m-%d %H:%M:%S')
                break
            loop += 1
            print "[INFO] Start %d refresh, after %d seconds" % (loop, interval)
            time.sleep(interval)

    def swipe_screen_down(self):
        """ swipe screen down for url widget maybe disappear"""
        display_w = self.d.info["displayWidth"]
        display_h = self.d.info["displayHeight"]
        self.d.swipe(display_w / 2, display_h / 2, display_w / 2, display_h)

    def web_check_phone(self, key, timeout=None):
        """
        @summary: check if open a website successfully in phone chrome
        @return: last time
        """
        if timeout == None:
            timeout = self.cfg.get("url_timeout")
        time_start = time.time()
        time.sleep(2)
        time_last = time.time()
        print("INFO: Start at %d" % time_start)
        while True:
            time_last = time.time()
            assert not self._locator.btn_web_notavailable\
            .exists, "ERROR:The webpage is not available"
            assert not self._locator.btn_web_timeout\
            .exists, "ERROR:The webpage is timeout"
            assert int(time_last - time_start) <= int(timeout), \
            "ERROR: The webpage does not loading successfully in %d s" \
            % int(timeout)
            if self._locator.btn_infobar_close.exists:
                self._locator.btn_infobar_close.click()
            if self.d(descriptionMatches=key, \
                className="android.view.View").exists or \
            self.d(description="nba gifs").exists:
                time.sleep(2)
                break
            time.sleep(10)
        print("INFO: Time last %d" % (time_last - time_start))
        return time_last - time_start
