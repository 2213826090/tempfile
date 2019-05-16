#Copyright (C) 2014  Yi, GraceX <gracex.yi@intel.com>
#Intel Corporation All Rights Reserved.

#The source code contained or described herein and
#all documents related to the source code ("Material") are owned by
#Intel Corporation or its suppliers or licensors.

#Title to the Material remains with Intel Corporation or
#its suppliers and licensors.
#The Material contains trade secrets and proprietary and
#confidential information of Intel or its suppliers and licensors.
#The Material is protected by worldwide copyright and
#trade secret laws and treaty provisions.
#No part of the Material may be used, copied, reproduced, modified,
#published, uploaded, posted, transmitted, distributed
#or disclosed in any way without Intel's prior express written permission.
#No license under any patent, copyright, trade secret or
#other intellectual property right is granted to
#or conferred upon you by disclosure or delivery of the Materials,
#either expressly, by implication, inducement, estoppel or otherwise.

#Any license under such intellectual property rights must be express
#and approved by Intel in writing.

"""
@summary: module for Google play application
@since: 08/21/2014
@author: Grace yi (gracex.yi@intel.com)
"""
import os
import time
from testlib.util.common import g_common_obj

class GooglePlayImpl:
    """
        class for Google play application Home UI
    """
#--------- begin locator -------------
    class Locator(object):
        """
            Helper for locator UI Object
        """

        def __init__(self, device):
            self.d = device

        @property
        def btn_play_store(self):
            """ UI button title Play Store """
            return self.d(text="Play Store")

        @property
        def btn_apps(self):
            """ UI button title Apps """
            return self.d(text="Apps")

        @property
        def btn_books(self):
            """ UI button title Books """
            return self.d(text="Books")

        @property
        def btn_play_store_apps(self):
            """ UI button apps link """
            return self.d(resourceId="com.android.vending:id/li_title", \
                text="APPS")

        @property
        def btn_play_store_books(self):
            """ UI button books link """
            return self.d(resourceId="com.android.vending:id/li_title", \
                text="BOOKS")

        @property
        def btn_books_top_free(self):
            """ UI button Top free """
            if self.d(resourceId="com.android.vending:id/play_header_list_tab_container")\
                .child_by_text("TOP FREE", allow_scroll_search=True) != None:
                return self.d(textContains="TOP FREE")
            return None

        @property
        def btn_apps_top_free(self):
            """ UI button Top free """
            if self.d(resourceId="com.android.vending:id/play_header_list_tab_container")\
                .child_by_text("TOP FREE", allow_scroll_search=True) != None:
                return self.d(textContains="TOP FREE")
            return None

        @property
        def btn_apps_firstapp(self):
            """ UI button first app in free apps list """
            return self.d(resourceId="com.android.vending:id/play_card", \
                instance="0")

        def btn_app_appnumber(self, index):
            """ UI button first book in free app list """
            return self.d(resourceId="com.android.vending:id/play_card", \
                instance=index)

        @property
        def btn_books_firstbook(self):
            """ UI button first book in free books list """
            return self.d(resourceId="com.android.vending:id/li_thumbnail", \
                instance="0")

        def btn_books_booknumber(self, index):
            """ UI button first book in free books list """
            return self.d(resourceId="com.android.vending:id/li_thumbnail", \
                instance=index)

        @property
        def btn_apps_install(self):
            """ UI button install in app info screen """
            return self.d(resourceId="com.android.vending:id/buy_button", \
                text="INSTALL")

        @property
        def btn_apps_uninstall(self):
            """ UI button uninstall in app info screen """
            return self.d(\
                resourceId="com.android.vending:id/uninstall_button", \
                text="UNINSTALL")

        @property
        def btn_apps_cancel_download(self):
            """ UI button cancel download in app info screen """
            return self.d(\
                resourceId="com.android.vending:id/cancel_download")

        @property
        def btn_ok(self):
            """ UI button ok """
            return self.d(textContains="OK")

        @property
        def btn_app_continue(self):
            """ UI button continue button in books info """
            return self.d(\
                resourceId="android:id/button1", text="Continue")

        @property
        def btn_apps_accept(self):
            """ UI button install in app info screen """
            return self.d(\
                resourceId="com.android.vending:id/continue_button_label", \
                text="ACCEPT")

        @property
        def btn_apps_download_process(self):
            """ UI button install in app info screen """
            return self.d(\
                resourceId="com.android.vending:id/downloading_percentage")

        @property
        def btn_apps_dynamic_status(self):
            """ UI button dynamic status """
            return self.d(\
                resourceId="com.android.vending:id/summary_dynamic_status")

        @property
        def btn_apps_launch(self):
            """ UI button install in app info screen """
            return self.d(resourceId="com.android.vending:id/launch_button", \
                   text="OPEN")

        @property
        def btn_book_buy(self):
            """ UI button buy button in books info """
            return self.d(resourceId="com.android.vending:id/buy_button")

        @property
        def btn_book_continue(self):
            """ UI button continue button in books info """
            return self.d(\
                resourceId="com.android.vending:id/continue_button_label")

        @property
        def btn_book_launch(self):
            """ UI button launch button in books info """
            return self.d(resourceId="com.android.vending:id/launch_button")

        @property
        def btn_book_sync_off(self):
            """ UI button app sync off """
            return self.d(textContains="sync off")

        @property
        def btn_book_mainpage(self):
            """ UI button books main page """
            return self.d(\
                resourceId="com.google.android.apps.books:id/main_page")

    def __init__ (self, cfg={}):

        self.d = g_common_obj.get_device()
        self._locator = GooglePlayImpl.Locator(self.d)
        self.cfg = cfg

    def set_orientation_n(self):
        """
        @summary: set orientation as n
        """
        self.d.orientation = "n"

    def quit_by_backkey(self):
        """
            quit app by press back key
        """
        self.d.press.back()
        self.d.press.back()
        self.d.press.home()

    def launch_from_am(self):
        """
            Launch google play from am
        """
        print "[INFO] Launch Google play from am"
        g_common_obj.launch_app_am(self.cfg.get("package_name"), \
            self.cfg.get("activity_name"))
        time.sleep(10)
        assert self._locator.btn_play_store.exists, "[ERROR] Launch fail"

    def stop_from_am(self):
        """
            Stop google play from am
        """
        print "[INFO] Stop Google play from am"
        g_common_obj.stop_app_am(self.cfg.get("package_name"))

    @staticmethod
    def startApp():
        d = g_common_obj.get_device()
        g_common_obj.launch_app_from_home_sc("Play Store")
        time.sleep(3)
        if d(
            textStartsWith="Just a sec").exists or d(
            text="Couldn't sign in").exists:
            d.press.back()
            assert False, "Account is not signed"
        for _ in range(30):
            if d(text="Accept").exists:
                d(text="Accept").click.wait()
            if d(className="android.widget.ImageButton").exists:
                break
            time.sleep(2)
        if d(className="android.widget.ImageButton").exists:
            d(className="android.widget.ImageButton").click.wait()
        if d(text="Settings").exists:
            d(text="Settings").click.wait()
        if d(text="Auto-update apps").exists:
            d(text="Auto-update apps").click.wait()
        if d(text="Do not auto-update apps").exists:
            d(text="Do not auto-update apps").click.wait()
        g_common_obj.back_home()

    def uninstall_app(self):
        """
            Uninstall app
        """
        print "[INFO] Stop search from am"
        install_package = self.get_install_app_package()
        cmd = "adb uninstall %s" % install_package
        os.system(cmd)

    def get_install_app_package(self):
        """
        @summary: get installed app packagename in logcat
        """
        key_message = self.cfg.get("ins_message")
        cmd = "adb logcat -d|grep -i \"" + key_message + "\""
        print cmd
        pipe = os.popen(cmd).read()
        print "[INFO] Install message is %s" % pipe
        assert len(pipe) >= 2, "Get app package error, [return mes]:%s" % pipe
        package = pipe.split("=")[-1].split()[0]
        print "[INFO] The package of installed app is %s" % package
        return package

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

    def check_download(self, timeout=60):
        self.stop_from_am()
        self.launch_from_am()
        count = 0
        while not self._locator.btn_play_store_apps.exists:
            time.sleep(15)
            count += 1
            if count >= 20:
                assert self._locator.btn_play_store_apps.exist, \
                "ERROR: Google Play launch failed in 150s!"
        self._locator.btn_play_store_apps.click()
        assert self._locator.btn_apps.exists, "ERROR: Launch apps failed!"
        self._locator.btn_apps_top_free.wait.exists(timeout=120000)
        self._locator.btn_apps_top_free.click()
        time.sleep(3)
        while not self._locator.btn_apps_firstapp.exists:
            print "[INFO] The app list does not display. Waitting 10s"
            time.sleep(10)
            timeout -= 10
            assert timeout >= 0, "ERROR: App list does not display in 60s"
        self.__launch_a_free_app(6, 60)
        os.system("adb logcat -c")
        self._locator.btn_apps_accept.click()
        time.sleep(10)
        assert self._locator.btn_apps_download_process.wait.exists(timeout=60000)
        self.quit_by_backkey()

    def google_play_app_download(self, timeout=60):
        """
        @summary: Download app from playstore
        """
        self.stop_from_am()
        self.launch_from_am()
        count = 0
        while not self._locator.btn_play_store_apps.exists:
            time.sleep(15)
            count += 1
            if count >= 20:
                assert self._locator.btn_play_store_apps.exist, \
                "ERROR: Google Play launch failed in 150s!"
        self._locator.btn_play_store_apps.click()
        assert self._locator.btn_apps.exists, "ERROR: Launch apps failed!"
        self._locator.btn_apps_top_free.click()
        time.sleep(3)
        while not self._locator.btn_apps_firstapp.exists:
            print "[INFO] The app list does not display. Waitting 10s"
            time.sleep(10)
            timeout -= 10
            assert timeout >= 0, "ERROR: App list does not display in 60s"
        self.__launch_a_free_app(6, 60)
        os.system("adb logcat -c")
        self._locator.btn_apps_accept.click()
        time.sleep(10)
        print "[INFO] Installing the app"
        l_timeout = 1800
        while self._locator.btn_apps_download_process.exists and not \
        self._locator.btn_apps_launch.exists:
            print "[INFO] Download process %s" % \
            (self._locator.btn_apps_download_process.info['text'])
            time.sleep(10)
            l_timeout -= 10
            assert l_timeout > 0, \
            "ERROR The download does not finish in 1800s.Skip!"
        assert self._locator.btn_apps_launch.exists or \
        self._locator.btn_apps_dynamic_status.exists

    def google_play_app_install(self, timeout=60):
        """
        @summary: install a free app in google play app list
        """
        self.google_play_app_download(timeout)
        l_timeout = 1800
        while not self._locator.btn_apps_launch.exists and\
         self._locator.btn_apps_dynamic_status.exists:
            print "[INFO] Dynamic status update"
            time.sleep(10)
            l_timeout -= 10
            assert l_timeout > 0, \
            "ERROR The app install does not finish in 1800s.Skip!"
        print "[INFO] Install Finished"
        time.sleep(5)
        assert self._locator.btn_apps_launch.exists, "[INFO] Install app error"
        install_package = self.get_install_app_package()
        print "[INFO] Launch installed app"
        self._locator.btn_apps_launch.click()
        time.sleep(10)
        assert self.focus_window(install_package), "ERROR: Launch app failed!"

    def google_play_read_book(self, timeout=60):
        """
        @summary: install a free app in google play app list
        """
        self.stop_from_am()
        self.launch_from_am()
        count = 0
        while not self._locator.btn_play_store_books.exists:
            time.sleep(15)
            count += 1
            if count >= 20:
                assert self._locator.btn_play_store_books.exist, \
                "ERROR: Google Play launch failed in 150s!"
        self._locator.btn_play_store_books.click()
        assert self._locator.btn_books.exists, "ERROR: Launch books failed!"
        self._locator.btn_books_top_free.click()
        time.sleep(3)
        while not self._locator.btn_books_firstbook.exists:
            print "[INFO] The app list does not display. Waitting 10s"
            time.sleep(10)
            timeout -= 10
            assert timeout >= 0, "ERROR: App list does not display in 60s"
        self.__launch_a_free_book(6, 60)
        if self._locator.btn_book_sync_off.exists:
            self._locator.btn_book_sync_off.click()
        r_timeout = 180
        while not self._locator.btn_book_mainpage.exists:
            print "[INFO] Loading the boot"
            time.sleep(10)
            r_timeout -= 10
            assert r_timeout > 0, \
            "ERROR The book does not finish loading in 180s.Skip!"

    def __launch_a_free_app(self, count, timeout=60):
        """
        @summary: loops click app until launching a free book
        """
        i = 0
        while int(i) <= count:
            self._locator.btn_app_appnumber(str(i)).click()
            if self._locator.btn_apps_cancel_download.exists:
                self._locator.btn_apps_cancel_download.click()
                time.sleep(2)
            while not self._locator.btn_apps_install.exists and not \
            self._locator.btn_apps_uninstall.exists:
                print "[INFO] The app info is loading.Wait 10s"
                time.sleep(10)
                timeout -= 10
                assert timeout >= 0, \
                "ERROR: There is no install or uninstall button Skip!"
            if self._locator.btn_apps_launch.exists:
                print "[INFO] The app has been installed. Uninstall first"
                self._locator.btn_apps_uninstall.click()
                self._locator.btn_ok.click()
                d_timeout = 180
                while self._locator.btn_apps_dynamic_status.exists and \
                not self._locator.btn_apps_install.exists:
                    print "[INFO] Dynamic status update"
                    time.sleep(10)
                    d_timeout -= 10
                    assert d_timeout > 0, \
                    "ERROR The app uninstall does not finish in 180s.Skip!"
            self._locator.btn_apps_install.click()
            if self._locator.btn_app_continue.exists:
                print "[Warning] The app need Payment options. \
                Install another one"
                i += 1
                self.d.press.back()
                self.d.press.back()
            else:
                break
        assert i <= 10, "ERROR: install a free app error!"

    def __launch_a_free_book(self, count, timeout=60):
        """
        @summary: loops click book until launching a free book
        """
        i = 0
        while int(i) <= count:
            self._locator.btn_books_booknumber(str(i)).click()
            while not self._locator.btn_book_launch.exists and not \
            self._locator.btn_book_buy.exists:
                print "[INFO] The book info is loading.Wait 10s"
                time.sleep(10)
                timeout -= 10
                assert timeout >= 0, \
                "ERROR: Book read button does not come out in 6s.Skip!"
            if self._locator.btn_book_launch.exists:
                print "[INFO] The app has been download. Read it"
                time.sleep(2)
                self._locator.btn_book_launch.click()
            elif self._locator.btn_book_buy.exists:
                print "[INFO] Loading the book"
                self._locator.btn_book_buy.click()
            time.sleep(5)
            if self._locator.btn_book_continue.exists:
                print "[Warning] The books need Payment options. \
                Read another one"
                i += 1
                self.d.press.back()
                self.d.press.back()
            else:
                break
        assert i <= 10, "ERROR: Launch a free book error!"
