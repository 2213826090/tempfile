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
@summary: ChromeExtendImpl class
@since: 02/10/2015
@author: Ding, JunnanX (junnanx.ding@intel.com)
"""
import os
import time
import difflib
import threading
from testlib.util.common import g_common_obj
from testlib.util.config import TestConfig
from testlib.browser.browser_impl import BrowserImpl
from testlib.browser.chrome_impl import ChromeImpl
from testlib.graphics.extend_systemui_impl import SystemUiExtendImpl as SystemUi
from testlib.graphics.common import Allow_Permission
from testlib.graphics.html5_impl import html5
import testlib.graphics.decorator as decorator


class ChromeExtendImpl(BrowserImpl, ChromeImpl):

    """Chrome uiautomator """

    class MainFrame(object):

        """Main frame UI"""

        def __init__(self, device):
            self.device = device

        @property
        def url_bar(self):
            return self.device(resourceId="com.android.chrome:id/url_bar")

        @property
        def next_btn(self):
            return self.device(text="Next")

        @property
        def noths_btn(self):
            return self.device(text="No thanks")

    class PluginVideo(object):

        """Video plugin UI"""

        def __init__(self, device):
            self.device = device

        @property
        def scrubber(self):
            return self.device(description="movie time scrubber")

        @property
        def play(self):
            return self.device(descriptionContains="play")

        @property
        def pause(self):
            return self.device(description="pause")

        @property
        def fullscreen(self):
            return self.device(description="enter full screen")

    def __init__(self):
        super(ChromeExtendImpl, self).__init__(cfg={})
        self.main_frame = ChromeExtendImpl.MainFrame(self.d)
        self.plugin_video = ChromeExtendImpl.PluginVideo(self.d)
        self.a = Allow_Permission()

    def launch(self):
        """Launch app"""
        html5.check_chrome_installed()
        g_common_obj.launch_app_am('com.android.chrome',
                                   'com.google.android.apps.chrome.Main')
        self.a.permission_allower()

    def resume(self):
        """Resume app"""
        cmdstr = "am start %s/%s" % ('com.android.chrome',
                                     'com.google.android.apps.chrome.Main')
        g_common_obj.adb_cmd(cmdstr)

    def chrome_setup(self):
        """
        @summary: close all welcom dialog
        """
        if self.d(resourceId="com.android.chrome:id/terms_accept").exists:
            self.d(resourceId="com.android.chrome:id/terms_accept").click.wait()
        if self.d(resourceId="com.android.chrome:id/negative_button").exists:
            self.d(resourceId="com.android.chrome:id/negative_button").click.wait()
        self.d.watcher("SKIP_WELCOME").when(resourceId="com.android.chrome:id/negative_button").\
        click(resourceId="com.android.chrome:id/negative_button")
        time.sleep(3)
        if self._locator.btn_chrome_accept.exists:
            self._locator.btn_chrome_accept.click()
            time.sleep(2)
        if self.main_frame.next_btn.exists:
            self.main_frame.next_btn.click()
            time.sleep(2)
        if self.main_frame.noths_btn.exists:
            self.main_frame.noths_btn.click()
            time.sleep(2)
        if self._locator.btn_chrome_setup.exists:
            self._locator.btn_chrome_account_skip.click()
            time.sleep(2)
        if self._locator.btn_chrome_account_done.exists:
            self._locator.btn_chrome_account_done.click()
            time.sleep(2)
        self.swipe_screen_down()
        #assert self._locator.btn_url_bar.exists or \
        #self._locator.btn_new_tab.exists, \
        #"[FAILURE] Skip Welcome dialog failed!"
        self.d.watcher("SKIP_WELCOME").remove()

    def open_website(self, url):
        """open URL"""
        print "[Debug] open_website URL: %s" % (url)
        if self.d(text="No thanks").exists:
            self.d(text="No thanks").click()
        if not self.d(resourceId="com.android.chrome:id/url_bar").exists:
            if self.d(resourceId="com.android.chrome:id/search_box_text").exists:
                self.d(resourceId="com.android.chrome:id/search_box_text").click.wait()
        self.d(resourceId="com.android.chrome:id/url_bar"
                     ).click.wait()
        self.d(resourceId="com.android.chrome:id/url_bar"
                     ).clear_text()
        self.d(resourceId="com.android.chrome:id/url_bar"
                     ).set_text(url)
        time.sleep(2)
        self.d.press.enter()
        time.sleep(3)
        self.a.permission_allower()
        time.sleep(3)
        if self.d(text="CONTINUE").exists:
            self.d(text="CONTINUE").click.wait()
            time.sleep(3)

    def play_video(self, secs=5):
        """play video"""
        print "[Debug] play_video"
        pre_dump = self.d.dump()
        pre_focus = SystemUi.get_current_focus()
        print pre_focus

        self.a.permission_allower()
#         if not self.d(description="play").exists:
#             self.d.press.back()
        time.sleep(2)
        self.plugin_video.play.click.wait(timeout=5000)
        time.sleep(secs)
        end_dump = self.d.dump()
        end_focus = SystemUi.get_current_focus()
        print end_focus

        seq_match = difflib.SequenceMatcher(None, pre_dump, end_dump)
        ratio = round(seq_match.ratio(), 2) * 100
        print "ratio:%s" % ratio

        if ratio == 100 and pre_focus == end_focus:
            assert False, "[FAILURE] Play video No any reply"

    def enter_video_fullscreen(self):
        if self.plugin_video.fullscreen.exists:
            self.plugin_video.fullscreen.click()
        time.sleep(5)
        # Skip guide pop
        y = self.d.info["displayHeight"]
        x = self.d.info["displayWidth"]
        self.d.swipe(x / 2, 0, x / 2, y / 4, steps=100)
        time.sleep(5)

    def open_new_tab(self, timeout=600):
        """override parent method"""
        pass

    def clear_data(self):
        """
        @summary: clean browser data
        """
        clear_cmd = "pm clear com.android.chrome"
        g_common_obj.adb_cmd_capture_msg(clear_cmd)
        self.d.press.home()

    def web_check(self, key, timeout=None):
        """
        @summary: check if open a website successfully
        @return: last time
        """
        print "Check the test result"
        time.sleep(timeout)
        self.d.press.menu()
        time.sleep(3)
        self.d(text="Find in page").click()
        time.sleep(3)
        self.d(resourceId="com.android.chrome:id/find_query").set_text(key)
        time.sleep(3)
        assert self.d(textMatches="[1-9]+/[1-9]+").exists, "The test case failed or timeout"
        if self.d(resourceId='com.android.chrome:id/close_find_button').exists:
            self.d(resourceId='com.android.chrome:id/close_find_button').click()

    @decorator.restart_uiautomator
    def enter_youtube_fullscreen(self, duration=30):

        x, y = self.d.info['displayWidth'] / 2, self.d.info['displayHeight'] / 2

        def click_center(x, y):
            print "Click center to show full screen button."
            for i in range(15):
                time.sleep(2)
                os.system("adb shell input touchscreen tap %s %s" %(x, y))

        thread = threading.Timer(2.0, click_center, [x, y])
        thread.start()

        for i in range(5):
            time.sleep(1)
            try:
                self.d(index="5", className="android.widget.Button").click()
                time.sleep(duration)
                return True
            except:
                pass
            try:
                self.d(index="3", className="android.widget.Button").click()
                time.sleep(duration)
                return True
            except:
                pass
            try:
                self.d(descriptionContains="full screen").click()
                time.sleep(duration)
                return True
            except:
                pass
        thread.join()
        raise Exception("Full screen playback failed.")

    def get_youtube_url_key(self, url_name, search_key):
        file_src = 'tests.html5_and_webgl.conf'
        content = TestConfig().read(file_src, 'youtube_video')
        return content.get(url_name), content.get(search_key)

chrome_impl = ChromeExtendImpl()