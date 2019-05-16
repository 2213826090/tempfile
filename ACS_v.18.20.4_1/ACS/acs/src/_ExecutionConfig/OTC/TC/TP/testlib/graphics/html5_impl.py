# Copyright (C) 2015  Jin, YingjunX <yingjunx.jin@intel.com>
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

'''
@summary: Class for ChromeCast operation
@since: 03/11/2015
@author: Yingjun Jin
'''


import time
import os
from testlib.util.common import g_common_obj
from testlib.util.config import TestConfig
# from testlib.browser.browser_impl import BrowserImpl
from testlib.graphics.tools import ConfigHandle
from testlib.util.repo import Artifactory


class Locator(object):

    """
    locator
    """

    def __init__(self, device):
        self._device = device

    @property
    def wait_exist(self):
        """ wait until the ui object exist """
        def _wait(uiobj, timeout=5):
            return uiobj.wait("exists", timeout * 500)
        return _wait

    @property
    def performance_tests(self):
        return self._device(text="Performance Tests")

    @property
    def next_btn(self):
        return self._device(text="Next")

    @property
    def noths_btn(self):
        return self._device(text="No thanks")

    @property
    def btn_chrome_setup(self):
        """ UI button setup title in Welcome pop up """
        return self._device(textContains="Set up Chrome", \
            resourceId="com.android.chrome:id/title")

    @property
    def btn_chrome_account_skip(self):
        """ UI button no thanks in signin pop up """
        return self._device(textContains="No thanks", \
            resourceId="com.android.chrome:id/negative_button")

    @property
    def btn_chrome_account_done(self):
        """ UI button done in signin pop up """
        return self._device(textContains="Done", \
            resourceId="com.android.chrome:id/positive_button")


class Html5Impl(object):

    '''
    classdocs
    '''
    pkg_name = "com.android.chrome"
    activity_name = "com.google.android.apps.chrome.Main"

    def __init__(self):
        self._device = g_common_obj.get_device()
        self._locator = Locator(self._device)
        self._config_handle = ConfigHandle()

    def read_cfg(self):
        """ read the cfg
        """
        self.web = self.cfg.get("web")
        self.key = self.cfg.get("key")
        self.data = self.cfg_data.get("string_data")
        self.net_str = self.data.split(",")

    def launch_chrome_am(self):
        """ Launch Chrome
        """
        print "Launch Chrome"
        g_common_obj.launch_app_am(self.pkg_name, self.activity_name)
        time.sleep(1)
        # Find "Accept & continue" obj, click url bar if not find accept obj.
        if self._device(resourceId="com.android.chrome:id/terms_accept").exists:
            self._device(resourceId="com.android.chrome:id/terms_accept").click.wait()
            time.sleep(2)
        else:
            if self._device(resourceId="com.android.chrome:id/url_bar").exists:
                self._device(resourceId="com.android.chrome:id/url_bar").click.wait()
                return
        self._locator.wait_exist(self._locator.performance_tests)
        # click "NO THANKS"
        if self._device(resourceId="com.android.chrome:id/negative_button").exists:
            self._device(resourceId="com.android.chrome:id/negative_button").click.wait()
            time.sleep(1)
        if self._locator.next_btn.exists:
            self._locator.next_btn.click.wait()
            time.sleep(1)
        if self._locator.noths_btn.exists:
            self._locator.noths_btn.click.wait()
            time.sleep(1)
        if self._locator.btn_chrome_setup.exists:
            self._locator.btn_chrome_account_skip.click.wait()
            time.sleep(1)
        if self._locator.btn_chrome_account_done.exists:
            self._locator.btn_chrome_account_done.click.wait()
            time.sleep(1)
        if self._device(resourceId="com.android.chrome:id/url_bar").exists:
            # Make sure able to set text for bxt platform.
            print "Rerun chrome to make sure setText normally."
            self.stop_chrome_am()
            g_common_obj.launch_app_am(self.pkg_name, self.activity_name)
            time.sleep(1)
            # Skip improving chrome performance stage.
            if self._device(text="NO THANKS").exists:
                self._device(text="NO THANKS").click.wait()
                time.sleep(1)
            self._device(resourceId="com.android.chrome:id/url_bar").click.wait()
            return

    def stop_chrome_am(self):
        """ Stop Settings via adb am command
        """
        print "Stop Settings by adb am"
        g_common_obj.stop_app_am(self.pkg_name)

    def run_test(self):
        """ open the wep to test CSS3 Selectors
        """
        print "Run test"
        if self._device(resourceId="com.android.chrome:id/url_bar").exists:
            self._device(resourceId="com.android.chrome:id/url_bar").clear_text()
            self._device(resourceId="com.android.chrome:id/url_bar").set_text(self.web)
            time.sleep(1)
            self._device.press.enter()
            return
        elif self._device(resourceId='com.android.chrome:id/search_box').exists:
            self._device(resourceId='com.android.chrome:id/search_box').clear_text()
            self._device(resourceId='com.android.chrome:id/search_box').set_text(self.web)
            time.sleep(1)
            self._device.press.enter()
            return

        #click the "No Thanks"
        if self._device(resourceId="com.android.chrome:id/negative_button"):
            self._device(resourceId="com.android.chrome:id/negative_button").click.wait()
        if not self._device(resourceId="com.android.chrome:id/url_bar").exists:
            if self._device(resourceId="com.android.chrome:id/search_box_text").exists:
                self._device(resourceId="com.android.chrome:id/search_box_text").click.wait()
        if self._device(resourceId="com.android.chrome:id/url_bar"):
            self._device(resourceId="com.android.chrome:id/url_bar").set_text(self.web)
        if self._device(resourceId='com.android.chrome:id/search_box'):
            self._device(resourceId='com.android.chrome:id/search_box').set_text(self.web)
        time.sleep(1)
        self._device.press.enter()
        if self._device(text="The page at 29a.ch says:").exists:
            self._device(text="OK").click.wait()

    def check_test_result(self):
        """ check html5 test result
        """
        print "Check the test result"
        self._device.press.menu()
        time.sleep(1)
        #try:
        #    self._device(textContains="Find in page").click.wait()
        #except Exception,e:
        #    print e
        #    self._device(resourceId="com.android.chrome:id/menu_item_text").click.wait()
        self._device(textContains="Find in page").click.wait()
        time.sleep(2)
        self._device(resourceId="com.android.chrome:id/find_query").set_text(self.key)
        time.sleep(3)
        if not self._device(text="1/1").exists:
            assert not self.check_network(self.net_str), "The network error,please check dut network!"
            assert False, "The test case failed or timeout"

    def run_case(self, tc_conf, tc_sect):
        """ run the test case
        """
        self.cfg = TestConfig().read(tc_conf, tc_sect)
        self.cfg_data = TestConfig().read(tc_conf, "check_network_database")
        self.read_cfg()
        self.check_chrome_installed()  # Check chrome previously
        self.launch_chrome_am()
        self.run_test()
        print self.cfg['key']
        self._device(descriptionContains=self.cfg['key']).wait.exists(timeout=120000)
        time.sleep(2)
        self.check_test_result()
        self._device.press.back()
        self.stop_chrome_am()

    def run_case_benchmark(self, tc_conf, tc_sect):
        """ run the test case with benchmark
        """
        self.cfg = TestConfig().read(tc_conf, tc_sect)
        self.cfg_data = TestConfig().read(tc_conf, "check_network_database")
        self.read_cfg()
        self.check_chrome_installed()  # Check chrome previously
        self.launch_chrome_am()
        if not self._device(resourceId="com.android.chrome:id/url_bar").exists:
            print "Try rerun chrome."
            self.stop_chrome_am()
            self.launch_chrome_am()
            self._device(resourceId="com.android.chrome:id/url_bar").clear_text()
            assert self._device(resourceId="com.android.chrome:id/url_bar").set_text(self.web), "Fail to set text"
        else:
            self._device(resourceId="com.android.chrome:id/url_bar").clear_text()
            assert self._device(resourceId="com.android.chrome:id/url_bar").set_text(self.web), "Fail to set text"
        time.sleep(3)
        self._device.press.enter()
        time.sleep(60)
        x = self._device.info['displayWidth']
        y = self._device.info['displayHeight']
        for _ in range(0, 2):
            g_common_obj.adb_cmd(
                'input tap' + ' ' + str(x / 2) + ' ' + str(y / 2))
            time.sleep(3)
        print "start the test"
        time.sleep(30)
        for _ in range(0, 2):
            g_common_obj.adb_cmd(
                'input tap' + ' ' + str(x / 2) + ' ' + str(y / 2))
            time.sleep(3)
        time.sleep(60)
        self.check_test_result()
        self.stop_chrome_am()

    def check_network(self, network_key):
        """ Check network key string in net database
        """
        string_list = list(network_key)
        print string_list
        for i in range(0, len(string_list)):
            print string_list[i]
            self._device(resourceId="com.android.chrome:id/close_find_button").click.wait()
            print "Check the network"
            self._device.press.menu()
            time.sleep(3)
            self._device(text="Find in page").click()
            time.sleep(3)
            self._device(resourceId="com.android.chrome:id/find_query"
                         ).set_text(string_list[i])
            time.sleep(3)
            if self._device(text="1/1").exists:
                return True
        return False

    def check_chrome_installed(self):
        """
        Make sure chrome app is pre-installed.
        If no, then install it.
        """
        result = self._config_handle.check_apps(self.pkg_name)
        if not result:
            cfgName = "tests.tablet.artifactory.conf"
            cfg_app = TestConfig().read(cfgName, "content_chrome").get('name')
            cfg_arti = TestConfig().read(section='artifactory').get('location')
            arti = Artifactory(cfg_arti)
            cmd = "adb install -r %s" % arti.get(cfg_app)
            print "Installing chrome app."
            os.system(cmd)

html5 = Html5Impl()
