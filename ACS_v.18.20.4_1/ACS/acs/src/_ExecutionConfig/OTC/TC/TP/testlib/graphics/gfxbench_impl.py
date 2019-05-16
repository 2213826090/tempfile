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
@summary: Gfxbench class
@since: 04/11/2016
@author: Zhang,RongX Z(rongx.z.zhang@intel.com)
"""
import time
from testlib.graphics.common import pkgmgr,busybox_obj,file_sys,get_current_focus_window
from testlib.graphics.glbenckmark_impl import GLBenchmarkImpl
from testlib.util.common import g_common_obj
from testlib.util.config import TestConfig
from testlib.util.repo import Artifactory
from testlib.graphics.tools import ConfigHandle


class Gfxbench(GLBenchmarkImpl):

    """ GfxbenchmarkImpl """
    CONFIG_FILE = 'tests.common.gfxbench40.conf'
    PKG_NAME = "com.glbenchmark.glbenchmark27"
    ROOT_DIR = "/sdcard/Android/data/com.glbenchmark.glbenchmark27/"
    RESULT_DIR = ROOT_DIR + "files/results/"
    MAIN_ACTIVITY = "net.kishonti.app.MainActivity"
    TEST_ACTIVITY = "net.kishonti.benchui.BenchTestActivity"
    USERNAME = "graphics_test"
    PASSWORD = "123456"
    TEST_GROUPS = ['High-Level Tests', 'Low-Level Tests', 'Special Tests']

    def __init__(self):
        self.device = g_common_obj.get_device()
        self.configer = TestConfig()
        self.config = self.configer.read(self.CONFIG_FILE, "Gfxbench40Impl")
        self.config_handle = ConfigHandle()
        self.config["artifactory_location"] = self.config_handle.read_configuration('artifactory', 'location', '/etc/oat', 'sys.conf')
        self.arti = Artifactory(self.config.get('artifactory_location'))
        self.apk_path = self.arti.get(self.config.get("apk"))
        self.remote_apk_cache_path = self.config.get("apk_cache_dir")
        self.apk_cache_path = self.arti.get(self.config.get("apk_cache_files"))
        self.apk_cache_name = self.config.get("apk_cache_files").split('/')[-1]

    def setup(self):

        gfb = Gfxbench()

        def _wait_init(timeout=60):
            """wait app initialize"""
            start_time = time.time()
            is_completed = False
            while time.time() - start_time < timeout:
                _, current_activity = get_current_focus_window()
                if current_activity == gfb.MAIN_ACTIVITY:
                    is_completed = True
                    break
                if gfb.device(text='Accept').exists:
                    gfb.device(text='Accept').click()
                if gfb.device(text='OK').exists:
                    gfb.device(text='OK').click()
                if gfb.device(text='Retry').exists:
                    gfb.device(text='Retry').click()
                time.sleep(5)
            assert is_completed, \
                "[FAILURE] Server timeout: %s" % (timeout)

        def _clean_chooses():
            """check off all checkbox"""
            def _search_item_checkbox(name):
                """scroll find checkbox"""
                if gfb.device(resourceId="com.glbenchmark.glbenchmark27:id/main_testSelectListView")\
                        .scroll.vert.to(text=name):
                    return gfb.device(text=name).right(className="android.widget.CheckBox")

            for each in gfb.TEST_GROUPS:
                checkbox = _search_item_checkbox(each)
                if checkbox.exists:
                    check = checkbox.info['checked']
                    if check == True:
                        checkbox.click()
                    else:
                        checkbox.click()
                        checkbox.click()
        # install apk
        if not pkgmgr._package_installed(pkgName=self.PKG_NAME):
            pkgmgr.apk_install(self.apk_path)
        # Push cache files
        if not any([True for i in file_sys.get_file_list(
                        self.remote_apk_cache_path + " -type d -print") if self.PKG_NAME in i]):
            busybox_obj.setup()
            clean_cache_cmd = "rm -rf %s%s" % (self.remote_apk_cache_path, self.PKG_NAME)
            push_cmd = "push %s %s" % (self.apk_cache_path, self.remote_apk_cache_path)
            cmd = "busybox tar xvf %s%s -C %s" % (self.remote_apk_cache_path,
                                                  self.apk_cache_name, self.remote_apk_cache_path)
            td_cmd = "rm -rf %s%s" % (self.remote_apk_cache_path, self.apk_cache_name)
            g_common_obj.adb_cmd_capture_msg(clean_cache_cmd)
            g_common_obj.adb_cmd_common(push_cmd)
            busybox_obj.adb_busybox_cmd(cmd)
            g_common_obj.adb_cmd_capture_msg(td_cmd)
        # Go to main menu
        g_common_obj.launch_app_am(self.PKG_NAME, self.MAIN_ACTIVITY)
        _wait_init(240)
        # Log in account
        for i in range(4):
            self.device(className="android.widget.RelativeLayout").swipe.left()
            time.sleep(1)
        if not self.device(text=self.USERNAME).exists:
            self.device(resourceId="com.glbenchmark.glbenchmark27:id/settings_editableUserName").\
                set_text(self.USERNAME)
            time.sleep(1)
            if not self.device(resourceId="com.glbenchmark.glbenchmark27:id/settings_editableUserPassword").exists:
                self.device.press.back()
                time.sleep(1)
            self.device(resourceId="com.glbenchmark.glbenchmark27:id/settings_editableUserPassword").\
                set_text(self.PASSWORD)
            time.sleep(1)
            if not self.device(resourceId="com.glbenchmark.glbenchmark27:id/settings_login").exists:
                self.device.press.back()
                time.sleep(1)
            self.device(resourceId="com.glbenchmark.glbenchmark27:id/settings_login").click.wait()
            time.sleep(1)
        for i in range(4):
            self.device(className="android.widget.RelativeLayout").swipe.right()
            time.sleep(1)
        # Show test lists
        if self.device(resourceId='com.glbenchmark.glbenchmark27:id/main_circleControl').exists:
            bounds = self.device(resourceId='com.glbenchmark.glbenchmark27:id/main_circleControl').info['visibleBounds']
            for i in range(10):
                self.device.click(
                    bounds['left'] + (bounds['right'] - bounds['left']) * 0.75,
                    bounds['bottom'] - 20 * i)
                if self.device(text='Test selection').exists:
                    break
        assert self.device(text='Test selection').exists, \
            "[FAILURE] Failed skip_guide"
        _clean_chooses()
        self.device(resourceId="com.glbenchmark.glbenchmark27:id/main_testSelectListView").scroll.toBeginning()
        time.sleep(3)

    def stop_gfxbenchmark(self):
        g_common_obj.adb_cmd_capture_msg("am force-stop %s" % self.PKG_NAME)
        cmd = "rm -rf  %s*" % (self.RESULT_DIR)
        g_common_obj.adb_cmd_capture_msg(cmd)

    def set_workaround(self):
        pid = g_common_obj.adb_cmd_capture_msg("ps | grep 'com.glbenchmark.glbenchmark27' |awk '{print $2}' ")
        print "PID is %s" % pid
        if pid:
            g_common_obj.adb_cmd_capture_msg("'echo -12>/proc/%s/oom_adj'" % pid)
            oom_adj = g_common_obj.adb_cmd_capture_msg("cat proc/%s/oom_adj" % pid)
            print oom_adj

    def clean(self):
        """uninstall apk"""
        cmd = "uninstall %s" % (self.PKG_NAME)
        g_common_obj.adb_cmd_common(cmd)

    def get_result(self, timeout, test_name):
        start_time = time.time()
        while time.time() - start_time < timeout:
            cmd = "ls %s*" % (self.RESULT_DIR)
            msg = g_common_obj.adb_cmd_capture_msg(repr(cmd))
            if msg.find("No such file or directory") == -1:
                break
            time.sleep(10)
        assert msg.find("No such file or directory") == -1, "Test time out"
        cmd = "cat %s*/*.json" % (self.RESULT_DIR)
        result = g_common_obj.adb_cmd_capture_msg(repr(cmd))
        assert result.find('status":"OK"') != -1, "run %s failed,and result is %s" % (test_name, result)

    def run_test(self, names, timeout=1800):
        """run test"""

        for each in names.split(','):
            print "[Debug] test: %s" % (each)
            if self.device(resourceId="com.glbenchmark.glbenchmark27:id/main_testSelectListView")\
                    .scroll.vert.to(text=each):
                self.device(text=each).right(className="android.widget.CheckBox").click.wait()
        checked_count = self.device(className="android.widget.CheckBox", checked=True).count
        assert checked_count == len(names.split(',')),\
            "[FAILURE] Failed Choose Test item"
        self.device(text='Start').click.wait()
        _, current_activity = get_current_focus_window()
        assert current_activity == self.TEST_ACTIVITY,\
            "[FAILURE] Tests failed to start"

        print "[Debug] start test: %s" % (names)
        start_time = time.time()
        is_completed = False
        while time.time() - start_time < timeout:
            _, current_activity = get_current_focus_window()
            if current_activity == self.MAIN_ACTIVITY:
                is_completed = True
                break
            time.sleep(10)
        assert is_completed,\
            "[FAILURE] Test time out: %s" % (timeout)