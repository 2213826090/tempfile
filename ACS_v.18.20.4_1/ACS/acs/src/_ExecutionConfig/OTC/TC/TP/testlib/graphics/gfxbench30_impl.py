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
#published, uploaded, posted, transmitted, distributed
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
import re
import time
from uiautomator import JsonRPCError, AutomatorDeviceObject

from testlib.graphics.common import busybox_obj, pkgmgr
import testlib.graphics.common as common
from testlib.graphics.glbenckmark_impl import GLBenchmarkImpl
from testlib.util.common import g_common_obj
from testlib.util.config import TestConfig
from testlib.util.repo import Artifactory
from testlib.graphics.tools import ConfigHandle


class Gfxbench30Impl(GLBenchmarkImpl):

    """ Gfxbenchmark30Impl """

    CONFIG_FILE = 'tests.common.gfxbench30.conf'
    PKG_NAME = "com.glbenchmark.glbenchmark27"
    INIT_ACTIVITY = "net.kishonti.benchui.initialization.InitActivity"
    MAIN_ACTIVITY = "net.kishonti.gfxbench.GfxMainActivity"
    TEST_ACTIVITY = "net.kishonti.benchui.BenchTestActivity"

    TEST_GROUPS = ['High-Level Tests', 'Low-Level Tests', 'Special Tests']
    TEST_Alpha_Blending = 'Alpha Blending'
    TEST_Alpha_Blending_Offscreen = '1080p Alpha Blending Offscreen'

    class HomeUI(object):

        """Home UI"""

        def __init__(self, device):
            self.device = device

        @property
        def main_circle_control(self):
            return self.device(resourceId='com.glbenchmark.glbenchmark27:id/main_circleControl')

        def skip_guide_selection(self):
            """skip selection guide"""
            for _ in range(10):
                if not self.device().scroll.horiz.backward(steps=5):
                    break
            if self.main_circle_control.exists:
                bounds = self.main_circle_control.info['visibleBounds']
                for i in range(10):
                    self.device.click(
                        bounds['left'] + (bounds['right'] - bounds['left']) * 0.75,
                        bounds['bottom'] - 20 * i)
                    if self.device(text='Test selection').exists:
                        break
            assert self.device(text='Test selection').exists,\
                "[FAILURE] Failed skip_guide"

        def go_category_testlist(self):
            """go to test list category"""
            self.device().scroll.horiz.to(text='Test selection')

    class TestList(object):

        """Performance Tests List UI"""

        def __init__(self, device):
            self.device = device

        @property
        def btn_start(self):
            return self.device(text='Start')

        def clean_chooses(self):
            """check off all checkbox"""
            for each in Gfxbench30Impl.TEST_GROUPS:
                checkbox = self.search_item_checkbox(each)
                if checkbox.exists:
                    check = checkbox.info['checked']
                    if check == True:
                        checkbox.click()
                    else:
                        checkbox.click()
                        checkbox.click()

        def search_item_checkbox(self, name):
            """scroll find checkbox"""
            if self.device().scroll.vert.to(text=name):
                return self.device(text=name).right(className="android.widget.CheckBox")

    def __init__(self):
        GLBenchmarkImpl.__init__(self)
        self.device = g_common_obj.get_device()
        self.home = Gfxbench30Impl.HomeUI(self.device)
        self.testlist = Gfxbench30Impl.TestList(self.device)

        self.configer = TestConfig()
        self.config = self.configer.read(self.CONFIG_FILE, "Gfxbench30Impl")
        config_handle = ConfigHandle()
        self.config["artifactory_location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat', 'sys.conf')
        self.arti = Artifactory(self.config.get('artifactory_location'))
        self.apk_cache_dir = self.config.get("apk_cache_dir")
        self.dut_download = self.config.get("dut_download")

    def setup(self):
        """setup implement's resource file"""
        busybox_obj.setup()

        if not pkgmgr._package_installed(pkgName=self.PKG_NAME):
            file_path = self.config.get("apk")
            apk_path = self.arti.get(file_path)
            pkgmgr.apk_install(apk_path)

        cmd = "cd %s || mkdir -p %s" % (self.apk_cache_dir, self.apk_cache_dir)
        g_common_obj.adb_cmd_capture_msg(repr(cmd))
        apk_cache_files = self.arti.get(self.config.get("apk_cache_files"))
        ret = g_common_obj.push_file(apk_cache_files, self.dut_download)
        assert ret, 'Failed push %s' % (apk_cache_files)
        files_name = os.path.basename(apk_cache_files)
        cmd = "busybox tar zxf %s/%s --directory %s > /dev/null 2>&1 && print UnzipComplete"\
            % (self.dut_download, files_name, self.apk_cache_dir)
        output = busybox_obj.adb_busybox_cmd(repr(cmd), 60 * 10)
        assert 'UnzipComplete' in output,\
            "[FAILURE] Failed unzip: %s" % (files_name)

    def clean(self):
        """clean implement's resource file"""
        busybox_obj.clean()
        cmd = "rm -rf %s; sync;" % (self.apk_cache_dir)
        g_common_obj.adb_cmd_capture_msg(repr(cmd))

        files_name = os.path.basename(self.config.get("apk_cache_files"))
        cmd = "rm -f %s/%s; sync;" % (self.dut_download, files_name)
        g_common_obj.adb_cmd_capture_msg(repr(cmd))

        cmd = "uninstall %s" % (self.PKG_NAME)
        print g_common_obj.adb_cmd_common(cmd)

    def launch(self):
        """launch app"""
        g_common_obj.launch_app_am(self.PKG_NAME, self.MAIN_ACTIVITY)
        self._wait_init()
        self.home.skip_guide_selection()

    def _wait_init(self, timeout=60 * 15):
        """wait app initialize"""
        start_time = time.time()
        is_completed = False
        while time.time() - start_time < timeout:
            _, current_activity = common.get_current_focus_window()
            if current_activity == self.MAIN_ACTIVITY:
                is_completed = True
                break
            if self.device(text='Accept').exists:
                self.device(text='Accept').click()
            if self.device(text='OK').exists:
                self.device(text='OK').click()
            if self.device(text='Retry').exists:
                self.device(text='Retry').click()
            time.sleep(5)
        assert is_completed,\
            "[FAILURE] Server timeout: %s" % (timeout)

    def resume(self):
        """resume app"""
        cmdstr = "am start %s" % (self.PKG_NAME)
        g_common_obj.adb_cmd(cmdstr)

    def run_test(self, names, timeout=1800):
        """run test"""
        self.home.go_category_testlist()
        self.testlist.clean_chooses()

        for each in names.split(','):
            print "[Debug] test: %s" % (each)
            if self.testlist.search_item_checkbox(each).exists:
                self.testlist.search_item_checkbox(each).click()
        checked_count = self.device(className="android.widget.CheckBox", checked=True).count
        assert checked_count == len(names.split(',')),\
            "[FAILURE] Failed Choose Test item"

        self.testlist.btn_start.click()
        _, current_activity = common.get_current_focus_window()
        assert current_activity == self.TEST_ACTIVITY,\
            "[FAILURE] Tests failed to start"

        print "[Debug] start test: %s" % (names)
        start_time = time.time()
        is_completed = False
        while time.time() - start_time < timeout:
            _, current_activity = common.get_current_focus_window()
            if current_activity == self.MAIN_ACTIVITY:
                is_completed = True
                break
            time.sleep(10)
        assert is_completed,\
            "[FAILURE] Test time out: %s" % (timeout)

    def get_result(self, name):
        """get test result"""
        result_text = ''
        try:
            text1 = self.device(text=name).right(className='android.widget.TextView')
            result_text = text1.info['text']

            text2 = text1.down(className='android.widget.TextView')
            if isinstance(text2, AutomatorDeviceObject) and text2.exists:
                result_text += text2.info['text']
        except JsonRPCError:
            assert False,\
                "[FAILURE] Failed get result: %s" % (name)
        print "[Debug] Result test:%s, result:%s" % (name, result_text)
        return result_text

    def run_test_alpha_blending(self, retry=5):
        """run alpha blending test"""
        for i in range(1, retry + 1):
            self.run_test(self.TEST_Alpha_Blending)
            result = self.get_result(self.TEST_Alpha_Blending)
            if 'try again later' in result:
                print "[Debug] Retry test. count:%s" % (i)
                time.sleep(10)
                continue
            break
        match = re.compile(r'([,0-9]*)([\w+\/]*)').match(result)
        assert match,\
            "[FAILURE] Case Failed: %s" % (result)
        return match.group(1), match.group(2)

    def run_test_alpha_blending_offscreen(self, retry=5):
        """run alpha blending offscreen test"""
        for i in range(1, retry + 1):
            self.run_test(self.TEST_Alpha_Blending_Offscreen)
            result = self.get_result(self.TEST_Alpha_Blending_Offscreen)
            if 'try again later' in result:
                print "[Debug] Retry test. count:%s" % (i)
                time.sleep(10)
                continue
            break
        match = re.compile(r'([,0-9]+)([\w+\/]+)').match(result)
        assert match,\
            "[FAILURE] Case Failed: %s" % (result)
        return match.group(1), match.group(2)
