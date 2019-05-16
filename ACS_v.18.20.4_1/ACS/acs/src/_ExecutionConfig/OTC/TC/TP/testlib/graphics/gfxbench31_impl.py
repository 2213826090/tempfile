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

from testlib.graphics.common import busybox_obj, pkgmgr, FileSystem
import testlib.graphics.common as common
from testlib.graphics.glbenckmark_impl import GLBenchmarkImpl
from testlib.util.common import g_common_obj
from testlib.util.config import TestConfig
from testlib.util.repo import Artifactory
from testlib.graphics.tools import ConfigHandle


class Gfxbench31Impl(GLBenchmarkImpl):

    """ Gfxbench31Impl """

    CONFIG_FILE = 'tests.common.gfxbench31.conf'
    # PKG_NAME = "net.kishonti.gfxbench.gl"
    # INIT_ACTIVITY = "net.kishonti.benchui.initialization.InitActivity"
    # MAIN_ACTIVITY = "net.kishonti.app.MainActivity"
    TEST_ACTIVITY = "net.kishonti.gfxbench.TfwActivityEx"
    PKG_NAME = "com.glbenchmark.glbenchmark27.corporate"
    MAIN_ACTIVITY = "net.kishonti.gfxbench.GfxMainActivity"
    RUN_ACTIVITY = "net.kishonti.benchui.corporate.SplashScreenCorporate"

    TEST_GROUPS = ['High-Level Tests', 'Low-Level Tests', 'Special Tests']
    TEST_TRex = 'T-Rex'
    TEST_Manhattan = 'Manhattan'
    TEST_ManhattanES31 = 'Manhattan ES 3.1'

    FREQUENCY_INFO = "/d/dri/0/i915_frequency_info"

    class HomeUI(object):

        """Home UI"""

        def __init__(self, device):
            self.device = device

        @property
        def main_circle_control(self):
            return self.device(resourceId='net.kishonti.gfxbench.gl:id/main_circleControl')

        def skip_guide_selection(self):
            """skip selection guide"""
            # for _ in range(10):
            #     if not self.device().scroll.horiz.backward(steps=5):
            #         break
            if self.main_circle_control.exists:
                bounds = self.main_circle_control.info['visibleBounds']
                for i in range(10):
                    self.device.click(
                        bounds['left'] + (bounds['right'] - bounds['left']) * 0.75,
                        bounds['bottom'] - 20 * i)
                    if self.device(text='Test select').exists:
                        break
            assert self.device(text='Test select').exists,\
                "[FAILURE] Failed skip_guide"
            self.device(text='Test select').click.wait()

        def go_category_testlist(self):
            """go to test list category"""
            self.device().scroll.to(text='Test select')

    class TestList(object):

        """Tests List UI"""

        def __init__(self, device):
            self.device = device

        @property
        def btn_start(self):
            return self.device(text='Start')

        # def clean_chooses(self):
        #     """check off all checkbox"""
        #     for each in Gfxbench31Impl.TEST_GROUPS:
        #         ui_checkbox = self.search_group_checkbox(each)
        #         if ui_checkbox.exists:
        #             check = ui_checkbox.info['checked']
        #             if check == True:
        #                 ui_checkbox.click()
        #             else:
        #                 ui_checkbox.click()
        #                 ui_checkbox.click()

        def search_group_checkbox(self, name):
            """scroll find group checkbox"""
            assert self.device().scroll.to(text=name),\
                "[FAILURE] Not found %s group" % (name)
            return self.device(text=name).right(className="android.widget.ImageView")

        def search_item_onscreen_checkbox(self, name):
            """scroll find item onscreen checkbox"""
            # y = self.device.info["displayHeight"]
            # x = self.device.info["displayWidth"]
            # for _ in range(10):
            #     self.device.swipe(x / 2, y / 2, x / 2, y, steps=5)
            # assert self.device().scroll.to(text=name),\
            #     "[FAILURE] Not found %s item" % (name)
            return self.device(text=name)\
                .right(className="android.widget.RelativeLayout", index=1)

        def search_item_offscreen_checkbox(self, name):
            """scroll find item offscreen checkbox"""
            # y = self.device.info["displayHeight"]
            # x = self.device.info["displayWidth"]
            # for _ in range(10):
            #     self.device.swipe(x / 2, y / 2, x / 2, y, steps=5)
            # assert self.device(scrollable=True).scroll.to(text=name),\
            #     "[FAILURE] Not found %s item" % (name)
            return self.device(text=name)\
                .right(className="android.widget.RelativeLayout", index=2)

    def __init__(self):
        GLBenchmarkImpl.__init__(self)
        self.device = g_common_obj.get_device()
        self.home = Gfxbench31Impl.HomeUI(self.device)
        self.testlist = Gfxbench31Impl.TestList(self.device)

        self.configer = TestConfig()
        self.config = self.configer.read(self.CONFIG_FILE, "Gfxbench31Impl")
        config_handle = ConfigHandle()
        self.config["artifactory_location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat', 'sys.conf')
        self.arti = Artifactory(self.config.get('artifactory_location'))
        self.apk_cache_dir = self.config.get("apk_cache_dir")
        self.dut_download = self.config.get("dut_download")
        self.fs = FileSystem()

    def setup(self):
        """setup implement's resource file"""
        busybox_obj.setup()

        if not pkgmgr._package_installed(pkgName=self.PKG_NAME):
            file_path = self.config.get("apk")
            apk_path = self.arti.get(file_path)
            pkgmgr.apk_install(apk_path)

        # cmd = "cd %s || mkdir -p %s" % (self.apk_cache_dir, self.apk_cache_dir)
        # g_common_obj.adb_cmd_capture_msg(repr(cmd))
        # apk_cache_files = self.arti.get(self.config.get("apk_cache_files"))
        # ret = g_common_obj.push_file(apk_cache_files, self.dut_download)
        # assert ret, 'Failed push %s' % (apk_cache_files)
        # files_name = os.path.basename(apk_cache_files)
        # cmd = "busybox tar zxf %s/%s --directory %s > /dev/null 2>&1 && print UnzipComplete"\
        #     % (self.dut_download, files_name, self.apk_cache_dir)
        # output = busybox_obj.adb_busybox_cmd(repr(cmd), 60 * 10)
        # assert 'UnzipComplete' in output,\
        #     "[FAILURE] Failed unzip: %s" % (files_name)

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
        g_common_obj.launch_app_am(self.PKG_NAME, self.RUN_ACTIVITY)
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

    def run_test(self, names, timeout=1800, onscreen=True, offscreen=False):
        """run test"""
        self.home.go_category_testlist()
        # self.testlist.clean_chooses()

        for each in names.split(','):
            print "[Debug] test: %s onscreen:%s offscreen:%s" % (each, onscreen, offscreen)
            if onscreen:
                onscreen_checkbox = self.testlist.search_item_onscreen_checkbox(each)
                onscreen_checkbox.click()
            if offscreen:
                offscreen_checkbox = self.testlist.search_item_offscreen_checkbox(each)
                offscreen_checkbox.click()
        # checked_count = self.device(className="android.widget.CheckBox", checked=True).count
        # assert checked_count == len(names.split(',')),\
        #     "[FAILURE] Failed Choose Test item"

        cmd = "sync; echo 3 > /proc/sys/vm/drop_caches"
        g_common_obj.adb_cmd_capture_msg(repr(cmd))

        self.testlist.btn_start.click()
        time.sleep(8)
        print "[Debug] start test: %s" % (names)
        _, current_activity = common.get_current_focus_window()
        assert current_activity == self.TEST_ACTIVITY,\
            "[FAILURE] Tests failed to start"

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

    def get_onscreen_result(self, name):
        """get test result"""
        result_text = ''
        try:
            text1 = self.device(text=name)\
                .right(className="android.widget.RelativeLayout", index=1)\
                .child(className="android.widget.TextView")
            result_text += text1.info['text']

            text2 = text1.down(className='android.widget.TextView')
            if isinstance(text2, AutomatorDeviceObject) and text2.exists:
                result_text += text2.info['text']
        except JsonRPCError:
            assert False,\
                "[FAILURE] Failed get result: %s" % (name)
        print "[Debug] Result test:%s, result:%s" % (name, result_text)
        return result_text

    def get_offscreen_result(self, name):
        """get test result"""
        result_text = ''
        try:
            text1 = self.device(text=name)\
                .right(className="android.widget.RelativeLayout", index=2)\
                .child(className="android.widget.TextView")
            result_text += text1.info['text']

            text2 = text1.down(className='android.widget.TextView')
            if isinstance(text2, AutomatorDeviceObject) and text2.exists:
                result_text += text2.info['text']
        except JsonRPCError:
            assert False,\
                "[FAILURE] Failed get result: %s" % (name)
        print "[Debug] Result test:%s, result:%s" % (name, result_text)
        return result_text

    def run_test_trex_onscreen(self, retry=5):
        """run_test_trex_onscreen"""
        for i in range(1, retry + 1):
            self.run_test(self.TEST_TRex, onscreen=True)
            result = self.get_onscreen_result(self.TEST_TRex)
            if 'try again later' in result:
                print "[Debug] Retry test. count:%s" % (i)
                time.sleep(10)
                continue
            break
        match = re.compile(r'([,0-9]+)([\w+\/]+)').match(result)
        assert match,\
            "[FAILURE] Case Failed: %s" % (result)
        return match.group(1), match.group(2)

    def run_test_trex_offscreen(self, retry=5):
        """run_test_trex_offscreen"""
        for i in range(1, retry + 1):
            self.run_test(self.TEST_TRex, onscreen=False, offscreen=True)
            result = self.get_offscreen_result(self.TEST_TRex)
            if 'try again later' in result:
                print "[Debug] Retry test. count:%s" % (i)
                time.sleep(10)
                continue
            break

        match = re.compile(r'([,0-9]+)([\w+\/]+)').match(result)
        assert match,\
            "[FAILURE] Case Failed: %s" % (result)
        return match.group(1), match.group(2)

    def run_test_manhattan31_onscreen(self, retry=5):
        """run_test_manhattan31_onscreen"""
        for i in range(1, retry + 1):
            self.run_test(self.TEST_ManhattanES31, onscreen=True)
            result = self.get_onscreen_result(self.TEST_ManhattanES31)
            if 'try again later' in result:
                print "[Debug] Retry test. count:%s" % (i)
                time.sleep(10)
                continue
            break
        match = re.compile(r'([,0-9]+)([\w+\/]+)').match(result)
        assert match,\
            "[FAILURE] Case Failed: %s" % (result)
        return match.group(1), match.group(2)

    def run_test_manhattan31_offscreen(self, retry=5):
        """run_test_manhattan31_offscreen"""
        for i in range(1, retry + 1):
            self.run_test(self.TEST_ManhattanES31, onscreen=False, offscreen=True)
            result = self.get_offscreen_result(self.TEST_ManhattanES31)
            if 'try again later' in result:
                print "[Debug] Retry test. count:%s" % (i)
                time.sleep(10)
                continue
            break

        match = re.compile(r'([,0-9]+)([\w+\/]+)').match(result)
        assert match,\
            "[FAILURE] Case Failed: %s" % (result)
        return match.group(1), match.group(2)

    def run_test_manhattan_onscreen(self, retry=5):
        """run_test_manhattan_onscreen"""
        for i in range(1, retry + 1):
            self.run_test(self.TEST_Manhattan, onscreen=True)
            result = self.get_onscreen_result(self.TEST_Manhattan)
            if 'try again later' in result:
                print "[Debug] Retry test. count:%s" % (i)
                time.sleep(10)
                continue
            break
        match = re.compile(r'([,0-9]+)([\w+\/]+)').match(result)
        assert match,\
            "[FAILURE] Case Failed: %s" % (result)
        return match.group(1), match.group(2)

    def run_test_manhattan_offscreen(self, retry=5):
        """run_test_manhattan_offscreen"""
        for i in range(1, retry + 1):
            self.run_test(self.TEST_Manhattan, onscreen=False, offscreen=True)
            result = self.get_offscreen_result(self.TEST_Manhattan)
            if 'try again later' in result:
                print "[Debug] Retry test. count:%s" % (i)
                time.sleep(10)
                continue
            break

        match = re.compile(r'([,0-9]+)([\w+\/]+)').match(result)
        assert match,\
            "[FAILURE] Case Failed: %s" % (result)
        return match.group(1), match.group(2)

    def start_test(self, names, onscreen=True, offscreen=False):
        """Start test, but won't finish it"""
        self.home.go_category_testlist()

        for each in names.split(','):
            print "[Debug] test: %s onscreen:%s offscreen:%s" % (each, onscreen, offscreen)
            if onscreen:
                onscreen_checkbox = self.testlist.search_item_onscreen_checkbox(each)
                onscreen_checkbox.click()
            if offscreen:
                offscreen_checkbox = self.testlist.search_item_offscreen_checkbox(each)
                offscreen_checkbox.click()

        self.testlist.btn_start.click()
        print "[Debug] start test: %s" % (names)
        end_time = 60
        cur_time = time.time()
        while time.time() - cur_time < end_time:
            _, current_activity = common.get_current_focus_window()
            if current_activity == self.TEST_ACTIVITY:
                break
            time.sleep(1)
        time.sleep(12)

    def get_current_frequency(self):
        ret = self.fs.get_file_context(True, self.FREQUENCY_INFO, 'Current\ f')
        pattern = re.compile(r'\d+')
        return int(pattern.search(ret).group(0))
