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

import time
import testlib.graphics.common as common
import xml.etree.ElementTree as ET

from testlib.graphics.glbenckmark_impl import GLBenchmarkImpl
from testlib.util.common import g_common_obj
from testlib.util.config import TestConfig
from testlib.util.repo import Artifactory
from testlib.graphics.tools import ConfigHandle


class GLBenchmarkExtendImpl(GLBenchmarkImpl):

    """ GLBenchmarkExtendImpl """
    # TEST_HD_ETC1 = 'C24Z16 ETC1'
    # EGYPT_HD = 'C24Z24MS4 Auto'
    CONFIG_FILE = 'tests.common.glbenchmark.conf'
    # PKG_NAME = "com.glbenchmark.glbenchmark25"
    PKG_NAME = "com.glbenchmark.glbenchmark27"
    # MAIN_ACTIVITY = "com.glbenchmark.activities.MainActivity"
    MAIN_ACTIVITY = "com.glbenchmark.activities.GLBenchmarkDownloaderActivity"
    # TEST_ACTIVITY = "com.glbenchmark.activities.GLBenchmarkActivity"
    TEST_ACTIVITY = "com.glbenchmark.activities.GLBRender"
    # RESULT_ACTIVITY = "com.glbenchmark.activities.ResultsActivity"
    RESULT_ACTIVITY = "com.glbenchmark.activities.GLBResults"
    RESULT_DIR = "/storage/emulated/legacy/Android/data/com.glbenchmark.glbenchmark27/cache/"
    RESULT_DIR2 = "/storage/emulated/0/Android/data/com.glbenchmark.glbenchmark27/cache/"
    RESULT_DIR3 = "/data/data/com.glbenchmark.glbenchmark27/cache/"
    BURSTFREQUENCY = "/d/mali_platform/pm_level"
    BURSTFREQUENCY2 = "/d/dri/0/i915_frequency_info | grep -i current | grep -Eo '[0-9]+'"
    BURSTFREQUENCY3 = "/sys/class/thermal/cooling_device0/cur_state"
    RESULT_KEYS = ['title', 'type', 'texture_type', 'score', 'fps', 'minfps', 'maxfps', 'uom', 'vsync_triggered', 'error', 'error_string']

    class TestResult(object):
        pass

    class HomeUI(object):

        """Home UI"""

        def __init__(self, device):
            self.device = device

        @property
        def performance_test(self):
            return self.device(resourceId="com.glbenchmark.glbenchmark27:id/listView1") \
                .child_by_text(
                "Performance Tests",
                allow_scroll_search=True,
                className="android.widget.TextView"
            )

    class TestList(object):

        """v2.7 Performance Tests List UI"""
        TESTLIST = {
                    "Performance Tests":
                        {
                            "test1": "Fill rate/C24Z16 Onscreen 888",
                            "test2": "Fill rate/C24Z16 Offscreen 888",
                            "test3": "Triangle throughput: Textured/C24Z16 Onscreen 888",
                            "test4": "Triangle throughput: Textured/C24Z16 Offscreen 888",
                            "test5": "Triangle throughput: Textured/C24Z16 Onscreen Vertex Lit 888",
                            "test6": "Triangle throughput: Textured/C24Z16 Offscreen Vertex Lit 888",
                            "test7": "Triangle throughput: Textured/C24Z16 Onscreen Fragment Lit 888",
                            "test8": "Triangle throughput: Textured/C24Z16 Offscreen Fragment Lit 888",
                            "test9": "GLBenchmark 2.7 T-Rex HD/C24Z16 Onscreen Auto",
                            "test10": "GLBenchmark 2.7 T-Rex HD/C24Z16 Offscreen Auto",
                            "test11": "GLBenchmark 2.7 T-Rex HD/C24Z16 Onscreen ETC1",
                            "test12": "GLBenchmark 2.7 T-Rex HD/C24Z16 Offscreen ETC1",
                            "test13": "GLBenchmark 2.7 T-Rex HD/C24Z16 Onscreen DXT1",
                            "test14": "GLBenchmark 2.7 T-Rex HD/C24Z16 Offscreen DXT1",
                            "test15": "GLBenchmark 2.7 T-Rex HD/C24Z16 Onscreen PVRTC4",
                            "test16": "GLBenchmark 2.7 T-Rex HD/C24Z16 Offscreen PVRTC4",
                            "test17": "GLBenchmark 2.7 T-Rex HD/C24Z16 Onscreen ETC1to565",
                            "test18": "GLBenchmark 2.7 T-Rex HD/C24Z16 Offscreen ETC1to565",
                            "test19": "GLBenchmark 2.7 T-Rex HD/C24Z24MS4 Onscreen Auto",
                            "test20": "GLBenchmark 2.7 T-Rex HD/C24Z24MS4 Offscreen Auto",
                            "test21": "GLBenchmark 2.7 T-Rex HD/C24Z16 Onscreen Fixed timestep Auto",
                            "test22": "GLBenchmark 2.7 T-Rex HD/C24Z16 Offscreen Fixed timestep Auto",
                            "test23": "GLBenchmark 2.5 Egypt HD/C24Z16 Onscreen Auto",
                            "test24": "GLBenchmark 2.5 Egypt HD/C24Z16 Offscreen Auto",
                            "test25": "GLBenchmark 2.5 Egypt HD/C24Z24MS4 Onscreen Auto",
                            "test26": "GLBenchmark 2.5 Egypt HD/C24Z24MS4 Offscreen Auto",
                            "test27": "GLBenchmark 2.5 Egypt HD/C24Z16 Onscreen Fixed timestep Auto",
                            "test28": "GLBenchmark 2.5 Egypt HD/C24Z16 Offscreen Fixed timestep Auto"
                        }
                    }

        def __init__(self, device):
            self.device = device
            self.testlist = self.TESTLIST

        def clear_choose(self):
            if self.btn_all.exists:
                self.btn_all.click()
            if self.btn_none.exists:
                self.btn_none.click()

        @property
        def onscreen(self):
            return self.device(textContains="Onscreen")

        @property
        def offscreen(self):
            return self.device(textContains="Offscreen")

        @property
        def btn_all(self):
            return self.device(className="android.widget.Button", text="All")

        @property
        def btn_none(self):
            return self.device(className="android.widget.Button", text="None")

        @property
        def btn_start(self):
            return self.device(clickable=True, text="Start")

        def search_item(self, name):
            return self.device(resourceId="com.glbenchmark.glbenchmark27:id/listView1") \
                .child_by_text(
                name,
                allow_scroll_search=True,
                className="android.widget.TextView"
            )

        def select_performance_test(self, testid=None):

            def _seperate_testid(testid=None):
                import re
                return int(re.findall(r'\d+', testid)[0])

            parent_text = str(self.testlist["Performance Tests"][testid]).split("/")[0]
            child_text = str(self.testlist["Performance Tests"][testid]).split("/")[1]
            if _seperate_testid(testid) > 22:
                self.device().scroll.toEnd()
                time.sleep(3)
                self.device(text=parent_text).down(text=child_text).click()
                time.sleep(.5)
            else:
                self.device().scroll.to(text=child_text)
                time.sleep(3)
                self.device(text=child_text).click()
                time.sleep(.5)

    def __init__(self):
        GLBenchmarkImpl.__init__(self)
        self.device = g_common_obj.get_device()
        self.home = GLBenchmarkExtendImpl.HomeUI(self.device)
        self.performc = GLBenchmarkExtendImpl.TestList(self.device)

        self.configer = TestConfig()
        self.config = self.configer.read(self.CONFIG_FILE, "GLBenchmark")
        config_handle = ConfigHandle()
        self.config["artifactory_location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat', 'sys.conf')
        self.arti = Artifactory(self.config.get('artifactory_location'))

    def setup(self):
        apk_path = self.arti.get(self.config.get("apk"))
        cmd = "install -r %s" % (apk_path)
        print g_common_obj.adb_cmd_common(cmd, 210)

    def clean(self):
        cmd = "rm -rf %s; sync;rm -rf %s; sync;" % (self.RESULT_DIR, self.RESULT_DIR2)
        uninstall_cmd = "pm uninstall %s" % self.PKG_NAME
        g_common_obj.adb_cmd_capture_msg(repr(cmd))
        g_common_obj.adb_cmd_capture_msg(repr(uninstall_cmd))

    def launch(self):
        g_common_obj.launch_app_am(self.PKG_NAME, self.MAIN_ACTIVITY)

    def resume(self):
        cmdstr = "am start %s" % (self.pkg_name)
        g_common_obj.adb_cmd(cmdstr)

    def run_performance_test_async(self, testid):
        orientation = self.device.orientation
        self.home.performance_test.click.wait(timeout=5000)
        self.performc.clear_choose()

        self.performc.select_performance_test(testid)
        self.performc.btn_start.click()
        time.sleep(5)
        _, current_activity = common.get_current_focus_window()
        assert current_activity == self.TEST_ACTIVITY, \
            "[FAILURE] Tests failed to start"

    def run_performance_test(self, testid, timeout=1800):
        orientation = self.device.orientation
        self.home.performance_test.click.wait(timeout=5000)
        self.performc.clear_choose()
        # print self.performc.onscreen.info['checked']
        # if onscreen != self.performc.onscreen.info['checked']:
        #     self.performc.onscreen.click()
        # if offscreen != self.performc.offscreen.info['checked']:
        #     self.performc.offscreen.click()
        #
        # for each in names.split(','):
        #     print "[Debug] performance test: %s" % (each)
        #     if self.performc.search_item(each).exists:
        #         self.performc.search_item(each).click()
        # checked_count = self.device(className="android.widget.CheckBox", checked=True).count
        #
        # assert checked_count == len(names.split(',')), \
        #     "[FAILURE] Failed Choose Test item"
        self.performc.select_performance_test(testid)
        self.performc.btn_start.click()
        time.sleep(5)
        _, current_activity = common.get_current_focus_window()
        assert current_activity == self.TEST_ACTIVITY, \
            "[FAILURE] Tests failed to start"

        # print "[Debug] start test: %s" % (names)
        start_time = time.time()
        is_completed = False
        while time.time() - start_time < timeout:
            g_common_obj.assert_exp_happens()
            _, current_activity = common.get_current_focus_window()
            if current_activity == self.RESULT_ACTIVITY:
                is_completed = True
                break
            time.sleep(10)
        self.device.orientation = orientation
        assert is_completed, \
            "[FAILURE] Test time out: %s" % (timeout)

    def run_performance_check_burstfrequency(self, testid, timeout=1800):
        orientation = self.device.orientation
        self.home.performance_test.click()
        self.performc.clear_choose()

        msg_flag = self.check_burstfrequency()
        print "msg_flag is %s" % (msg_flag)
        # if onscreen != self.performc.onscreen.info['checked']:
        #     self.performc.onscreen.click()
        # if offscreen != self.performc.offscreen.info['checked']:
        #     self.performc.offscreen.click()
        #
        # for each in names.split(','):
        #     print "[Debug] performance test: %s" % (each)
        #     if self.performc.search_item(each).exists:
        #         self.performc.search_item(each).click()
        # time.sleep(3)
        # checked_count = self.device(className="android.widget.CheckBox", checked=True).count
        #
        # assert checked_count == len(names.split(',')), \
        #     "[FAILURE] Failed Choose Test item"
        self.performc.select_performance_test(testid)
        self.performc.btn_start.click()
        time.sleep(5)
        _, current_activity = common.get_current_focus_window()
        assert current_activity == self.TEST_ACTIVITY, \
            "[FAILURE] Tests failed to start"

        # print "[Debug] start test: %s" % (names)
        start_time = time.time()
        is_completed = False
        while time.time() - start_time < timeout:
            g_common_obj.assert_exp_happens()
            msg = self.check_burstfrequency()
            if msg != msg_flag:
                is_completed = True
                break
            time.sleep(3)
        self.device.orientation = orientation
        assert is_completed, \
            "[FAILURE] Test burst frequency fail:msg is %s" % (msg)

    def check_all_results(self):
        cmd = "cat %slast_results_*" % (self.RESULT_DIR)
        cmd2 = "cat %slast_results_*" % (self.RESULT_DIR2)
        cmd3 = "cat %slast_results_*" % (self.RESULT_DIR3)
        msg = g_common_obj.adb_cmd_capture_msg(repr(cmd))
        if msg.find("No such file or directory") != -1 or msg == '':
            msg = g_common_obj.adb_cmd_capture_msg(repr(cmd2))
            if msg.find("No such file or directory") != -1 or msg == '':
                msg = g_common_obj.adb_cmd_capture_msg(repr(cmd3))
        assert len(msg), \
            "[FAILURE] Failed get result xml: %s" % (cmd)
        results = self.parse_reulsts(msg)
        def is_error(x):
            return x['error'] != '0'
        errors = [item for item in results if is_error(item)]

        assert len(errors) == 0, \
            "[FAILURE] Error found in results: %s" % (msg)
        return results

    def parse_reulsts(self, str_xml):
        results = []

        def parse_items(child):
            items = {}
            for item in child:
                items[item.tag] = item.text
            assert sorted(items.keys()) == sorted(self.RESULT_KEYS), \
                "[FAILURE] Failed parsing result"
            return items

        parser = ET.XMLParser(encoding="utf-8")
        root = ET.fromstring(str_xml, parser=parser)
        for child in root:
            results.append(parse_items(child))

        return results

    def check_burstfrequency(self):
        cmd = "cat %s" % (self.BURSTFREQUENCY)
        cmd2 = "cat %s" % (self.BURSTFREQUENCY2)
        cmd3 = "cat %s" % (self.BURSTFREQUENCY3)
        msg = g_common_obj.adb_cmd_capture_msg(repr(cmd))
        if msg.find("No such file or directory") != -1 or msg == '':
            msg = g_common_obj.adb_cmd_capture_msg(repr(cmd2))
            if msg.find("No such file or directory") != -1 or msg == '':
                msg = g_common_obj.adb_cmd_capture_msg(repr(cmd3))
        print "cat msg is %s" % (msg)
        return msg
