# -*- coding: utf-8 -*-
'''
Created on 01/21/2016
@author: Zhao Xiangyi
'''

import time
import thread
import os
from testlib.util.common import g_common_obj
from testlib.util.uiatestbase import UIATestBase
from testlib.util.config import TestConfig
from testlib.util.repo import Artifactory
from testlib.graphics.tools import ConfigHandle
from testlib.common.base import getTmpDir, clearTmpDir
from testlib.util.process import shell_command
from testlib.graphics.extend_systemui_impl import SystemUiExtendImpl
from testlib.graphics.extend_glbenckmark_impl import GLBenchmarkExtendImpl
from testlib.graphics.glbenckmark_impl import GLBenchmarkImpl
import re


class VsyncFPSTool(UIATestBase):

    @classmethod
    def setUpClass(cls):
        print "[setUpClass]: %s" % cls.__name__
        super(VsyncFPSTool, cls).setUpClass()
        cls.d = g_common_obj.get_device()
        g_common_obj.root_on_device()
        g_common_obj.remount_device()
        config = TestConfig()
        cfg_file = 'tests.tablet.artifactory.conf'
        cfg_arti = config.read(cfg_file, 'artifactory')
        config_handle = ConfigHandle()
        cfg_arti["location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat/', 'sys.conf')
        # cfg_apk = config.read(cfg_file, 'content_gears')
        cfg_apk = config.read(cfg_file, 'content_graphic')
        arti = Artifactory(cfg_arti.get('location'))
        binary_name = cfg_apk.get("name")
        file_path = arti.get(binary_name)
        # result = config_handle.check_apps("com.wt.fpstest")
        result = config_handle.check_apps("com.glbenchmark.glbenchmark27")
        if result == 0:
            g_common_obj.adb_cmd_common('install ' + file_path)

    def setUp(self):
        print "[Setup]:%s" % self.__class__.__name__
        super(VsyncFPSTool, self).setUp()
        self.tmpdir = getTmpDir()
        self.systemui = SystemUiExtendImpl()
        self.benchmark = GLBenchmarkExtendImpl()
        self._glBenchmark = GLBenchmarkImpl()

    def tearDown(self):
        print "[tearDown]:%s" % self.__class__.__name__
        super(VsyncFPSTool, self).tearDown()
        clearTmpDir()

    @classmethod
    def tearDownClass(cls):
        print "[tearDownClass]: %s" % cls.__name__
        super(VsyncFPSTool, cls).tearDownClass()

    # def setup_FPSTool(self):
    #     self.cfg_apk = TestConfig().read('tests.tablet.artifactory.conf', "content_gears")
    #     folder_name = self.cfg_apk.get("folder_name")
    #     config_handle = ConfigHandle()
    #     self.cfg_apk["artifactory_location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat', 'sys.conf')
    #     arti = Artifactory(self.cfg_apk.get('artifactory_location'))
    #     folder_path = arti.get(self.cfg_apk.get("fpst_folder"))
    #     print "fpst folder", folder_path
    #     bin_folder = folder_path.replace(".tar.gz", "")
    #     os.system("cd " + os.path.dirname(folder_path) +
    #         ";tar xzf " + folder_path)
    #     os.system("mv " + bin_folder + " %s/%s" % (self.tmpdir, folder_name))
    #     os.system("cd %s/%s" % (self.tmpdir, folder_name) + ";ls")
    #     print "tmpdir path: %s/%s" % (self.tmpdir, folder_name)
    #     cmd = "python %s/%s/FPStool.py SurfaceView" % (self.tmpdir, folder_name)
    #     print cmd
    #     _, info = shell_command(cmd, 90)
    #     for line in info:
    #         fps = 60
    #         line = line.strip('\r\n').strip()
    #         if line[0].isdigit():
    #             fps = float(line)
    #             print fps
    #             assert fps > 56.00, "%s fps is lower than spec!" % (line)
    #
    def capture_state(self, timeout):
        result = 0
        t = 0
        while t < timeout:
            if result >= 0:
                cmd = "dumpsys SurfaceFlinger | grep 'VSYNC state'"
                msg = g_common_obj.adb_cmd_capture_msg(repr(cmd))
                result = msg.find("disabled")
                print result
                t += 1
                time.sleep(0.1)
            else:
                print result
                break
        time.sleep(2)
        thread.exit_thread()
        assert result >= 0, "Can not catch VSYNC state: enable!"
    #
    # def launch_gears(self, sleeptime):
    #     print """Launch GearsforAndroid at the same time
    #     """
    #     g_common_obj.launch_app_am("com.wt.fpstest", "com.wt.fpstest.activities.MainActivity")
    #     time.sleep(sleeptime)
    #     thread.exit_thread()

    def test_Vsync_FPSTool(self):
        """
        test_Vsync_FPSTool

        Steps:
        1. Quickly swipe hoemscreen and immediately execute command:
         adb shell dumpsys SurfaceFlinger | grep "VSYNC state", try again if it is "disabled".
        2. Launch GLbenchmark 2.7 and start performance test at the same time.
        3. After finished test, check FPS result from result xml.
        """
        print "[RunTest]: %s" % self.__str__()

        print """[Step] 1. adb shell dumpsys SurfaceFlinger | grep "VSYNC state"
        """
        thread.start_new_thread(self.capture_state, (30,))
        self.benchmark.launch()

        # for _ in range(0, 3):
        #     _, activity = self.systemui.get_current_focus()
        #     if not activity == "com.wt.fpstest.activities.MainActivity":
        #         print "[DEBUG]Launch FpsTest.apk failed, Trying to relaunch it."
        #         g_common_obj.launch_app_am("com.wt.fpstest", "com.wt.fpstest.activities.MainActivity")
        #     else:
        #         break
        self.benchmark.run_performance_test("test23")
        print """[Step] 2. Catch fps value after finish test.
        """
        result = self.benchmark.check_all_results()[0]['fps']
        fps_val = int(re.findall(r'\d+', result)[0])
        print "fps result is: %d." % fps_val
        assert fps_val >= 55,\
            "Test failed, fps is: %d." % fps_val
        g_common_obj.assert_exp_happens()
