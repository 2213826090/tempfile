# -*- coding:utf-8 -*-

'''
@summary: Android memory track test.
@since: 07/04/2016
@author: Lijin Xiong
'''

import signal
import os,time,re
import subprocess
from threading import Thread
from testlib.util.uiatestbase import UIATestBase
from testlib.util.common import g_common_obj
from testlib.androidframework.fetch_resources import resource
from testlib.androidframework.common import AdbUtils,UiAutomatorUtils,ShellUtils,Environment,InstrumentationInterface
from testlib.util.log import Logger


LOG = Logger.getlogger(__name__)

### OS
OS_OPEN_JUST_ONCE_TXT = "Just once"
APP_CRASH_POPUP_BEGIN_TXT = "Unfortunately"
APP_CRASH_POPUP_END_TXT = "has stopped"

### Youtube
YOUTUBE_PACKAGE_NAME = "com.google.android.youtube"

class MemTrack(UIATestBase):

    youtube_sample_video = "http://www.youtube.com/watch?v=YRhFSWz_J3I"

    def setUp(self):
        super(MemTrack, self).setUp()
        self.__test_name = __name__
        print "Setup: %s" % self.__test_name
        self.d = g_common_obj.get_device()
        UiAutomatorUtils.unlock_screen()
        for i in ["device_ui", "jank_test"]:
            _apk_path = resource.get_resource_from_atifactory\
            ("tests.tablet.artifactory.conf", "CTS_APKS", i)
            g_common_obj.adb_cmd_common("install -r %s" % _apk_path)
        _bin_path = resource.get_resource_from_atifactory\
            ("tests.tablet.artifactory.conf", "BIN", "memtrack")
        g_common_obj.adb_cmd_common("push  %s /data/" % _bin_path)

    def test_dumpsys_memtrack_reporting(self):
        dumpsys_cmd = "dumpsys meminfo"
        output = AdbUtils.run_adb_cmd(dumpsys_cmd)
        self.assertTrue("Total PSS by category:" in output)
        self.assertTrue(": Dalvik" in output and ": Native" in output and ": Ashmem" in output and
                        ": GL" in output)
        self.assertTrue("Total RAM:" in output and "Free RAM:" in output and "Used RAM:" in output and
                        "Lost RAM:" in output)

    def test_long_loop_read_of_memtrack(self):
        null_pipe = open(os.devnull, 'w')
        AdbUtils._run_adb_cmd("dmesg -c", add_ticks=False)
        long_loop_read_proc_cmd = 'for i in 1 2 3 4 5 6 7 8 9 10; do while :; ' \
                                  'do cat /sys/class/drm/card0/gfx_memtrack/* > /dev/null; done & done'
        long_loop_proc = subprocess.Popen(long_loop_read_proc_cmd, shell=True, stdout=null_pipe,
                                          stderr=subprocess.STDOUT)
        try:
            # launch youtube and watch a video
            UiAutomatorUtils.launch_activity_with_data_uri(MemTrack.youtube_sample_video)
            if self.d(text=OS_OPEN_JUST_ONCE_TXT).wait.exists(timeout=3000):
                self.d(text=OS_OPEN_JUST_ONCE_TXT).click()
            self.assertTrue(self.d(packageName=YOUTUBE_PACKAGE_NAME).wait.exists(timeout=10000))
            # let the youtube play for some time
            time.sleep(10)
            # check for potential errors
            error_msgs_cmd = "dmesg"
            output = AdbUtils._run_adb_cmd(error_msgs_cmd, add_ticks=False)
            self.assertTrue(" trace " not in output)
            self.assertTrue(" bug " not in output)
            self.assertTrue(" stack " not in output)
        finally:
            long_loop_proc.terminate()
            os.kill(long_loop_proc.pid, signal.SIGTERM)
            ShellUtils.kill_processes_containing_string("gfx_memtrack")

    def test_loop_read_of_memtrack(self):
        egl_mtrack_dumpsys_meminfo = AdbUtils.run_adb_cmd('dumpsys meminfo | grep "EGL mtrack"')
        LOG.info("EGL mtrack: " + egl_mtrack_dumpsys_meminfo)
        self.assertTrue(" 0 " not in egl_mtrack_dumpsys_meminfo)
        AdbUtils._run_adb_cmd("dmesg -c", add_ticks=False)
        loop_read_cmd = 'for i in 1 2 3 4 5 6 7 8 9 10; do sh -c "cat /sys/class/drm/card0/gfx_memtrack/*" & done'
        output = AdbUtils.run_adb_cmd(loop_read_cmd)
        if Environment.file_not_found_cmd_output in output:
            return
        self.assertTrue("No such file or directory" not in output)
        self.assertTrue("Total used GFX Shmem" in output)
        error_msgs_cmd = "dmesg"
        output = AdbUtils._run_adb_cmd(error_msgs_cmd, add_ticks=False)
        self.assertTrue(" trace " not in output)
        self.assertTrue(" bug " not in output)
        self.assertTrue(" stack " not in output)

    memtrack_loop_cmd = "while :; do cat /sys/class/drm/card0/gfx_memtrack/i915_gem_meminfo > /dev/null; done"
    memtrack_proc = None
    jank_test_output = None

    def test_memtrack_effect_framerate(self):

        def run_memtrack():
            global memtrack_proc
            MemTrack.memtrack_proc = AdbUtils.get_adb_cmd_process(MemTrack.memtrack_loop_cmd)

        def run_jank_test():
            jank_runner = InstrumentationInterface()
            MemTrack.jank_test_output = jank_runner.run_instrumentation("android.cts.jank.ui.CtsDeviceJankUi", "",
                                                      "android.cts.jank/android.support.test.runner.AndroidJUnitRunner")

        # must stop python uiautomator in order to run the jank test
        AdbUtils.kill_python_uiautomator_rpc_server_on_dut()

        # initial run of jank test
        run_jank_test()
        LOG.info("initial run of the jank test yielded: " + MemTrack.jank_test_output)
        self.assertTrue(InstrumentationInterface.was_instrumentation_test_successful(MemTrack.jank_test_output),
                        "initial run of the jank test was not successful")
        fps_values = re.findall("\|fps\|(\d+\.\d+)", MemTrack.jank_test_output)
        self.assertTrue(len(fps_values) > 0, "could not find fps information in jank test output")
        initial_fps_value = float(fps_values[0])

        UiAutomatorUtils.close_all_tasks()
        # must stop python uiautomator in order to run the jank test
        AdbUtils.kill_python_uiautomator_rpc_server_on_dut()

        # subsequent run of the jank test, with memtrack running along side
        memtrack_thread = Thread(target=run_memtrack)
        jank_thread = Thread(target=run_jank_test)
        LOG.info("Starting memtrack")
        memtrack_thread.start()
        time.sleep(0.5)  # allow memtrack to start before running the jank test
        LOG.info("Starting jank ui on dut")
        jank_thread.start()
        jank_thread.join()
        MemTrack.memtrack_proc.kill()  # kill memtrack so the thread will join
        memtrack_thread.join()

        LOG.info("second run of the jank test yielded: " + MemTrack.jank_test_output)
        self.assertTrue(InstrumentationInterface.was_instrumentation_test_successful(MemTrack.jank_test_output),
                        "second run of the jank test was not successful")
        fps_values = re.findall("\|fps\|(\d+\.\d+)", MemTrack.jank_test_output)
        self.assertTrue(len(fps_values) > 0, "could not find fps information in jank test output")
        second_fps_value = float(fps_values[0])

        LOG.info("initial fps value: %s, second fps value: %s" % (initial_fps_value, second_fps_value))
        self.assertTrue(second_fps_value < initial_fps_value, "fps when running memtrack was not smaller than "
                                                              "fps without running memtrack")

    def test_memtrack_test_from_android_tree(self):
        root_cmd = "root"
        AdbUtils.run_adb_cmd(root_cmd, adb_shell=False)
        set_permissions_cmd = "chmod 755 /data/memtrack_test"
        AdbUtils.run_adb_cmd(set_permissions_cmd)
        run_memtrack_cmd = "/data/memtrack_test"
        output = AdbUtils.run_adb_cmd(run_memtrack_cmd)
        #assert "/system/bin/" in output
        assert "init" in output
        # assert the output has at least 2 lines
        lines = output.splitlines()
        assert len(lines) > 2
        # assert that a line has 8 columns
        assert len(lines[0].split()) == 8