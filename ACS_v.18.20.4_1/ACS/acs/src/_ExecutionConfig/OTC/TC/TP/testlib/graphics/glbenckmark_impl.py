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

'''
@summary: Class for GLBenchmark 2.5.1 APK UI Operation
@since: 10/08/2014
@author: Grace Yi
'''
import os
import time
from testlib.util.common import g_common_obj

class Locator(object):

    def __init__(self, device):
        self._device = device

    @property
    def onscreen(self):
        return self._device(text="Onscreen")

    @property
    def offscreen(self):
        return self._device(text="Offscreen")

    @property
    def start(self):
        return self._device(text="Start")

    @property
    def performance_tests(self):
        return self._device(text="Performance Tests")

    # @property
    # def egypt_hd_etc1(self):
    #     return self._device(className="android.widget.CheckBox",  instance=4)

    @property
    def wait_exist(self):
        """ wait until the ui object exist """
        def _wait(uiobj, timeout=60):
            return uiobj.wait("exists", timeout*1000)
        return _wait

    @property
    def wait_gone(self):
        """ wait until the ui object gone """
        def _wait(uiobj, timeout=60):
            return uiobj.wait("gone", timeout*1000)
        return _wait

class TimeoutError(Exception): pass

class GLBenchmarkImpl:
    '''
    classdocs
    '''
    # apk_name = "GLBenchmark_2.5.1.apk"
    # pkg_name = "com.glbenchmark.glbenchmark25"
    apk_name = "GLBenchmark_2_7_0.apk"
    pkg_name = "com.glbenchmark.glbenchmark27"
    activity_name ="com.glbenchmark.activities.GLBenchmarkDownloaderActivity"
    result_folder = "/storage/emulated/0/Android/data/com.glbenchmark.glbenchmark27/cache/"

    def __init__(self):
        self._device = g_common_obj.get_device()
        self._locator = Locator(self._device)
        self.base_path = os.path.split(os.path.realpath(__file__))[0].split(os.sep)
        self.apk_file = (os.sep).join\
        (self.base_path+['GLBenchmark_2_7_0.apk'])

    def install(self):
        """
        @summary: install ll apk
        """
        print "[INFO] Install apk"
        target_path = (os.sep).join(self.base_path) + "/"
        file_path = "/data/local/tmp/" + GLBenchmarkImpl.apk_name
        cmd = "pull " + file_path + " " + target_path
        print cmd
        g_common_obj.adb_cmd_common(cmd)
        self._device.watcher("Accept").\
        when(textMatches="Accept!").click(textMatches="Accept!")
        cmd = "install " + self.apk_file
        message = g_common_obj.adb_cmd_common(cmd)
        print "Install message:", message
        assert "Success" in message, message

    def uninstall(self):
        """
        @summary: uninstall ll apk
        """
        print "[INFO] Uninstall apk"
        cmd = "uninstall " + self.pkg_name
        message = g_common_obj.adb_cmd_common(cmd, 120)
        print "Uninstall message:", message
        assert "Success" in message, message

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

    def launch_app_am(self):
        """ Launch GLBenchmark via adb am command
        """
        print "Launch GLBenchmark by adb am"
        g_common_obj.launch_app_am(\
            GLBenchmarkImpl.pkg_name, GLBenchmarkImpl.activity_name)
        self._locator.wait_exist(self._locator.performance_tests)

    @staticmethod
    def stop_app_am():
        """ Stop GLBenchmark via adb am command
        """
        print "Stop GLBenchmark by adb am"
        g_common_obj.stop_app_am(GLBenchmarkImpl.pkg_name)

    def enter_performance_tests(self):
        """ Click "Performance Tests" on Application home screen
        """
        print "Enter Performance Tests"
        self._locator.performance_tests.click()
        # self._locator.wait_exist(self._locator.onscreen)
        self._locator.wait_exist(self._locator.start)

    # def run_egypt_hd_etc1(self, onscreen):
    #     """ Run Egypt HD ETC1 benchmark on device
    #     @param: boolean
    #         onscreen = True, run benchmark with Onscreen
    #         onscreen = False, run benchmark with Offscreen
    #     """
    #     print "Run GLBenchamark Egypt HD ETC1"
    #     self.enter_performance_tests()
    #     if self._locator.egypt_hd_etc1.info['checked'] is False:
    #         self._locator.egypt_hd_etc1.click()
    #
    #     if onscreen:
    #         if self._locator.onscreen.info['checked'] is False:
    #             self._locator.onscreen.click()
    #         if self._locator.offscreen.info['checked'] is True:
    #             self._locator.offscreen.click()
    #     else:
    #         if self._locator.offscreen.info['checked'] is False:
    #             self._locator.offscreen.click()
    #         if self._locator.onscreen.info['checked'] is True:
    #             self._locator.onscreen.click()
    #     # start test
    #     self.__click_start()

    def clear_result_file(self):
        """ Clear result files
        """
        print "Clear result files"
        clear_cmd = "rm -f %s/results_*.xml; echo $?" % GLBenchmarkImpl.result_folder[0:-1]
        result_list = g_common_obj.adb_cmd_capture_msg(clear_cmd).split('\r\n')
        msg_fail = "Clear result files fail"
        if len(result_list) > 0:
            ret = result_list[-1].rstrip()
            assert int(ret) == 0, msg_fail
        else:
            assert False, msg_fail

    def wait_running(self, waittime):
        """ Wait running complete
        check there is a result file under GLBenchmark result folder
        """
        print "Wait running complete"
        loop = 1
        cmd = 'ls %s/results_*.xml; echo $?' % GLBenchmarkImpl.result_folder[0:-1]
        msg_find_result_fail = "Look for result file fail"
        duration = waittime
        starttime = time.time()
        while waittime > 0:
            result_list = g_common_obj.adb_cmd_capture_msg(cmd).split('\r\n')
            if len(result_list) > 0:
                ret = result_list[-1].rstrip()
                assert int(ret) == 0, msg_find_result_fail
                if 'No such file or directory' in ''.join(result_list):
                    print "Running not complete iteration %d" % loop
                elif 'results_' in ''.join(result_list):
                    break
            else:
                assert False, msg_find_result_fail
            loop += 1
            time.sleep(10)
            waittime -= 10
        endtime = time.time()
        print "Elapse %.3fs" % (endtime-starttime)
        if waittime <= 0:
            raise TimeoutError("Running not complete within %ds" % duration)

    def __click_start(self):
        """ Click Start Button
        """
        if self._locator.start.info['enabled'] is False:
            assert False, "Start button is disable"
        self._locator.start.click()
