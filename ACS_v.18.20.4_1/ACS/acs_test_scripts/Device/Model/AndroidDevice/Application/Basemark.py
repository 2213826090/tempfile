"""
:copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
The source code contained or described here in and all documents related
to the source code ("Material") are owned by Intel Corporation or its
suppliers or licensors. Title to the Material remains with Intel Corporation
or its suppliers and licensors. The Material contains trade secrets and
proprietary and confidential information of Intel or its suppliers and
licensors.

The Material is protected by worldwide copyright and trade secret laws and
treaty provisions. No part of the Material may be used, copied, reproduced,
modified, published, uploaded, posted, transmitted, distributed, or disclosed
in any way without Intel's prior express written permission.

No license under any patent, copyright, trade secret or other intellectual
property right is granted to or conferred upon you by disclosure or delivery
of the Materials, either expressly, by implication, inducement, estoppel or
otherwise. Any license under such intellectual property rights must be express
and approved by Intel in writing.

:organization: INTEL MCG PSI
:summary: This script implements the basemark benchmark
:since: 18/02/2013
:author: pbluniex
"""
import os
import re
import time
from acs_test_scripts.Device.Model.AndroidDevice.Application.IAndroidPackage import IAndroidPackage
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException


class Basemark(IAndroidPackage):

    """
    Implementation of Basemark as Application
    """

    def __init__(self, device):
        """
        Initializes this instance.

        Nothing to be done in abstract class.
        """
        IAndroidPackage.__init__(self, device)
        self._path_ref = os.path.join("BENCHMARKS", "BASEMARK")
        self.is_lower_better = False

        self.__disable_egl = False
        self._result = None
        self.__basemark_test = None
        self.__score = None
        self.__tests = ["TAIJI", "HOVERJET"]
        self._results = {"score": []}
        self.__app_log = "/sdcard/rightware/basemarkes2v1/fm_mobile_app_log.txt"

    def __wait_app_init(self):
        """
        Wait until application end to init
        """
        init_pattern = "c_init_fonts"
        retry = 2

        while retry > 0:
            tail = self.adb_shell("tail -n1 %s" % self.__app_log, 3)
            if tail == init_pattern:
                return True

            retry -= 1
            time.sleep(5 * (retry != 0))

        return False

    def _fetch_result(self):
        """
        Return the score of basemark run
        """
        if self.__score is None:
            exception_str = "No score available %s" % self.__tests[self.__basemark_test]
            raise DeviceException(DeviceException.OPERATION_FAILED, exception_str)

        self._results["score"].append(float(self.__score.group("score")))
        self.__score = None

    def wait(self, timeout):
        """
        Wait until the benchmark ends
        """
        self._logger.debug("Basemark - wait test case begin.")
        test = self.__tests[self.__basemark_test]
        pattern = "end of game test %s fps:[ ]*(?P<score>\d+.\d+)" % test.lower()
        cmd = "tail -n1 %s" % self.__app_log
        end_time = time.time() + timeout
        while self.__score is None and time.time() < end_time:
            time.sleep(3)
            line_score = self.adb_shell(cmd, 3)
            self._logger.debug("Basemark - wait: Line score: %s" % line_score)
            self.__score = re.search(pattern, line_score.lower())
        if self.__score is None:
            raise DeviceException(DeviceException.OPERATION_FAILED, "Timeout while %s" % test)
        self._logger.debug("Basemark - wait test case end")

    def post_install(self):
        """
        Post installation of the application
        """

        if self._arguments not in self.__tests:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "There is no test named %s" % self._arguments)

        self.__basemark_test = self.__tests.index(self._arguments)

        rightware = os.path.basename(self._additionnals)

        cmd = "cd /sdcard && tar xzf %s && rm %s && " \
              "ln -s /sdcard/rightware /data/rightware" % (rightware, rightware)
        self.adb_shell(cmd, 30)

        cmd = "chmod -R 777 /sdcard/rightware /sdcard/rightware/basemarkes2v1"
        self.adb_shell(cmd, 3)

        width, height = self._phonesystem.get_screen_dimensions()
        script_lua = "/sdcard/rightware/basemarkes2v1/script.lua"
        sed_cmd = "sed -i 's/^\([[:space:]]*%s[[:space:]]*=[[:space:]]*\)[0-9]\+;$/\\1%d;/' " + script_lua

        cmd = sed_cmd % ("width", width)
        cmd += " && "
        cmd += sed_cmd % ("height", height)

        self.adb_shell(cmd, 3)

    def uninstall(self):
        """
        Uninstall benchmark
        """
        IAndroidPackage.uninstall(self)

        self.adb_shell("rm -r /sdcard/rightware; rm /data/rightware", 10)

    def drive(self):
        """
        Drive the application
        """
        self._logger.debug("Basemark - drive test case")

        if self.__disable_egl:
            cmd = ["move_home", "enter"]
            self._keyevent.scenario(cmd, 1)
        cmd = list([])
        for _ in range(0, self.__basemark_test):
            cmd.append("dpad_down")

        cmd.append("dpad_center")
        self._keyevent.scenario(cmd, 1)

        cmd_nt = "echo \"New Test drive\" >> %s" % self.__app_log
        self.adb_shell(cmd_nt, 3)
        self.__disable_egl = False
        self._logger.debug("Basemark - test case driven")

    def __launch_basemark(self):
        """
        Launch basemark
        """
        self._logger.debug("Basemark - launch test case")
        egl_not_available = "requested EGL_DEPTH_SIZE not available"
        self._get_device_logger().add_trigger_message(egl_not_available)

        IAndroidPackage.start(self)

        if self._get_device_logger().is_message_received(egl_not_available, 5):
            self.__disable_egl = True

        self._get_device_logger().remove_trigger_message(egl_not_available)

        return self.__wait_app_init()

    def start(self):
        """
        Start the application
        """
        self._logger.debug("Basemark - start test case")
        retry = 3

        while retry > 0:
            if not self.__launch_basemark():
                self._logger.error("Failed to launch Basemark application")
                self.stop()
                retry -= 1
            else:
                self._logger.debug("Basemark - test case launched")
                return

        raise DeviceException(DeviceException.OPERATION_FAILED,
                              "Cannot start application")

    def stop(self):
        """
        Stop the application

        In Basemark, the application should not be stopped
        until the end of all the executions, so that it does not get
        unloaded from memory, something which would affect the
        performance score.
        """
        pass
