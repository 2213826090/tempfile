"""
@copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
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

@organization: INTEL MCG PSI
@summary: This script implements the AndeBench benchmark for performance measurement
@since: 16/04/2013
@author: jbourgex
"""
import re
import os
import time
from acs_test_scripts.Device.Model.AndroidDevice.Application.IAndroidPackage import IAndroidPackage
from ErrorHandling.DeviceException import DeviceException


class Andebench(IAndroidPackage):
    """
    Implementation of Andebench as Application
    """

    def __init__(self, device):
        """
        Initializes this instance.

        @type device: Device
        @param device: The DUT
        """

        IAndroidPackage.__init__(self, device)
        self._path_ref = os.path.join("BENCHMARKS", "ANDEBENCH")
        self.is_lower_better = False
        self.__pattern = "AndEMark\s*%s\s*\d*.\d*\s*:\s*"
        self._results = {"score": []}
        self.__available_tests = ["JAVA", "NATIVE"]

    def _fetch_result(self):
        """
        Return the score of benchmark run
        """

        msg = self.__pattern % self._arguments
        res = self._get_device_logger().get_message_triggered_status("regex:" + msg)
        if not res:
            raise DeviceException(DeviceException.OPERATION_FAILED,
                                  "Can't get score for %s" % self._arguments)
        matchobj = re.search("%s(?P<result>\d+\.*\d*)" % msg, res[0])
        result = float(matchobj.group("result"))
        if result is not None:
            self._results["score"].append(result)
        else:
            raise DeviceException(DeviceException.OPERATION_FAILED,
                                  "Can't fetch result for %s" % self._arguments)
        self._get_device_logger().remove_trigger_message(msg)

    def wait(self, timeout):
        """
        Wait until the end of the run
        @type timeout: integer
        @param timeout: Time in second beyond the application should end
        """
        if not self._get_device_logger().\
           is_message_received("regex:" + self.__pattern % self._arguments, timeout):
            raise DeviceException(DeviceException.TIMEOUT_REACHED,
                                  "Timeout while executing Andebech")

    def drive(self):
        """
        Drive application
        """
        self._logger.debug("Andebench - \'drive\' begin")
        self._get_device_logger().add_trigger_message("regex:" + self.__pattern % self._arguments)
        cmd_list = ["TAB", "ENTER"]
        self._keyevent.scenario(cmd_list, 1)
        self._logger.debug("Andebench - \'drive\' end")

    def start(self):
        """
        Start application
        """
        self._logger.debug("Andebench - \'start\' begin")
        IAndroidPackage.start(self)
        time.sleep(5)
        if "java" in self._arguments.lower():
            cmd_list = ["BUTTON_MODE", "DPAD_LEFT", "ENTER", "TAB", "TAB", "TAB", "TAB", "ENTER", "BACK"]
        else:
            cmd_list = ["BUTTON_MODE", "DPAD_LEFT", "ENTER", "TAB", "TAB", "TAB", "ENTER", "BACK"]
        self._keyevent.scenario(cmd_list, 1)
        self._logger.debug("Andebench - \'start\' end")

    def post_install(self):
        """
        Post-installation actions
        """
        self.disable_jni()
        self.clear_cache()
