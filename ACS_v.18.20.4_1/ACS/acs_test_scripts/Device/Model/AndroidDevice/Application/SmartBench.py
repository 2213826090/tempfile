"""
@copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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
@summary: This script implements the Smartbench benchmark for performance measurement
@since: 25/02/2014
@author: plongepe
"""
import os
import re
import time
from acs_test_scripts.Device.Model.AndroidDevice.Application.IAndroidPackage import IAndroidPackage
from ErrorHandling.DeviceException import DeviceException


class SmartBench(IAndroidPackage):

    """
    Implementation of Smartbench as Application
    """

    def __init__(self, device):
        """
        Initializes this instance.

        @type device: Device
        @param device: The DUT
        """
        IAndroidPackage.__init__(self, device)
        self._path_ref = os.path.join("BENCHMARKS", "SMARTBENCH")
        self.is_lower_better = False
        self.__pattern = "value%s=(?P<result>\d+)"
        self.__end_log = "SBGLJellyfish:onStop()"
        self._results = {"Prod": [], "Game": []}

    def _fetch_result(self):
        """
        Get score for Smartbench
        """
        for key in self._results.keys():
            msg = self.__pattern % key
            res = self._get_device_logger().get_message_triggered_status("regex:" + msg)
            if not res:
                raise DeviceException(DeviceException.OPERATION_FAILED,
                                      "Can't get score for %s" % msg)
            matchobj = re.search(msg, res[0])
            result = matchobj.group("result")
            if result is not None and result.isdigit():
                self._results[key].append(int(matchobj.group("result")))
            else:
                raise DeviceException(DeviceException.OPERATION_FAILED, "Can't fetch result")

            self._get_device_logger().remove_trigger_message("regex:" + msg)


    def wait(self, timeout):
        """
        Wait until the end of the run

        @type timeout: integer
        @param timeout: Time in second beyond the application should end
        """
        if not self._get_device_logger().\
           is_message_received("regex:" + self.__end_log, timeout):
            raise DeviceException(DeviceException.TIMEOUT_REACHED,
                                  "Timeout 1 while executing SmartBench")

        self._get_device_logger().add_trigger_message("regex:" + self.__pattern % "Prod")
        self._get_device_logger().add_trigger_message("regex:" + self.__pattern % "Game")

        time.sleep(5)
        cmd_list = ["ENTER", "BACK"]
        self._keyevent.scenario(cmd_list, 2.0)

        if not self._get_device_logger().\
           is_message_received("regex:" + self.__pattern % "Game", timeout):
            raise DeviceException(DeviceException.TIMEOUT_REACHED,
                                  "Timeout 2 while executing SmartBench")

    def go_back_home(self):
        time.sleep(1)
        cmd_list = ["BACK"]
        self._keyevent.scenario(cmd_list, 2.0)

    def drive(self):
        """
        Drive application
        """
        IAndroidPackage.start(self)
        time.sleep(3)
        cmd_list = ["ENTER", "ENTER"]
        self._keyevent.scenario(cmd_list, 2.0)

    def post_install(self):
        """
        Post-installation actions
        """
        self.disable_jni()
        self.clear_cache()
