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

:organization: OTC
:summary: This script implements the Coremark benchmark for performance measurement
:since: 23/06/2014
:author: cimihaix
"""
import os
import re
import time
import requests
import subprocess
from acs_test_scripts.Device.Model.AndroidDevice.Application.IAndroidPackage import IAndroidPackage
from ErrorHandling.DeviceException import DeviceException

class CoreMarkApplication(IAndroidPackage):
    """
    Implementation of Andebench as Application
    """

    def __init__(self, device):
        """
        Initializes this instance.

        :type device: Device
        :param device: The DUT
        """

        IAndroidPackage.__init__(self, device)
        self._results = {"score": []}
        self.__device = device
        self._result = None


    def start(self):
        """
        Start application
        """
        artifact_param = self._global_config.benchConfig.get_parameters("ARTIFACT_MANAGER")
        addr = artifact_param.get_param_value("cache_folder") + self._arguments.split(";")[0]

        self.__device.push(addr, "/data/local/tmp/", 60)

        IAndroidPackage.start(self)

        command = "adb shell uiautomator runtest Performance.jar -c other.ACSCoremark"

        _, out = self.__device.run_cmd(command, 3000)

        out = out.split("\n")
        for line in out:
            if line.find("Score") == 0:
                match = re.search( r'[0-9]+(.[0-9]+)?', line)
                self._result = match.group(0)


    def _fetch_result(self):
        """
        Return the score of benchmark run
        """
        if self._result is not None:
            self._results["score"].append(float(self._result))
        else:
            raise DeviceException(DeviceException.OPERATION_FAILED,
                                  "Can't fetch result for Coremark")

    def post_install(self):
        """
        Post-installation actions
        """
        self.disable_jni()
        self.clear_cache()
