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
:summary: Use Case base for Chess application crash test
:since: 20/05/2014
:author: cimihaix
"""

import os
import time
import re
import math
import numpy
import subprocess
import requests
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from acs_test_scripts.Utilities.PnPUtilities import PnPResults
from Core.Report.SecondaryTestReport import SecondaryTestReport
from lxml import etree


class ChessCrashTest(UseCaseBase):

    """
    Class live to run benchmarks application
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor

        :type tc_name: BaseConf
        :param tc_name: Configuration of the usecase

        :type global_config: Dictionnary
        :param global_config: Global configuration of the campaign
        """
        # Call power measurement base Init function
        UseCaseBase.__init__(self, tc_name, global_config)
        self._artifact_manager = self._em.get_artifact_manager("ARTIFACT_MANAGER")
        self._system_api = self._device.get_uecmd("System")
        self._app_api = self._device.get_uecmd("AppMgmt")

    def set_up(self):
        """
        Set up the test configuration

        :rtype: tuple
        :return: tuple of Verdict and comment
        """
        #Download and push uiautomator jar on the device
        arguments = self._tc_parameters.get_param_value("SUPPORT")
        app_location = self._tc_parameters.get_param_value("APKS")
        local_artifact = self._artifact_manager.get_artifact(artifact_name=app_location, transfer_timeout=3600)
        self._app_api.install_device_app(local_artifact, 120)

        local_artifact = self._artifact_manager.get_artifact(artifact_name=arguments, transfer_timeout=3600)

        if os.path.isfile(local_artifact):
            result, output = self._device.push(local_artifact, "/data/local/tmp/", 60)
            if result != Global.SUCCESS:
                return result, output
        else:
            return Global.BLOCKED, "Cannot find {0}".format(local_artifact)

        return Global.SUCCESS, "No errors"

    def run_test(self):
        """
        Run the test

        :rtype: tuple
        :return: tuple of Verdict and comment
        """
        res, out = self._device.run_cmd("adb shell uiautomator runtest Performance.jar -c other.ChessApp| grep \"APP CRASHED\"", 600)

        if out == "APP CRASHED":
            return Global.FAILURE, "Application crashed"
        return Global.SUCCESS, "No errors"


    def tear_down(self):
        """
        Tear down

        :rtype: tuple
        :return: tuple of Verdict and comment
        """
        return Global.SUCCESS, "No errors"
