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
:summary: Use Case base for applications based power measurement
:since: 20/02/2013
:author: pbluniex
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


class PowerOffMenu(UseCaseBase):

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

    def __fetch_artifactory_file(self, appuri):
        """
        Retrieve a file from artifactory
        """
        artifact_url_base = "https://mcg-depot.intel.com/artifactory/acs_test_artifacts/"
        artifact_url = artifact_url_base + appuri

        local_path = os.path.join(self._execution_config_path, "EMBD")
        local_filename = os.path.join(local_path, os.path.basename(appuri))
        local_filename = local_filename.split(";")[0]

        if not os.path.exists(local_path):
            os.mkdir(local_path)

        if os.path.exists(local_filename):
            return local_filename

        self._device.get_logger().info("Download %s => %s" % (artifact_url, local_filename))

        start = time.time()
        r = requests.get(artifact_url, proxies={'http': ''})
        r.raise_for_status()

        length = float(r.headers['content-length'])
        progressize = 0

        with open(local_filename, "wb") as f:
            for chunk in r.iter_content(chunk_size=1024000):
                if chunk:
                    progressize += len(chunk)
                    percent = int(progressize / length * 100)
                    self._logger.debug("progress of downloading %s file: %d%%" %
                                       (appuri, percent))
                    f.write(chunk)

        dlduration = time.time() - start
        self._device.get_logger().info("Download %s in %f seconds (birate: %d MiO/s)" %
                          (appuri, dlduration, length / (dlduration * 8000000)))

        return local_filename




    def set_up(self):
        """
        Set up the test configuration

        :rtype: tuple
        :return: tuple of Verdict and comment
        """
        #Download and push uiautomator jar on the device
        arguments = self._tc_parameters.get_param_value("ARGUMENTS")
        addr = self.__fetch_artifactory_file(arguments.split("artifact://")[1])

        result, output = self._device.push(addr.split(";")[0], "/data/local/tmp/", 60)

        return Global.SUCCESS, "No errors"

    def run_test(self):
        """
        Run the test

        :rtype: tuple
        :return: tuple of Verdict and comment
        """
        self._io_card.press_power_button(4.0)

        res, out = self._device.run_cmd("adb shell uiautomator runtest Performance.jar -c other.PowerOffWindow| grep \"RESULT: OK\"", 120)

        self._device.run_cmd("adb shell input keyevent KEYCODE_ESCAPE", 2)

        if not out == "RESULT: OK":
            return Global.FAILURE, "Window not detected"
        return Global.SUCCESS, "No errors"


    def tear_down(self):
        """
        Tear down

        :rtype: tuple
        :return: tuple of Verdict and comment
        """
        return Global.SUCCESS, "No errors"
