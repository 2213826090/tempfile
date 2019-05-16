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
:summary: Use Case base for push and pull
:since: 30/06/2014
:author: cimihaix
"""

import os
import time
import re
import math
import numpy
import subprocess
import requests
import commands
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from acs_test_scripts.Utilities.PnPUtilities import PnPResults
from Core.Report.SecondaryTestReport import SecondaryTestReport
from lxml import etree


class PushAndPull(UseCaseBase):

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
        self._iterations = self._tc_parameters.get_param_value("ITERATIONS")
        self._size       = self._tc_parameters.get_param_value("SIZE")
        self._dest       = self._tc_parameters.get_param_value("DEST")

    def set_up(self):
        """
        Set up the test configuration

        :rtype: tuple
        :return: tuple of Verdict and comment
        """
        return Global.SUCCESS, "No errors"

    def run_test(self):
        """
        Run the test

        :rtype: tuple
        :return: tuple of Verdict and comment
        """
        os.system("dd if=/dev/zero of=test1.file bs=1048576 count=" + self._size)
        res, out = commands.getstatusoutput("md5sum test1.file")
        out = out.split(" ")[0]

        for i in range(int(self._iterations)):
            result, output = self._device.push("test1.file", self._dest, 300)
            res, output = self._device.run_cmd("adb pull " + self._dest + "test1.file" + " /tmp/tmp", 10)
            res, output2 = commands.getstatusoutput("md5sum /tmp/tmp")
            res, outp = self._device.run_cmd("adb shell rm " + self._dest  + "test1.file", 10)
            res, outp = self._device.run_cmd("rm /tmp/tmp", 10)
            output = output2.split(" ")[0]
            print output
            if not output.split(" ")[0] == out:
                return Global.FAILURE, "md5 sums are not the same"

        return Global.SUCCESS, "No errors"


    def tear_down(self):
        """
        Tear down

        :rtype: tuple
        :return: tuple of Verdict and comment
        """
        os.system("rm test1.file")
        return Global.SUCCESS, "No errors"
