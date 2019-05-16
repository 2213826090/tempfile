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
:summary: This script implements coremark benchmark
:since: 16/04/2013
:author: pbluniex
"""
import re
from acs_test_scripts.Device.Model.AndroidDevice.Application.IBinary import IBinary
import os
from ErrorHandling.DeviceException import DeviceException


class Coremark(IBinary):

    """
    Coremark benchmark implementation
    """

    def __init__(self, device):
        """
        Initializes this instance.

        :type device: Device
        :param device: The DUT
        """
        IBinary.__init__(self, device)
        self.is_lower_better = False
        self._path_ref = os.path.join("BENCHMARK", "COREMARK")
        self._results = {"score": []}
        self.__siblings = 1

    def _fetch_result(self):
        """
        Get score for sunspider
        """
        result = self._fetch_file()

        match = re.finditer('Iterations/Sec[ ]*:[ ]*(?P<score>[0-9\.]+)', result)
        if match:
            scores = [float(item.group("score")) for item in match]
            self._results["score"].append(min(scores) * self.__siblings)
        else:
            raise DeviceException(DeviceException.OPERATION_FAILED,
                                  "Can not fetch Coremark result")

    def install(self, appuri, additionnals=None, arguments=None, destination=None):
        """
        Install the application on the device

        :type appuri: String
        :param appuri: The full path to the application file

        :type additionnals: String
        :param additionnals: The full path of additionnals elements to run the
                             application

        :type arguments: String
        :param arguments: The arguments of the application. May be everything
                          the application need to run.

        :type destination: String
        :param destination: The directory where the application will be installed
        """

        siblings, cores = self._phonesystem.get_cpu_info(["siblings", "cpu cores"])
        self.__siblings = int(siblings)

        IBinary.install(self, appuri, additionnals, arguments, destination)

    def post_install(self):
        """
        Post installation configurations
        """
        self._command = "./%s & " % self._benchmark_name
        for _ in range(1, self.__siblings):
            self._command += "./%s & " % self._benchmark_name
        self._logger.info(self._command)
        IBinary.post_install(self)
