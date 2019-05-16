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
:summary: This script implements the Benchmark_Pi benchmark for performance measurement
:since: 16/04/2013
:author: plongepe
"""
import os
import re
import time
from acs_test_scripts.Device.Model.AndroidDevice.Application.IAndroidPackage import IAndroidPackage
from ErrorHandling.DeviceException import DeviceException


class BenchmarkPi(IAndroidPackage):

    """
    Implementation of Benchmark_Pi as Application
    """

    def __init__(self, device):
        """
        Initializes this instance.

        :type device: Device
        :param device: The DUT
        """
        IAndroidPackage.__init__(self, device)
        self._path_ref = os.path.join("BENCHMARKS", "BENCHMARKPI")
        self.is_lower_better = True
        self.__run_no = 0
        self._results = {"score": []}

    def _fetch_result(self):
        """
        Get score for Benchmark_Pi
        """
        self._result_file = "/data/data/gr.androiddev.BenchmarkPi/shared_prefs/gr.androiddev.BenchmarkPi_preferences.xml"

        result = self._fetch_file()
        match = re.search('.*bestTime.*value=.(?P<score>[0-9\.]+).[ ]*', result)
        if match:
            self._results["score"].append(float(match.group("score")))
        else:
            raise DeviceException(DeviceException.OPERATION_FAILED, "Can not fetch BenchmarkPi result")

    def wait(self, timeout):
        """
        Wait until the end of the run

        :type timeout: integer
        :param timeout: Time in second beyond the application should end
        """
        self._run_no += 1
        self.adb_shell("sleep 2", 3)

    def start(self):
        """
        Start application
        """
        self.adb_shell("rm -rf /data/data/gr.androiddev.BenchmarkPi/*", 3)
        IAndroidPackage.start(self)

        time.sleep(2)
        cmd_list = ["MOVE_HOME","ENTER"]
        self._keyevent.scenario(cmd_list)

    def post_install(self):
        """
        Post-installation actions
        """
        self._logger.debug("BenchmarkPi post-install execution.")
        self.disable_jni()
        self.clear_cache()

        result_folder = "benchmarkpi_result"
        report_dir = self._get_report_tree()
        report_dir.create_subfolder(result_folder)
        self.__benchmarkpi_report_path = report_dir.get_subfolder_path(result_folder)
