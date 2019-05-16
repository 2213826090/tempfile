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
@summary: This script implements the RL Bench benchmark for performance measurement
@since: 03/19/2014
@author: plongepe
"""
import os
import re
import time
from acs_test_scripts.Device.Model.AndroidDevice.Application.IAndroidPackage import IAndroidPackage
from ErrorHandling.DeviceException import DeviceException


class RL_Bench(IAndroidPackage):

    """
    Implementation of RL Bench as Application
    """

    def __init__(self, device):
        """
        Initializes this instance.

        @type device: Device
        @param device: The DUT
        """
        IAndroidPackage.__init__(self, device)
        self._path_ref = os.path.join("BENCHMARKS", "RLBENCHMARK")
        self.is_lower_better = True
        self.__pattern = ".*=\"%s\" value=\"(?P<result>[0-9\.]+)\".*"
        self._results = {"test1": [], "test2": [], "test3": [], "test4": [], "test5": [],
                         "test6": [], "test7": [], "test8": [], "test9": [], "test10": [],
                         "test11": [], "test12": [], "test13": [], "overall": []}

    def _fetch_result(self):
        """
        Get score for CF Bench
        """
        self._logger.info("++ fetch results")
        self._result_file = "/data/data/com.redlicense.benchmark.sqlite/shared_prefs/rlbenchmark.xml"
        result = self._fetch_file()

        for key in self._results.keys():
            msg = self.__pattern % key
            match = re.search(msg, result)
            if match:
                self._logger.info("++ score: " + key + " = " + match.group("result"))
                self._results[key].append(int(match.group("result")))
            else:
                raise DeviceException(DeviceException.OPERATION_FAILED, "Can not fetch RL Bench result for: " + key)

    def wait(self, timeout):
        """
        Wait until the end of the run

        @type timeout: integer
        @param timeout: Time in second beyond the application should end
        """
        self._logger.info("++ wait")
        time.sleep(timeout)
        self._logger.info("++ end of wait")

    def go_back_to_menu(self):
        #self.adb_shell("rm -rf /data/data/com.redlicense.benchmark.sqlite/shared_prefs/*", 3)
        self._logger.info("go back to menu !")


    def drive(self):
        """
        Drive the application
        """
        self._logger.info("++ drive")
        cmd_list = ["ENTER"]
        self._keyevent.scenario(cmd_list, 2)

    def start(self):
        """
        Start application
        """
        self._logger.info("++ start")
        IAndroidPackage.start(self)
        time.sleep(5)
        cmd_list = ["MOVE_HOME", "TAB"]
        self._keyevent.scenario(cmd_list, 2)


    def post_install(self):
        """
        Post-installation actions
        """
        self.disable_jni();
        self.clear_cache();

        result_folder = "rlbench_result"
        report_dir = self._get_report_tree()
        report_dir.create_subfolder(result_folder)
        self.__cfbench_report_path = report_dir.get_subfolder_path(result_folder)
