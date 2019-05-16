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

@organization: INTEL PCCG PSTV-BA
@summary: This script implements the Geek Bench performance benchmark
@since: 04/07/2014
@author: skoolanx
"""
import os
import time
import sys
from acs_test_scripts.Device.Model.AndroidDevice.Application.IAndroidPackage import IAndroidPackage
from ErrorHandling.DeviceException import DeviceException

"""Load PCCG Library"""
library_path = os.path.join(os.path.abspath(os.path.curdir), "_ExecutionConfig/PCCG/library")
sys.path.append(library_path)
from PCCGUtilities import *


class GeekBench(IAndroidPackage):
    """
    Implementation of Benchmark_Pi as Application
    """

    def __init__(self, device):
        """
        Initializes this instance.
        @type device: Device
        @param device: The DUT
        """
        IAndroidPackage.__init__(self, device)
        self._path_ref = os.path.join("BENCHMARKS", "GEEKBENCH")
        self.__device = device
        self.__count = 0

    def __get_screenshot(self):
        """
        Capture the screenshot at end of the test
        """
        report_dir = self.__device.get_report_tree()
        self.__geek_bench_report_path = os.path.join(report_dir.get_report_path(), "geekbench_result")

        filename = os.path.join(self.__geek_bench_report_path, "GeekBench %s.png" % time.ctime().replace(':', '-'))
        self.__device.screenshot(filename=filename)
        if self.__count == 3:
            msg = "PARTIALLY AUTOMATED, Please check screenshots placed at %s" % self.__geek_bench_report_path
            raise DeviceException(msg)

    def _fetch_result(self):
        """
        Return the score of geekbench run
        """
        self.__count += 1
        self.__get_screenshot()

    def wait(self, timeout):
        """
        Wait until the end of the run

        @type timeout: integer
        @param timeout: Time in second beyond the application should end
        """
        self._logger.debug("++ clear logcat buffer")
        PCCGUtilities.clear_logcat()

        self._logger.info("++ wait while tests are running")
        for _ in range(timeout):
            time.sleep(1)
            dlc_inst = PCCGUtilities.find_in_logcat(pat='Displayed com.primatelabs.geekbench3/.DocumentActivity',
                                                    l_opt='ActivityManager:I *:S')
            if isinstance(dlc_inst, str):
                self._logger.debug("++ Scores generated")
                time.sleep(5)
                break
        self._logger.info("++ end of wait")

    def go_back_to_menu(self):
        self._logger.info("go back to menu !")
        cmd_list = ["BACK", "BACK", "BACK", "HOME"]
        self._keyevent.scenario(cmd_list, 0.2)

    def drive(self):
        """
        Drive the application
        """
        self._logger.info("++ drive")
        if PCCGUtilities.tap(attr="text", value="Run Benchmarks"):
            self._logger.debug("Tapped on 'Run Benchmarks'")
        else:
            raise DeviceException("Unable to find the 'Run Benchmarks' button")

    def start(self):
        """
        Start application
        """
        self._logger.info("++ wait 10 secs before running benchmark")
        time.sleep(10)
        self._logger.info("++ start")
        IAndroidPackage.start(self)
        time.sleep(3)

    def post_install(self):
        """
        Post-installation actions
        """
        self.clear_cache()
