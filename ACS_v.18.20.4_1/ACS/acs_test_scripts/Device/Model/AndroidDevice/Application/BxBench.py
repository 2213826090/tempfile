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
:summary: This script implements bxbench benchmark
:since: 16/04/2013
:author: pbluniex
"""
import re
from acs_test_scripts.Device.Model.AndroidDevice.Application.IBrowsing import IBrowsing
from ErrorHandling.DeviceException import DeviceException


class BxBench(IBrowsing):

    """
    BxBench benchmark implementation (from Eembc)
    """

    INITIALIZING = -1
    BENCH_SITE = 0
    COMPLETE = 1

    def __init__(self, device):
        """
        Initializes this instance.

        :type device: Device
        :param device: The DUT
        """
        IBrowsing.__init__(self, device)
        self.is_lower_better = False

        self._results = {"score": []}
        self.__state = self.INITIALIZING

    def __process_message(self, msg):
        """
        Process a message from browser console output

        :type msg: str
        :param msg: Message to process
        """
        site = re.search("Bench sites/(?P<site>.*)", msg)
        if site:
            self._logger.info("Test site: %s" % site.group("site"))
            self.__state = self.BENCH_SITE
            return

        complete = re.search("Geomean Medians Score \(.*\): (?P<score>[\d\.]*)", msg)
        if complete:
            self.__state = self.COMPLETE
            self._results["score"].append(float(complete.group("score")))
            return

    def __pop_logcat(self, logcat):
        """
        Process data from logcat lines

        :type logcat: list
        :param logcat: list of logcat message triggered by device logger
        """
        for line in logcat:
            match = re.search(self._logcat_pattern, line)
            msg = match.group("msg")
            self.__process_message(msg)

    def wait(self, timeout):
        """
        Wait until the end of benchmark

        :type timeout: integer
        :param timeout: Timeout beyond no message is triggered
        """
        logger = self._get_device_logger()

        while self.__state != self.COMPLETE:
            logcat = logger.is_message_received("regex:" + self._logcat_pattern, timeout)
            if logcat:
                self.__pop_logcat(logcat)
            else:
                raise DeviceException(DeviceException.TIMEOUT_REACHED,
                                      "Timeout while browsing")
