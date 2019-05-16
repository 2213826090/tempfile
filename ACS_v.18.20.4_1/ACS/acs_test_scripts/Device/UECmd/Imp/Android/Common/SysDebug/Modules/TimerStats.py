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
:summary: This file implements the SysDebug UECmd for Android device
:since: 16/02/2015
:author: sdubrayx
"""

import re
import os
import platform
import time
from lxml import etree
from acs_test_scripts.Device.UECmd.Imp.Android.Common.SysDebug.SysDebug import SysDebug


class TimerStats(SysDebug):

    """
    :summary: System UEcommands operations for Android platforms
    """

    def __init__(self, device, config=None):
        """
        Constructor.

        :type config: dictionary
        :param config: Configuration of the module. Unused for this module
        """
        SysDebug.__init__(self, device)
        self._use_aplog = False
        self._print_debug = False
        self.__timer_stats_file = "/proc/timer_stats"
        self._min_per_sec = 20 # only show timers seen at least 10 times per second
        self._trigger_group = [["number", "\s*", "\d+", ""],
                               ["deferred", "", "[D]?", "\s*,\s*"],
                               ["pid", "\s*", "[0-9]+", "\s*"],
                               ["pname", "\s*", ".*", "\s*"],
                               ["function", "\s+", "[A-Za-z0-9_]+", "\s+"],
                               ["timer", "\s*\(", "[A-Za-z0-9_]+", "\).*$"]]

    def init(self):
        """
        Initialization
        """
        SysDebug.init(self)
        return True

    def reset(self, delay_s=0):
        """
        Enable keywords
        """
        if delay_s:
            cmd = "adb shell nohup sh -c \"sleep %d; echo 1 > %s\" &" % (delay_s, self.__timer_stats_file)
            self._exec(cmd, force_execution=True, wait_for_response=False, timeout=1)

        cmd = "echo 1 > %s" % self.__timer_stats_file
        self._device.run_cmd("adb shell %s" % cmd, self._uecmd_default_timeout,
                             force_execution=True, silent_mode=True)

        return True

    def fetch(self):
        """
        Fetch sysdebug information on a connected device
        """
        cmd = "echo 0 > %s" % self.__timer_stats_file
        self._device.run_cmd("adb shell %s" % cmd, self._uecmd_default_timeout,
                             force_execution=True, silent_mode=True)

    def _retrieve_sysdebug_message(self):
        """
        Retrieve triggered message in aplog
        """
        cmd = "adb shell cat %s |sort -nr" % self.__timer_stats_file
        _ , output = self._device.run_cmd(cmd, self._uecmd_default_timeout,
                                          force_execution=True, silent_mode=True)
        return output.splitlines()

    def stats(self, start, stop):
        """
        Return stats

        :type start: Integer
        :param start: not used

        :type stop: Integer
        :param stop: not used

        :rtype: etree.Element
        :return: The Xml tree vue of statistic results of the module
        """
        SysDebug.stats(self, start, stop)
        xmltree = etree.Element("TimerStats")
        total_time = stop - start # in seconds
        for item in self._sys_debug_infos:
            if "D" in item["deferred"]:
                continue
            if int(item["number"]) > total_time * self._min_per_sec:
                timer = etree.Element("timer")
                timer.attrib["number"] = item["number"]
                timer.attrib["pid"] = item["pid"]
                timer.attrib["process"] = item["pname"]
                timer.attrib["function"] = item["function"]
                timer.attrib["timer"] = item["timer"]
                xmltree.append(timer)
        return xmltree
