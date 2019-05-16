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


class DynamicDebug(SysDebug):

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
        self.__dyndebug_keywords = []
        self.__dyndebug_file = "/d/dynamic_debug/control"
        self.__data_file = "/data/dyndbg_tmp"

        if config:
            for key in config:
                self.__dyndebug_keywords.append(key)
        else:
            self._logger.debug("Warning ! No keywords provided, module will be disabled")

    def init(self):
        """
        Initialization
        """
        SysDebug.init(self)

        # if not defined
        if not self.__dyndebug_keywords:
            return False
        return True

    def reset(self, delay_s=0):
        """
        Enable keywords
        """
        cmd = "cat %s | grep -e '%s'" % (self.__dyndebug_file, "' -e '".join(self.__dyndebug_keywords))
        cmd += "| sed 's/\\([^:]\\+\\):\\([0-9]\\+\\).*/file \\1 line \\2 +p/' > %s" % self.__data_file
        self._device.run_cmd("adb shell " + cmd, self._uecmd_default_timeout,
                             force_execution=True, silent_mode=True)

        cmd = "while read line; do echo $line > %s; done < %s" % (self.__dyndebug_file, self.__data_file)
        self._device.run_cmd("adb shell " + cmd, self._uecmd_default_timeout,
                                 force_execution=True, silent_mode=True)
        return True

    def fetch(self):
        """
        Fetch sysdebug information on a connected device
        Here, there is nothing to fetch but we disable all debug logs
        """
        cmd = "cat %s | grep -e '%s'" % (self.__dyndebug_file, "' -e '".join(self.__dyndebug_keywords))
        cmd += "| sed 's/\\([^:]\\+\\):\\([0-9]\\+\\).*/file \\1 line \\2 -p/' > %s" % self.__data_file
        self._device.run_cmd("adb shell " + cmd, self._uecmd_default_timeout,
                             force_execution=True, silent_mode=True)

        cmd = "while read line; do echo $line > %s; done < %s" % (self.__dyndebug_file, self.__data_file)
        self._device.run_cmd("adb shell " + cmd, self._uecmd_default_timeout,
                              force_execution=True, silent_mode=True)

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
        xmltree = etree.Element("DynamicDebug")
        return xmltree
