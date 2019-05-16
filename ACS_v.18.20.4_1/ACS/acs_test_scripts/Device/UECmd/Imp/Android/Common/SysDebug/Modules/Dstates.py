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
:since: 21/01/2015
:author: sdubrayx
"""

from lxml import etree
from acs_test_scripts.Device.UECmd.Imp.Android.Common.SysDebug.SysDebug import SysDebug


class Dstates(SysDebug):

    """
    :summary: System UEcommands operations for Android platforms
    """

    def __init__(self, device, _config=None):
        """
        Constructor.

        :type config: dictionary
        :param config: Configuration of the module. Unused for this module
        """
        SysDebug.__init__(self, device)
        self._sleep_mode_apis = self._device.get_uecmd("SleepMode")
        self.__d_states_files = self._sleep_mode_apis.get_d_states_files()
        self._use_aplog = False
        self._print_debug = False

    def _retrieve_sysdebug_message(self):
        """
        Get the dstates informations from the device
        """
        sysinfo = []

        for dsfile in self.__d_states_files:
            cmd = "cat %s |grep -B2 -e D0 -e D3 |grep -v \"^\s*$\"" % dsfile
            _ , output = self._device.run_cmd("adb shell " + cmd, self._uecmd_default_timeout,
                                              force_execution=True, silent_mode=True)
            sysinfo += output.splitlines()

        return sysinfo

    def init(self):
        """
        Initialization
        """
        SysDebug.init(self)

        if not self.__d_states_files:
            self._logger.debug("Debug file does not exist, Dstates module should not be loaded")
            return False
        return True

    def _retrieve_sysinfo(self):
        """
        Redefined because we do not want to use regexps
        """
        sysinfo = self._retrieve_sysdebug_message()
        for item in sysinfo:
            self._sys_debug_infos.append(item)
        return sysinfo

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

        xmltree = etree.Element("Dstates")
        for item in self._sys_debug_infos:
            ds = etree.Element("device")
            ds.attrib["line"] = item
            xmltree.append(ds)

        return xmltree
