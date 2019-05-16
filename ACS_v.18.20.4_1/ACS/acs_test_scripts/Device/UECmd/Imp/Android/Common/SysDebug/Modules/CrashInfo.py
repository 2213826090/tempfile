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
:summary: This file implements the an UECmd SysDebug class for Android device
          This catch the occurrence of Panic events
:since: 18/03/2013
:author: pbluniex
"""

from lxml import etree
from acs_test_scripts.Device.UECmd.Imp.Android.Common.SysDebug.SysDebug import SysDebug


class CrashInfo(SysDebug):

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

        self.__event_number = 0
        self._use_aplog = False
        self._trigger_group = [["id", "^", "\d+", "\s*\|"],
                               ["eventid", "\s*", "[0-9a-f]*", "\s*\|"],
                               ["eventname", "\s*", "[A-Z]*", "\s*\|"],
                               ["type", "\s*", "[A-Z_]*", "\s*\|"],
                               ["data0", "\s*", "[^\|]*", "\s*\|"],
                               ["data1", "\s*", "[^\|]*", "\s*\|"],
                               ["data2", "\s*", "[^\|]*", "\s*\|"],
                               ["evdate", "\s*", "[0-9-/:]*", "\s*\|"],
                               ["crashdir", "\s*", ".*", "$"]]
        self._date_format = "%Y-%m-%d/%H:%M:%S"

    def _retrieve_sysdebug_message(self):
        """
        Get the crashlogs that occurs during the measure
        """
        sysinfo = []
        evocc = self.__get_nb_events()

        # crash event occurs
        nbocc = evocc - self.__event_number
        if not nbocc:
            return sysinfo

        cmd = "crashinfo getevent |head -n %d|tail -n%d" % (nbocc + 1, nbocc)
        rawsysinfo = self._exec("adb shell " + cmd, force_execution=True)
        sysinfo = rawsysinfo.splitlines()

        return sysinfo

    def __get_nb_events(self):
        """
        Get number of events

        :rtype: integer
        :return: Total number of events in crashinfo
        """
        cmd = "crashinfo status|grep 'Number of events'|cut -d':' -f2"
        nbev = self._exec("adb shell %s" % cmd, force_execution=True)
        if nbev and nbev.strip().isdigit():
            return int(nbev)
        else:
            return 0

    def init(self):
        """
        Initialisation of module: Get the last id of crashinfo
        """
        SysDebug.init(self)

        self.__event_number = self.__get_nb_events()
        return True

    def synchronize(self):
        """
        Synchronization of sysdebug module before unplug USB connector

        :rtype: Boolean
        :return: True if module is synchronized, False otherwise
        """
        self.__event_number = self.__get_nb_events()
        return True

    def reset(self, delay_s=0):
        """
        Reset the sysdebug informations
        """
        self.__event_number = self.__get_nb_events()

    def stats(self, start, stop):
        """
        Return wakelocks statistics which have been triggered between
        start and stop.

        :type start: Integer
        :param start: The timestamp of the beginning of the measure

        :type stop: Integer
        :param stop: The timestamp of the end of the measure

        :rtype: etree.Element
        :return: The Xml tree vue of statistic results of the module
        """
        SysDebug.stats(self, start, stop)

        xmltree = etree.Element("CrashInfo")
        for item in self._sys_debug_infos:
            panic = etree.Element("crash")
            panic.attrib["eventid"] = item["eventid"]
            panic.attrib["date"] = item["evdate"]
            panic.attrib["eventname"] = item["eventname"]
            panic.attrib["type"] = item["type"]
            panic.attrib["data0"] = item["data0"]
            panic.attrib["data1"] = item["data1"]
            panic.attrib["data2"] = item["data2"]
            xmltree.append(panic)

        self._logger.debug("crashinfo stats : %s" % str(self._sys_debug_infos))
        return xmltree
