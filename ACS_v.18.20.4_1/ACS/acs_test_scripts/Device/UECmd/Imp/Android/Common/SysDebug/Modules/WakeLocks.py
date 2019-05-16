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
:summary: This file implements the SysDebug UEcmd for Android device
:since: 27/02/2013
:author: pbluniex
"""

import time
import re
import math
import os
import posixpath
from lxml import etree
from acs_test_scripts.Device.UECmd.Imp.Android.Common.SysDebug.SysDebug import SysDebug


class WakeLocks(SysDebug):

    """
    :summary: System UEcommands operations for Android platforms
              Fetching SysDebug informations on wakelocks.
    """

    def __init__(self, device, _config=None):
        """
        Constructor.

        :type device: Device
        :param device: The DUT

        :type config: dictionary
        :param config: Configuration of the module. Unused for this module
        """
        SysDebug.__init__(self, device)

        self._retrieve_aplog_command = "grep -h WAKELOCK"

        # Trigger group :
        # Key is the name of the group
        # Value is a list of [group, prefix, pattern, suffix] where pattern
        # will be fetched in groups by re.search
        self._trigger_group = [["action", "WAKELOCK_", "(ACQUIRE)|(RELEASE)", ": "],
                               ["timestamp", "TIMESTAMP=", "[0-9]*", ", "],
                               ["tag", "TAG=", "[^,]*", ", "],
                               ["type", "TYPE=", "[A-Z_]*", " *, "],
                               ["count", "COUNT=", "[0-9]*", ", "],
                               ["pid", "PID=", "[0-9]*", ", "],
                               ["uuid", "UID=", "[0-9]*", ".*$"]]

        self.__wakelocks_file = None
        self.__active_since_col = None
        self.__wakelocks_stats = {}
        self.__ts_origin = None
        self.__wl_current = None

    def _retrieve_sysdebug_message(self):
        """
        Retrieve triggered message in aplog
        """
        # Ensure the path is well formatted for the regex
        self._aplog_folder = posixpath.abspath(self._aplog_folder)
        cmd = "find {0} -type f -maxdepth 1 -name \"/aplog*\"|".format(self._aplog_folder)
        cmd += "sort -k 1.13nr|xargs cat|tac|"
        cmd += "sed -n \"1,/beginning of main\|beginning of \/dev\/log\/kernel/p\"|"
        cmd += "tac| %s" % self._retrieve_aplog_command
        _ , occurences = self._device.run_cmd("adb shell %s" % cmd, self._uecmd_default_timeout,
                                              force_execution=True, silent_mode=True)
        return occurences.splitlines()

    def __toxml(self):
        """
        Compute statistics into lxml.etree

        :rtype: etree.Element
        :return: The Xml tree vue of statistic results of the module
        """
        xmltree = etree.Element("WakeLocks")
        for tag in self.__wakelocks_stats:
            xmlwl = etree.Element("WakeLock")
            xmlwl.attrib["name"] = tag
            for attrib, value in self.__wakelocks_stats[tag].items():
                xmlwl.attrib[attrib] = str(value)

            xmltree.append(xmlwl)

        return xmltree

    def __sort(self):
        """
        Sort by timestamp before computing
        """
        tmp_stats = {}
        ret_stats = []

        for wakelock in self._sys_debug_infos:
            tmp_stats[int(wakelock["timestamp"])] = wakelock

        for timestamp in sorted(tmp_stats.keys()):
            ret_stats.append(tmp_stats[timestamp])

        return ret_stats

    def __get_origin_timestamp(self):
        """
        Compute the origin timestamp of wakelocks
        """
        if not self._sys_debug_infos:
            return

        wakelock = self._sys_debug_infos[-1]

        wl_date = time.strftime("%Y-") + wakelock["date"]
        event_stime = time.strptime(wl_date, "%Y-%m-%d %H:%M:%S")
        event_ts = time.mktime(event_stime)

        timestamp = event_ts * 1000 + int(wakelock["milli"])
        self.__ts_origin = timestamp - float(wakelock["timestamp"]) / 10 ** 6

    def __trig_action(self, wakelock):
        """
        Treatment of an entry in sys debug infos
        """
        mstimestamp = self.__ts_origin + float(wakelock["timestamp"]) / 10 ** 6
        if wakelock["action"] == "ACQUIRE":
            if self._start_timestamp * (10 ** 3) < mstimestamp:
                self.__wl_current["acquired"] += 1
            self.__wl_current["nb"] = 1
            if "ms_timestamp_acquire" not in self.__wl_current:
                self.__wl_current["ms_timestamp_acquire"] = mstimestamp
        elif wakelock["action"] == "RELEASE":
            if self._start_timestamp * (10 ** 3) < mstimestamp:
                self.__wl_current["released"] += 1
            self.__wl_current["nb"] = -1
            self.__wl_current["ms_timestamp_release"] = mstimestamp

    def __init_tag(self, wakelock):
        """
        Initialisation of the __wakelocks_stats struct for current
        wakelock

        :type wakelock: dictionary
        :param wakelock: Current wakelock read from sys_debug_infos
        """
        tag = wakelock["tag"]

        if tag not in self.__wakelocks_stats.keys():
            self.__wakelocks_stats[tag] = {
                "acquired": 0,
                "released": 0,
                "nb": -1,
                "spent": 0}

        self.__wl_current = self.__wakelocks_stats[tag]

    def __compute_spent_time(self, start, stop):
        """
        Compute spent time on current wakelock

        :type start: float
        :param start: timestamp in seconds when measure started

        :type stop: float
        :param stop: timestamp in seconds when measure stoped
        """
        if self.__wl_current["nb"] == -1:
            self.__wl_current["nb"] = 0
            ms_timestamp_acquire = self.__wl_current.pop("ms_timestamp_acquire", start * (10 ** 3))
            ms_timestamp_release = self.__wl_current.pop("ms_timestamp_release", stop * (10 ** 3))

            if start * (10 ** 3) > ms_timestamp_acquire:
                ms_timestamp_acquire = start * (10 ** 3)
            if stop * (10 ** 3) < ms_timestamp_release:
                ms_timestamp_release = stop * (10 ** 3)
            if ms_timestamp_release > ms_timestamp_acquire:
                self.__wl_current["spent"] += ms_timestamp_release - ms_timestamp_acquire

    def __update_spent_time(self, start, stop):
        """
        Update spent time for unreleased wakelocks at the end of the measure

        :type start: float
        :param start: timestamp in seconds when measure started

        :type stop: float
        :param stop: timestamp in seconds when measure stoped
        """
        for wakelock in self.__wakelocks_stats.values():
            if wakelock["nb"]:
                ms_timestamp_acquire = wakelock.pop("ms_timestamp_acquire", start * (10 ** 3))
                ms_timestamp_release = wakelock.pop("ms_timestamp_release", stop * (10 ** 3))

                if start * (10 ** 3) > ms_timestamp_acquire:
                    ms_timestamp_acquire = start * (10 ** 3)

                if stop * (10 ** 3) < ms_timestamp_release:
                    ms_timestamp_release = stop * (10 ** 3)

                if ms_timestamp_release > ms_timestamp_acquire:
                    wakelock["spent"] += ms_timestamp_release - ms_timestamp_acquire

        # delete all never acquired
        dellist = []
        for wakelock in self.__wakelocks_stats.keys():
            wl = self.__wakelocks_stats[wakelock]
            if wl["spent"] <= 0:
                dellist.append(wakelock)
        for elem in dellist:
            self.__wakelocks_stats.pop(elem, None)

    def init(self):
        """
        Initialization
        """
        SysDebug.init(self)

        self.__wakelocks_file = "/sys/kernel/debug/wakeup_sources"
        self.__active_since_col = 6

        return True

    def synchronize(self):
        """
        If there is unexpected wakelock return False

        :rtype: Boolean
        :return: True if SysModule is synchronized, False otherwise
        """
        config = self._device.get_config("WakeLocksAcceptedBlocker")
        if not config:
            return True

        expected = [wl.strip() for wl in config.split(",")]
        self._logger.debug("WakeLocks::synchronize: valid wakelocks = %s" % str(expected))

        cmd = "cat %s | sed 's/[[:space:]]\+/ /g' |cut -d' ' -f1,%d"\
              "| grep -v -e ' 0' -e 'name'" % (self.__wakelocks_file, self.__active_since_col)
        _ , res = self._device.run_cmd("adb shell %s" % cmd, self._uecmd_default_timeout,
                                       force_execution=True, silent_mode=True)

        curwls = [wl.strip("\"") for wl in res.splitlines()]

        for wlock in curwls:
            if wlock.split() and wlock.split()[0] not in expected:
                self._logger.warning("WAKELOCK BLOCKER : %s" % wlock)
                # return False
            else:
                self._logger.warning("WAKELOCK EXPECTED : %s" % wlock)

        return True

    def _retrieve_sysinfo(self):
        """
        Retrieve the informations between start and stop.
        Informations are retrieved from aplogs
        Needs to be rewritten, because we do not filter for wakelocks
        """
        sysinfo = self._retrieve_sysdebug_message()

        regex = self._get_trigger_regex()

        for item in sysinfo:
            sysgroup = re.search(regex, item)
            if not sysgroup:
                self._logger.error("Line does not match pattern skip it : \n"
                                   "   %s\n"
                                   "   %s" % (str(item), regex))
                continue

            #self._logger.debug("_retrieve_sysinfo : Adding [%s]" % item)
            self._sys_debug_infos.append(self._SysDebug__get_trigger_dic(sysgroup))

    def stats(self, start, stop):
        """
        Return wakelocks statistics which have been triggered between
        start_trigger and stop_trigger.

        :type start: Integer
        :param start: The timestamp of the beginning of the measure

        :type stop: Integer
        :param stop: The timestamp of the end of the measure

        :rtype: etree.Element
        :return: The Xml tree vue of statistic results of the module
        """
        SysDebug.stats(self, start, stop)

        sys_debug_infos = self.__sort()

        self.__get_origin_timestamp()

        for wakelock in sys_debug_infos:
            self.__init_tag(wakelock)
            self.__trig_action(wakelock)
            self.__compute_spent_time(start, stop)

        self.__update_spent_time(start, stop)

        self._logger.debug("wakelocks stats : %s" % self.__wakelocks_stats)
        return self.__toxml()
