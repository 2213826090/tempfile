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
:since: 01/03/2013
:author: pbluniex
"""

import re
import math
import time
from lxml import etree
from acs_test_scripts.Device.UECmd.Imp.Android.Common.SysDebug.SysDebug import SysDebug


class Pmu(SysDebug):

    """
    :summary: System UEcommands operations for Android platforms
    """
    __lss_table_file = "/sys/kernel/debug/pmu_sss_states"
    __pmu_stats_log = "/sys/kernel/debug/pmu_stats_log"
    __pmu_interval_file = "/sys/module/intel_soc_pm_debug/parameters/pmu_stats_interval"

    def __init__(self, device, config):
        """
        Constructor

        :type config: dictionary
        :param config: Configuration of the module.
                       The configurable parameter is the interval of dumps.
                       Example of parameter: {"interval : 30}
        """
        SysDebug.__init__(self, device)

        self.__stats_interval = 180
        self.__dump_nb = 0
        self.__lss_table = {}
        self._retrieve_aplog_command = "sed -n '/----MID_PMU_STATS_LOG_BEGIN----/," \
                                       "/----MID_PMU_STATS_LOG_END----/p'"

        self._regex_begin += "I\s+KERNEL\s*:\s*\[[\s\.\d,]*\]\s+"
        self.__seconds_before_next_dump = 0
        self.__init_sync_ts = 0
        self.__sysinfo = None

        if config and "interval" in config and config["interval"].isdigit():
            self.__stats_interval = int(config["interval"])
            self._logger.debug("Update pmu_stats_interval to %d" % self.__stats_interval)
        else:
            self._logger.debug("No change : config : %s" % str(config))

    def __retrieve_lss_table(self):
        """
        Retrieve the lss table correspondance in __lss_table with the help of
        __lss_table_file
        """
        rawSstate = self._exec("adb shell cat %s" % self.__lss_table_file,
                               force_execution=True)

        lines = rawSstate.splitlines()

        header = {"lss": 0, "block": 0, "subsystem": 0}
        head = lines[0]

        for item in header:
            header[item] = head.find(item) + len(item)

        # Remove header lines
        lines.pop(0)
        lines.pop(0)
        for line in lines:
            lss = line[3:header["lss"]].strip()
            subsystem = line[header["block"]:header["subsystem"]].strip()

            self.__lss_table[lss] = subsystem

    def __aplog_dump_init_sync(self):
        """
        Get time to wait before starting the measure.
        If stats_interval is too high (300), the measure may not contains dump
        of pmu_stats_log
        The logical steps are :
            - Get the current interval
            - Get the date of last dump of pmu_stats_log
            - Get the current date in timestamp
            - Get the difference and compare to the interval to determine how
              time we should wait until the next dump
        """
        cmd = "cat " + self.__pmu_interval_file + " && date -u +%s"

        result = self._exec("adb shell " + cmd, force_execution=True)

        # Set pmu_stats_interval to new one
        cmd = "echo %d > %s" % (self.__stats_interval, self.__pmu_interval_file)
        self._exec("adb shell " + cmd, force_execution=True)

        (interval, timestamp) = result.splitlines()

        dbl_min = float(interval) / 60
        cmd = "find %s -name \"aplog*\"" % self._aplog_folder
        cmd += "|xargs grep -h MID_PMU_STATS_LOG_BEGIN|sort|tail -n1"
        result = self._exec("adb shell " + cmd, force_execution=True)

        match = re.search(self._regex_begin, result)
        ts_start = int(timestamp)
        if match:
            last_date = time.strftime("%Y-") + match.group("date")
            last_ts = time.strptime(last_date, "%Y-%m-%d %H:%M:%S")
            ts_start = time.mktime(last_ts)

        self.__seconds_before_next_dump = min(int(interval), ts_start + int(interval) - int(timestamp))
        self.__init_sync_ts = time.time()
        self._logger.debug("Next PMU_STATS_LOGS dump should occur in %d second(s)" %
                           self.__seconds_before_next_dump)

    def __parse_lss_blocked_count(self, sstate):
        """
        Parse the number of block for each LSS for the SState in argument

        :type sstate: str
        :param sstate: The current SState block in the parsing of current
                       pmu_stats_log dump

        Ignore following lines :
            Display blocked: X times
            Camera blocked: X times
            LSS      #blocked

        And parse lines that match :
            XX      Y

        Where XX is a LSS number and Y the number the lss has been blocked
        The block ends up with empty line
        """
        pattern = self._regex_begin + "(?P<lss>\d+)\s*(?P<blocked>\d+)$"
        lss_match = None
        line = self.__sysinfo.pop(0)

        # Block ends with blank line
        while not re.search(self._regex_begin + "$", line):
            lss_match = re.search(pattern, line)
            if not lss_match:
                line = self.__sysinfo.pop(0)
                continue

            lss = lss_match.group("lss")
            blocked = lss_match.group("blocked")
            if lss not in self._sys_debug_infos[sstate]:
                self._sys_debug_infos[sstate][lss] = ["0"] * (self.__dump_nb - 1)

            self._sys_debug_infos[sstate][lss].append(blocked)

            line = self.__sysinfo.pop(0)

    def __parse_aplog_dump(self):
        """
        Parse a dump of pmu_stats_log

        Ignore the residency and transition tables and parse lines that match:
            X: Block Count

        Where X is the sstate (s0i1, s0i3 or lpmp3)
        """

        # start to look for sstate block count
        pattern = self._regex_begin + "(?P<sstate>(s0i1|s0i3|lpmp3)): Block Count"
        sstate_match = None

        line = self.__sysinfo.pop(0)

        # Ignore header until the beggining of the block
        while not re.search(pattern, line):
            line = self.__sysinfo.pop(0)

        while not re.search(self._regex_begin + "$", line):
            sstate_match = re.search(pattern, line)
            if not sstate_match:
                self._logger.debug("Line does not match pattern : %s (%s)" % (line, pattern))
                line = self.__sysinfo.pop(0)
                continue

            cur_sstate = sstate_match.group("sstate")
            # Initialization of dictionary if item not exists.
            # Each LSS will be associated with a list of blocker increasement
            if cur_sstate not in self._sys_debug_infos:
                self._sys_debug_infos[cur_sstate] = {}

            self._logger.debug("Add %s sstate (%s)" %
                               (cur_sstate, sstate_match.group("date")))
            self.__parse_lss_blocked_count(cur_sstate)

            line = self.__sysinfo.pop(0)

    def __ignore_current_dump(self):
        """
        Got to the end of the current dump of pmu_stats_log
        """
        end_ptrn = self._regex_begin + "----MID_PMU_STATS_LOG_END----"
        while self.__sysinfo and not re.search(end_ptrn, self.__sysinfo.pop(0)):
            pass

    def __parse_pmu_stats_log_run(self):
        """
        Parse a log from aplog
        """
        pattern = self._regex_begin + "----MID_PMU_STATS_LOG_BEGIN----"
        line = self.__sysinfo.pop(0)

        begin = re.search(pattern, line)
        if not begin:
            self._logger.error("Log does not match requirements : %s\n"
                               "search pattern : %s" % (line, pattern))
            return

        if self._is_pattern_in_range(begin):
            self._logger.debug("_retrieve_sysinfo : Adding [%s]" % begin.group("date"))
            self.__dump_nb += 1
            self.__parse_aplog_dump()

        # Ignore the lines until the end of the cump
        self.__ignore_current_dump()

    def __compute_lss_stats(self, hist_values):
        """
        Compute increasement of lss during the measure

        :type hist_values: list
        :param hist_values: historic values of blockers found in pmu_stats_log dumps

        :rtype: list
        :return: Differential vue of hist_values. Each value is the real number of
                 blockers since last dump.
        """
        previous = 0
        ret = []
        for value in hist_values:
            diff = str(int(value) - int(previous))
            ret.append(diff)
            previous = value

        return ret

    def __toxml(self, dic_stats):
        """
        Return lxml.etree structure of statistics

        :type dic_stats: dictionary
        :param dic_stats: Statistic representation of informations.
                         Each entry is a dictionary as following:
                            "SState": {
                                "Subsys1": [val1, val2, val3],
                                "Subsys2": [val4, val5, val5],
                                ...
                            }

        :rtype: etree.Element
        :return: The lxml representation of the statistic vue.
        """
        xmltree = etree.Element("Pmu")
        for sstate in dic_stats:
            xmlsstate = etree.Element("SState")
            xmlsstate.attrib["name"] = sstate

            for subsystem, value in dic_stats[sstate].items():
                xmlsubsys = etree.Element("SubSystem")
                xmlsubsys.attrib["name"] = subsystem
                xmlsubsys.attrib["initial_value"] = value[0]
                xmlsubsys.attrib["history"] = ";".join(value[1:])
                xmlsstate.append(xmlsubsys)

            xmltree.append(xmlsstate)

        return xmltree

    def _retrieve_sysinfo(self):
        """
        Get the PMU stats structure from device
        """
        self._sys_debug_infos = {}
        self.__sysinfo = self._retrieve_sysdebug_message()

        self.__dump_nb = 0
        while self.__sysinfo:
            self.__parse_pmu_stats_log_run()

    def __has_blockers(self):
        """
        :rtype: boolean
        :return: True if there is at least one unexpected s0ix blocker
                 False otherwise
        """
        config = self._device.get_config("s0ixLssAcceptedBlocker")
        if not config:
            return False

        expected = [blocker.strip() for blocker in config.split(",")]
        self._logger.debug("Pmu::__has_blockers: valid blockers = %s" % str(expected))

        pattern = "^pci\s*(?P<vendor>[0-9a-fA-F]{4})\s*(?P<model>[0-9a-fA-F]{4})\s*"\
                  "(?P<pci>[0-9]{4}:[0-9]{2}:[0-9]{2})\.[0-9]\s*"\
                  "(?P<name>[^:]*):(\s*lss:(?P<lss>[0-9]*))?.*$"

        cmd = "grep -e \"^pci\" /sys/kernel/debug/mid_pmu_states |grep 'blocking s0ix'"
        res = self._exec("adb shell %s" % cmd, force_execution=True)

        blockers = res.splitlines()
        for blocker in blockers:
            match = re.search(pattern, blocker)
            if not match:
                self._logger.error("Line does not match pattern : %s / %s" %
                                   (blocker, pattern))
                continue

            # If there is no LSS associated with blocking s0ix, we try with pci
            # name
            if "lss" in match.groupdict().keys():
                if not match.group("lss") in expected:
                    self._logger.warning("PMU BLOCKER FOUND (LSS) : %s" % match.group("lss"))
                    # return True
            elif "pci" in match.groupdict().keys():
                if not match.group("pci") in expected:
                    self._logger.warning("PMU BLOCKER FOUND (PCI) %s" % match.group("pci"))
                    # return True

        return False

    def synchronize(self):
        """
        Synchronization of sysdebug module before unplug USB connector :

           aplog stats dump must occure one time before the new interval
           value is taken in consideration. This synchronization consists in
           waiting until the dump should occur

        :rtype: boolean
        :return: True if the module is synchronized, False otherwise
        """
        # If there is s0ix blocker, module is not synchronized
        if self.__has_blockers():
            return False

        # If need to wait next dump, module is not synchronized
        seconds_spent = time.time() - self.__init_sync_ts
        if seconds_spent < self.__seconds_before_next_dump:
            seconds = self.__seconds_before_next_dump - seconds_spent
            self._logger.info("Pmu::synchronize : Wait %d second until next "
                              "pmu_stats_log dump" % seconds)
            return False

        return True

    def init(self):
        """
        Initialization
        """
        SysDebug.init(self)

        __lss_table_file = "/sys/kernel/debug/pmu_sss_states"
        __pmu_stats_log = "/sys/kernel/debug/pmu_stats_log"
        __pmu_interval_file = "/sys/module/intel_soc_pm_debug/parameters/pmu_stats_interval"

        cmd = "ls %s %s %s 1> /dev/null 2>&1 && echo Ok || echo NOk" % \
             (__lss_table_file, __pmu_stats_log, __pmu_interval_file)
        testfiles = self._exec("adb shell %s" % cmd, 1, force_execution=True)
        if testfiles == "NOk":
            return False

        self.__aplog_dump_init_sync()
        self.__retrieve_lss_table()
        return True

    def stats(self, start, stop):
        """
        Return representation of lss changes

        :type start: Integer
        :param start: The timestamp of the beginning of the measure

        :type stop: Integer
        :param stop: The timestamp of the end of the measure

        :rtype: etree.Element
        :return: The Xml tree vue of statistic results of the module
        """
        SysDebug.stats(self, start, stop)

        self._logger.debug("sysDebugInfo = %s" % self._sys_debug_infos)

        pmustats = {}
        for sstate in self._sys_debug_infos:
            pmustats[sstate] = {}
            for lss in self._sys_debug_infos[sstate]:
                if lss not in self.__lss_table:
                    self._logger.error("Unable to retrieve LSS Subsystem (%s)" % lss)

                lss_subsys = self.__lss_table[lss]

                stats = self.__compute_lss_stats(self._sys_debug_infos[sstate][lss])
                pmustats[sstate][lss_subsys] = stats

        self._logger.debug("Stats : %s" % str(pmustats))
        return self.__toxml(pmustats)
