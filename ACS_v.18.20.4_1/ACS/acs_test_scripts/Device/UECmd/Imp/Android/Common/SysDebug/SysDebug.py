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
import posixpath
from acs_test_scripts.Utilities.PnPUtilities import grouped_regex_from_list
from acs_test_scripts.Device.UECmd.Imp.Android.Common.Base import Base


class SysDebug(Base):

    """
    Sysdebug top class
    """
    # Member that are common to any instance
    __host_offset = None
    __boot_timestamp = None

    def __init__(self, device, _config=None):
        """
        Constructor

        :type device: Device
        :param device: The DUT

        :type config: str
        :param config: The configuration of the module

        """
        Base.__init__(self, device)
        self._kernel_version = None
        self._start_timestamp = None
        self._stop_timestamp = None
        self._trigger_group = None
        self._sys_debug_infos = []
        self.__cur_timestamp = None
        self._print_debug = True

        self._device = device
        self._logger = device.get_logger()
        self._use_aplog = True
        self._aplog_level = "[A-Z]"
        self._regex_begin = "^(?P<date>[0-9]{2}-[0-9]{2} [0-9]{2}:[0-9]{2}:[0-9]{2})"\
                            "\.(?P<milli>[0-9]{3})\s+\d+\s+\d+\s+"
        self._retrieve_aplog_command = "grep -h "
        self._retrieve_aplog_regexps = []
        self._statistics = None
        self._date_format = "%Y-%m-%d %H:%M:%S"

        self._system_module = self._device.get_device_module("SystemModule")
        self._system_module.init()
        self._aplog_folder = self._system_module.system_properties.aplog_folder
        self.all_logs = []

    def __get_trigger_dic(self, re_obj):
        """
        Return a dictionary of re_obj with the help of _trigger_group

        :type re_obj: MatchObject
        :param re_obj : The result of regular expression search method
        """
        ret = {}
        if "date" in re_obj.groupdict().keys():
            ret["date"] = re_obj.group("date")
            ret["milli"] = re_obj.group("milli")

        for group in self._trigger_group:
            ret[group[0]] = re_obj.group(group[0])

        return ret

    def _get_trigger_regex(self):
        """
        Return the regular expression of the logcat line to trig
        """
        regpattern = ""
        if self._use_aplog:
            regpattern = self._regex_begin + self._aplog_level + "\s+"

        return regpattern + grouped_regex_from_list(self._trigger_group)

    def _retrieve_sysdebug_message(self):
        """
        Retrieve triggered message in aplog
        """
        if self._retrieve_aplog_regexps:
            return self._retrieve_sysdebug_message_regexp()
        else:
            return self._retrieve_sysdebug_message_command()

    def _retrieve_sysdebug_message_regexp(self):
        """
        Retrieve triggered message in aplog with python regexp
        """
        if not self.all_logs:
            # Ensure the path is well formatted for the regex
            self._aplog_folder = posixpath.abspath(self._aplog_folder)
            cmd = "find {0} -type f -maxdepth 1 -name \"aplog*\" |".format(self._aplog_folder)
            cmd += "sort -k 1.13nr|xargs cat|"
            cmd += "tac |sed -n \"1,/ACS_TESTCASE:[[:space:]]*RUNTEST:/p\" |tac"

            _, output = self._device.run_cmd("adb shell %s" % cmd, self._uecmd_default_timeout,
                                             force_execution=True, silent_mode=True)
            self.all_logs = output.splitlines()
        msgs = [x for x in self.all_logs if [y for y in self._retrieve_aplog_regexps if re.search(y, x)]]
        return msgs

    def _retrieve_sysdebug_message_command(self):
        """
        Retrieve triggered message in aplog with _retrieve_aplog_command
        """
        # Ensure the path is well formatted for the regex
        self._aplog_folder = posixpath.abspath(self._aplog_folder)
        cmd = "find {0} -type f -maxdepth 1 -name \"aplog*\" |".format(self._aplog_folder)
        cmd += "sort -k 1.13nr|xargs cat|"
        cmd += "tac |sed -n \"1,/ACS_TESTCASE:[[:space:]]*RUNTEST:/p\" |tac|%s" % self._retrieve_aplog_command

        _, occurences = self._device.run_cmd("adb shell %s" % cmd, self._uecmd_default_timeout,
                                             force_execution=True, silent_mode=True)
        return occurences.splitlines()

    def _is_pattern_in_range(self, regroup):
        """
        Compute if regex group is in range of [start; stop]

        :type regroup: MatchObject
        :param regroup : The result of regular expression search method

        :rtype: boolean
        :return: True if log has been written between start and stop,
                 False otherwise
        """

        # If MatchObject has no date group, then return True because
        # date is not mandatory
        if "date" not in regroup.groupdict().keys():
            return True

        if self._use_aplog:
            sys_date = time.strftime("%Y-") + regroup.group("date")
        else:
            sys_date = regroup.group("date")

        sys_time = time.strptime(sys_date, self._date_format)

        timestamp = time.mktime(sys_time)

        if self._use_aplog:
            timestamp += float(regroup.group("milli")) / 1000

        return self._start_timestamp <= timestamp <= self._stop_timestamp

    def _retrieve_sysinfo(self):
        """
        Retrieve the informations between start and stop.
        Informations are retrieved from aplogs

        The informations have to be stored in _trigger_group member
        with following format:

            [[groupname, prefix, pattern, suffix],
             [...]]

        The list will be formatted in regular expression pattern. For each
        item, the formatted pattern will be :

            "prefix(?P<groupname>pattern)suffix"

        Each pattern will be appended to the final regular expression:

        The final pattern is prefixed by the beginning of the line:

            "mm-dd HH:MM:SS.xxx I "

        where :
            * mm  is the month
            * dd  is the day
            * HH  is the hour
            * MM  is the minute
            * SS  is the second
            * xxx is the millisecond
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

            is_pattern_in_range = self._is_pattern_in_range(sysgroup)
            if is_pattern_in_range:
                if self._print_debug:
                    self._logger.debug("_retrieve_sysinfo : Adding [%s]" % item)
                self._sys_debug_infos.append(self.__get_trigger_dic(sysgroup))
            else:
                if self._print_debug:
                    self._logger.debug("_retrieve_sysinfo : Out of range [%s]" % item)

        if self._print_debug:
            self._logger.debug("_retrieve_sysinfo outputs %s" % str(self._sys_debug_infos))

    def synchronize(self):
        """
        Synchronization of sysdebug module before unplug USB connector

        :rtype: Boolean
        :return: True if module is synchronized, False otherwise
        """
        return True

    def init(self):
        """
        Initialization of the wakelocks.
        At this step, the board must be connected

        :rtype: boolean
        :return: True if module initialisation succeed
        """
        return True

    def reset(self, delay_s=0):
        """
        Reset the sysdebug informations on the device
        """
        pass

    def fetch(self):
        """
        Fetch sysdebug information with a connected device
        """
        pass

    def stats(self, start, stop):
        """
        Return lxml.etree node for statistic vue for results of sysdebug
        modules

        :type start: Integer
        :param start: The timestamp of the beginning of the measure

        :type stop: Integer
        :param stop: The timestamp of the end of the measure
        """
        self._start_timestamp = start
        self._stop_timestamp = stop

        self._retrieve_sysinfo()

    def get_aplog_messages(self):
        """
        Returns all logs from aplog (if parsed)

        :rtype: list
        :return: list of logs
        """
        return self.all_logs

    def set_aplog_messages(self, aplogs):
        """
        Provide aplogs message to be used by the class (avoids parsing in each module)

        :type start: list
        :param start: list of logs
        """
        if aplogs and isinstance(aplogs, list):
            self.all_logs = aplogs
