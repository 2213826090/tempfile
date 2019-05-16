"""
:copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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
:summary: This file interact with application log generated
:since: 15/09/2014
:author: vgomberx
"""
from acs_test_scripts.Device.UECmd.Imp.Android.Common.BaseV2 import BaseV2
from acs_test_scripts.Device.UECmd.Interface.SysDebug.IAplog import IAplog
from ErrorHandling.DeviceException import DeviceException
from UtilitiesFWK.Utilities import Global
import re
import time


class Aplog(BaseV2, IAplog):

    """
    :summary: Aplog related UEcommands for Android platforms
    """
    # these value can be use directly on usecases
    PUPDR_MISC_TAG = {}
    PUPDR_SHUTDOWN_REASON = {}
    THERMAL_TAG = {}
    EM_TAG = {}

    def __init__(self, device):
        """
        Constructor.
        """
        BaseV2.__init__(self, device)
        IAplog.__init__(self, device)
        self._logger = device.get_logger()
        # Get system module to get hardware specific config
        self._system_module = self._device.get_device_module("SystemModule")
        self._system_module.init()
        self._aplog_folder = self._system_module.system_properties.aplog_folder
        self.PUPDR_SHUTDOWN_REASON = self._system_module.system_properties.PUPDR_SHUTDOWN_REASON
        self.PUPDR_MISC_TAG = self._system_module.system_properties.PUPDR_MISC_TAG
        self.EM_TAG = self._system_module.system_properties.EM_TAG
        self.THERMAL_TAG = self._system_module.system_properties.THERMAL_TAG

    def parse_shutdown_reason(self, txt):
        """
        parse the shutdown reason

        :warning: due to a problem with several log that look the same,
                    we need to parse the whole list instead of one element

        :type txt: str
        :param txt: text to parse

        :rtype: str
        :return: the shutdown reason found
        """
        result = "UNKNOWN"
        txt = txt.lower()
        shutdown_txt = self.PUPDR_MISC_TAG["SHUTDOWN_REASON"].get("MAIN").lower()

        for key in self.PUPDR_SHUTDOWN_REASON.keys():
            extra_txt = self.PUPDR_SHUTDOWN_REASON[key].lower()

            whole_tag = (shutdown_txt + extra_txt)
            if whole_tag in txt:
                result = key
                break

        return result

    def parse_battery_info(self, txt_list):
        """
        parse the battery info available in aplog and return
        a list of extracted battery info in the same order of the input

        :type txt_list: str or list
        :param txt_list: text or list to parse

        :rtype: tuple
        :return: list of tuple (capacity, voltage)
        """
        result = []
        # in case of str , we cast it in list
        if type(txt_list) is str:
            txt_list = [txt_list]

        batt_info_txt = self.EM_TAG["BATTERY_INFO"].get("MAIN").lower()
        for aplog_line in txt_list:
            txt = aplog_line.lower()
            # example of what we are parsing : battery_level: [73,4000,280,37]
            if batt_info_txt in txt:
                m = re.match("(.*)\[(.*),(.*),(.*),(.*)\]", txt)
                # if fail to parse try again with restricted format
                if m is None:
                    m = re.match("(.*)\[(.*),(.*),(.*)\]", txt)
                # we take only the capacity and voltage
                if m is not None:
                    matching_element = m.groups()
                    if len(matching_element) >= 3:
                        capacity = matching_element[1]
                        voltage = matching_element[2]
                        if str(capacity).strip().isdigit() and str(voltage).strip().isdigit():
                            result.append((int(capacity), int(voltage)))

        return result

    def parse_thermal_cooling_event(self, intent):
        """
        Parse the thermal cooling event and return several information about it

        :type intent: str
        :param intent: thermal cooling intent

        :rtype: tuple
        :return: a tuple (zone, level, temperature) or None if nothing is found
        """
        cooling_intent = self.THERMAL_TAG["COOLING_INTENT"].get("MAIN").lower()
        result = None
        # example of what is parsed :ThermalCooling: Received THERMAL INTENT:(ProfileName, ZoneName, State, EventType, Temp):(Default, FrontSkin, 2, HIGH, 37989)
        txt = intent.lower()
        if cooling_intent.lower() in txt:
            m = re.match("(.*):\((.*),(.*),(.*),(.*),(.*)\)", txt)
            if m is not None:
                matching_element = m.groups()
                clean_element = []
                # the first part of this group is useless
                i = 0
                for element in matching_element:
                    if i != 0:
                        clean_element.append(element.strip())
                    i += 1

                # check if the zone is on element
                if len(clean_element) >= 5:
                    # matching element should look like that Default, FrontSkin, 2, HIGH, 37989
                    zone = clean_element[1]
                    level = clean_element[2]
                    temperature = clean_element[4]
                    result = (zone, level, temperature)
                    self._logger.info("thermal_intent_parsing: zone %s - level %s - temperature %s" % (zone, level, temperature))
                else:
                    self._logger.error("thermal_intent_parsing: there is no enough element to parse the info, expect 5 got %s" % str(clean_element))

        return result

    def get_log_date(self, txt):
        """
        Try to read at which date the log was generated, and structured
        date object.
        aplog log does not contains a year, that why it easier
        to manipulate the date in this way and then adjust the year by yourself.
        time_struct does not handle millisecond

        :rtype: tuple
        :return: (time.struct_time, left millisecond)
        """
        millisec = None
        time_structure = None
        # ex : "10-29 07:47:39.300  1167  1167 I ThermalCooling...."
        from_year_to_millisec = txt.split(".", 1)
        if len(from_year_to_millisec) > 0:
            from_year_to_sec = from_year_to_millisec[0].strip()
            time_structure = time.strptime(from_year_to_sec, "%m-%d %H:%M:%S")
            tmp_millisec = from_year_to_millisec[1].split(" ", 1)
            if len(tmp_millisec) > 0:
                millisec = tmp_millisec[0]

        return time_structure, millisec

    def inject_tag(self, tag):
        """
        inject a tag that may be used to delimit a zone where
        you want to search for a value

        :type tag: str
        :param tag: tag to inject on aplog

        :rtype: boolean
        :return: True if the tag was injected, false otherwise
        """
        result = False
        self._logger.info("Trying to inject tag %s in aplog" % tag)
        formated_tag = str(tag).replace("\\", "\\\\")

        cmd = "adb shell log -p i -t ACS_TAG_INJECTOR '%s'" % formated_tag
        output = self._device.run_cmd(cmd, self._uecmd_default_timeout, force_execution=True)

        if output[0] == Global.SUCCESS:
            result = True

        return result

    def find_txt_between_tag(self, start_tag, stop_tag, txt, nb_matching_result=0, raise_error=True):
        """
        search for a text that is between a start and stop tag
        and return the a list of element found between them that contains txt.
        the result list element 0 is the closest to start tag.

        :type start_tag: str or list
        :param start_tag: first element where we start to search from

        :type stop_tag: str or list
        :param stop_tag: last element after which we want to stop searching
                         stop tag will be after start_tag

        :type txt: str or list
        :param txt: text that we are looking for

        :type nb_matching_result: int
        :param nb_matching_result: number of matching result you want to iter on
                                by default will return all result between the tag

        :rtype: list
        :return: list of str that contains all line matching the result
        """
        # start to browse each aplog until start and stop tag are seen
        start_info = self.__get_sorted_aplog(start_tag)
        stop_info = self.__get_sorted_aplog(stop_tag)
        stop_tag_txt = str(stop_tag)
        start_tag_txt = str(start_tag)
        result_info = []
        error_msg = ""
        if len(start_info) == 0:
            error_msg = "The start tag [%s] was not seen on your aplogs.\n" % (start_tag_txt)
        else:
            # take the most recent start tag info
            start_info = start_info[0]

        stop_info_found = False
        if len(stop_info) == 0:
            error_msg += "The stop tag [%s] was not seen on your aplogs.\n" % (stop_tag_txt)
        elif len(start_info) > 0:
            # search for the stop tag that is after start tag
            result = None
            for aplog_nb, line, misc_info in stop_info:
                # first check that we seen the txt after start tag
                if (start_info[0] > aplog_nb) or (start_info[0] == aplog_nb and start_info[1] <= line):
                    result = (aplog_nb, line, misc_info)
                    stop_info_found = True
                    break

            if stop_info_found:
                stop_info = result
            else:
                error_msg += "The stop tag [%s] found is older than the start tag [%s]  in your  aplogs\n" % (stop_tag_txt, start_tag_txt)

        if error_msg != "":
            error_msg = error_msg.replace("\n", "", 1)
            self._logger.error(error_msg)
            if raise_error:
                raise DeviceException(DeviceException.OPERATION_FAILED, error_msg)

        else:
            # get all aplog where txt appear, revert them to start by the older one
            info_where_txt_appear = reversed(self.__get_sorted_aplog(txt))

            result_found = False
            start_found = False
            result_info = []
            for aplog_nb, line, misc_info in info_where_txt_appear:

                # first check that we seen the txt after start tag
                if not start_found:
                    if (start_info[0] > aplog_nb) or (start_info[0] == aplog_nb and start_info[1] <= line):
                        start_found = True
                # then check that we are not outside stop tag
                if start_found:
                    if (stop_info[0] < aplog_nb) or (stop_info[0] == aplog_nb and stop_info[1] >= line):
                        result_found = True
                        result_info.append(misc_info)
                        self._logger.debug("text [%s] found in [aplog=%s][line=%s][info='%s']" % (str(txt), aplog_nb, line, misc_info))
                        if nb_matching_result > 0:
                            if len(result_info) >= nb_matching_result:
                                break

            if not result_found:
                self._logger.debug("text [%s] not found in the wanted range" % str(txt))

        return result_info

    def __get_sorted_aplog(self, txt):
        """
        get a sorted list of line from aplogs that contains tag
        the most recent one will be at index 0

        :type txt: str or dict
        :param txt: text that we are looking for

        :rtype: list
        :return: list of tuple as (aplog number, line number, txt) that contains the tag
        """
        # allow user to use the define tag above or directly text
        extra_pattern = None
        if type(txt) is dict:
            if len(txt) > 1:
                # extra patter must be a list
                extra_pattern = txt.get("NOT_SORTED")
            tag = txt.get("MAIN")
        else:
            tag = txt

        formated_tag = str(tag).replace("\\", "\\\\")
        # use simple quote to take the literal slash
        aplog = self._exec('adb shell "grep -nH \'%s\' %s/aplog?(.)*([0-9]) | sort -k 1.13nr |sed \'/^$/d\'"' % (formated_tag, self._aplog_folder), force_execution=True)
        tmp_result = []
        result = []
        aplog = filter(None, aplog.strip().split("\n"))
        for line in aplog:
            info = line.split(":", 2)
            if len(info) > 2:
                # compute aplog number
                aplog_no = info[0].split(".")[-1].strip()
                if aplog_no.isdigit():
                    aplog_no = int(aplog_no)
                else:
                    aplog_no = 0
                # check the extra pattern
                ok_to_add = True
                if type(extra_pattern) is list:
                    for ele in extra_pattern:
                        if ele.lower() not in info[2].lower():
                            ok_to_add = False
                            break
                if ok_to_add:
                    # in order : (aplog number , line , the remaining part of the line)
                    tmp_result.append((aplog_no, int(info[1].strip()), info[2]))

        # sort the list of result by putting the more recent one as first element
        most_recent_index = 0
        nearest_recent_index = 0
        while len(tmp_result) > 0:
            index = 0
            most_recent_info = tmp_result[most_recent_index]
            # search if we found a more recent element than most_recent_info
            for sub_aplog_nb, sub_line, _ in tmp_result:
                if (most_recent_info[0] > sub_aplog_nb) or (most_recent_info[0] == sub_aplog_nb and most_recent_info[1] < sub_line):
                    most_recent_info = tmp_result[index]
                    nearest_recent_index = most_recent_index
                    most_recent_index = index
                index += 1

            result.append(tmp_result.pop(most_recent_index))
            most_recent_index = max(nearest_recent_index - 1, 0)
        return result
