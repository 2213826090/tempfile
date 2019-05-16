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
:summary: This file implements an UECmd SysDebug class for Android device
          This catch the occurrence of WiFi safe range for lte coex events
:since: 03/11/2014
:author: emarchan
"""

from acs_test_scripts.Device.UECmd.Imp.Android.Common.SysDebug.Modules.SafeRangeBase import SafeRangeBase
from lxml import etree
from acs_test_scripts.Device.UECmd.Imp.Android.Common.SysDebug.SysDebug import SysDebug
from acs_test_scripts.Utilities.NetworkingUtilities import AcsWifiFrequencies
from acs_test_scripts.Utilities.CommunicationUtilities import LteSafeRanges


class SafeRangeWifi(SafeRangeBase):

    """
    :summary: System UEcommands operations for Android platforms
    """

    def __init__(self, device, config=None):
        """
        Constructor.

        :type config: dictionary
        :param config: Defines what this instance will return: Use 'frequencies' to have the safe frequencies range or
        'channels' for the frequencies converted in channels.
        """
        self._domains = 'WIFI'
        SafeRangeBase.__init__(self, device, config)

        if self._coex_mgr_version == 1:
            self._retrieve_aplog_regexps.append("frequencyToWifiChannelBitmap(")

            """
            Log format:
            CWS_SERVICE_MGR-CoexMgr: frequencyToWifiChannelBitmap(2401, 2437, 10) length = 14
            """
            self._trigger_group = [["wifi_safe_range_min", ".*frequencyToWifiChannelBitmap\(", "\d+", ",\s*"],
                           ["wifi_safe_range_max", "", "\d+", ",.*$"]]

        elif self._coex_mgr_version == 2:
            """
            Log format:
            I/CWS_CELLCOEX_MGR(  684): WifiCoexMgr: Wi-Fi safe channels: [0, 0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 2]
            D/CWS_CELLCOEX_MGR(  631): CoexProperty: mws.wifi.safeChannels: setValue([0, 0, 0, 0, 0, 10, 10, 10, 10, 10, 10, 10, 10]) while current val=[10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10]
            """
            self._aplog_level = "D"
            self._trigger_group = [["wifi_prios", ".*setValue\(\[", "[^\]]*", "\].*$"]]
            # self._trigger_group = [["wifi_prios", ".*channels:\s*\[", "[^\]]*", "\].*$"]]

    def stats(self, start, stop):
        """
        Return safe range statistics which have been triggered between
        start and stop.

        :type start: Integer
        :param start: The timestamp of the beginning of the measure

        :type stop: Integer
        :param stop: The timestamp of the end of the measure

        :rtype: etree.Element
        :return: The Xml tree view of statistic results of the module
        """
        if self._coex_mgr_version == 1:
            if self._config == "frequencies":
                return SafeRangeBase.stats(self, start, stop)
            else:
                SysDebug.stats(self, start, stop)
                domain = self._domains
                xmltree = etree.Element("SafeRange%s" % domain)
                match_log = ""
                for cur_log in reversed(self._sys_debug_infos):
                    if "%s_safe_range_min" % domain.lower() in cur_log.keys():
                        xmlrange = etree.Element("SafeRange")

                        # Look for the 1st matching channel frequency
                        # Central freq = min freq + 20 (bandwitdh) / 2
                        lsr = LteSafeRanges()
                        min_freq = int(cur_log["%s_safe_range_min" % domain.lower()])
                        xmlrange.attrib["min"] = lsr.get_safe_wifi_channel_from_freq(min_freq, "min")

                        max_freq = int(cur_log["%s_safe_range_max" % domain.lower()])
                        xmlrange.attrib["max"] = lsr.get_safe_wifi_channel_from_freq(max_freq, "max")

                        xmltree.append(xmlrange)
                        match_log = cur_log
                        break

                self._logger.debug("SafeRange%s stats : %s" % (domain, str(match_log)))
                return xmltree
        elif self._coex_mgr_version == 2:
            SysDebug.stats(self, start, stop)
            domain = self._domains
            xmltree = etree.Element("SafeRange%s" % domain)
            match_log = ""
            for cur_log in reversed(self._sys_debug_infos):
                if "wifi_prios" in cur_log.keys():
                    xmlrange = etree.Element("SafeRange")
                    """
                    Here we need to parse the safe channel bitmap to find the safe range
                    """
                    cur_prios = cur_log["wifi_prios"]
                    cur_prios = cur_prios.replace(' ', '')
                    cur_prios = cur_prios.split(',')
                    min_safe_value = 1
                    max_safe_value = cur_prios
                    for i in range(1, len(cur_prios) + 1):
                        if cur_prios[i - 1] != "0":
                            min_safe_value = str(i)
                            break
                    for i in reversed(range(1, len(cur_prios) + 1)):
                        if cur_prios[i - 1] != "0":
                            max_safe_value = str(i)
                            break

                    if self._config == "channels":
                        xmlrange.attrib["min"] = min_safe_value
                        xmlrange.attrib["max"] = max_safe_value
                    else:
                        xmlrange.attrib["min"] = AcsWifiFrequencies.get_frequency(int(min_safe_value))
                        xmlrange.attrib["max"] = AcsWifiFrequencies.get_frequency(int(max_safe_value))
                    xmltree.append(xmlrange)
                    match_log = cur_log
                    break

            self._logger.debug("SafeRange%s stats : %s" % (domain, str(match_log)))
            return xmltree

