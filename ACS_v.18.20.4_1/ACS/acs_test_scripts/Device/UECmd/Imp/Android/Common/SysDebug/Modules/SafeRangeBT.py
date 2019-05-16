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
          This catch the occurrence of Bluetooth safe range for lte coex events
:since: 03/11/2014
:author: emarchan
"""

from acs_test_scripts.Device.UECmd.Imp.Android.Common.SysDebug.Modules.SafeRangeBase import SafeRangeBase
from acs_test_scripts.Device.UECmd.Imp.Android.Common.SysDebug.SysDebug import SysDebug
from lxml import etree
from Utilities.LocalConnectivityUtilities import get_bt_channel_from_freq


class SafeRangeBT(SafeRangeBase):

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
        self._domains = 'BT'
        SafeRangeBase.__init__(self, device, config)
        assert self._coex_mgr_version in [1, 2], "Wrong coex manager version"

        if self._coex_mgr_version == 1:
            if self._config == self.CONFIG_CHANNELS:
                """
                Log format:
                D/BluetoothManagerService( 4875): COEX_SAFECHANNELS : BR/EDR Channel Range : [0, 78]
                """
                self._retrieve_aplog_regexps.append("BR/EDR Channel Range :")
                self._trigger_group = [["bt_safe_range_min", ".*BR/EDR Channel Range\s*:\s*\[", "\d+", ",\s*"],
                               ["bt_safe_range_max", "", "\d+", "\].*$"]]

            elif self._config == self.CONFIG_FREQUENCIES:
                """
                Log format:
                D/BluetoothManagerService( 4875): COEX_SAFECHANNELS : Building BR/EDR Channel Range for [2402, 2480]
                """
                self._retrieve_aplog_regexps.append("Building BR/EDR")
                self._trigger_group = [["bt_safe_range_min", ".*Building BR/EDR Channel Range for\s*\[", "\d+", ",\s*"],
                               ["bt_safe_range_max", "", "\d+", "\].*$"]]
        elif self._coex_mgr_version == 2:
            """
            Log format:
             I/CWS_CELLCOEX_HANDLER(  641): BtCoexHandler: BT Safe frequency range = [2400, 2484]
            """
            self._trigger_group = [["bt_safe_range_min", ".*BT Safe frequency range = \[", "\d+", ",\s*"],
                           ["bt_safe_range_max", "", "\d+", "\].*$"]]

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
        if self._coex_mgr_version == 2 and self._config == "channels":
            SysDebug.stats(self, start, stop)
            domain = self._domains
            xmltree = etree.Element("SafeRange%s" % domain)
            match_log = ""
            domain_low = self._domains.lower()
            for cur_log in reversed(self._sys_debug_infos):
                if "%s_safe_range_min" % domain_low in cur_log.keys():
                    xmlrange = etree.Element("SafeRange")
                    if self._config == "channels":
                        xmlrange.attrib["min"] = str(get_bt_channel_from_freq(int(cur_log["%s_safe_range_min" % domain_low])))
                        xmlrange.attrib["max"] = str(get_bt_channel_from_freq(int(cur_log["%s_safe_range_max" % domain_low])))
                    xmltree.append(xmlrange)
                    match_log = cur_log
                    break

            self._logger.debug("SafeRange%s stats : %s" % (domain, str(match_log)))
            return xmltree
        else:
            return SafeRangeBase.stats(self, start, stop)

