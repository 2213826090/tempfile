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
          This catch the occurrence of CWS safe range for lte coex events.
          THIS MODULE CAN'T BE USED ALONE, IT NEEDS TO BE INSTANCIED FROM SafeRangeXXX.
:since: 03/11/2014
:author: emarchan
"""

from lxml import etree
from acs_test_scripts.Device.UECmd.Imp.Android.Common.SysDebug.SysDebug import SysDebug
from ErrorHandling.DeviceException import DeviceException
from UtilitiesFWK.Utilities import Global


class SafeRangeBase(SysDebug):
    CONFIG_CHANNELS = "channels"
    CONFIG_FREQUENCIES = "frequencies"

    def __init__(self, device, config=None):
        """
        Constructor.

        :type config: dict
        :param config: Defines what this instance will return: Use 'frequencies' to have the safe frequencies range or
        'channels' for the frequencies converted in channels.
        """
        SysDebug.__init__(self, device)

        self._modem_api = self._device.get_uecmd("Modem")
        self._config = None
        if config is not None:
            if self.CONFIG_FREQUENCIES in config.keys():
                self._config = self.CONFIG_FREQUENCIES
            elif self.CONFIG_CHANNELS in config.keys():
                self._config = self.CONFIG_CHANNELS
            else:
                msg = "SafeRangeBase configuration is not good!"
                self._logger.error(msg)
                raise DeviceException(DeviceException.INVALID_PARAMETER, msg)
        else:
            self._config = self.CONFIG_CHANNELS

        self._coex_mgr_version = self._modem_api.get_lte_coex_manager_version()
        self._trigger_group = []

        if self._coex_mgr_version == 1:
            self._aplog_level = "D"
            self._retrieve_aplog_regexps.append("COEX_SAFECHANNELS *:")
        elif self._coex_mgr_version == 2:
            self._aplog_level = "I"
            if self._domains == 'WIFI':
                # We'll compute the frequency from the channel list.
                self._retrieve_aplog_regexps.append("CWS_CELLCOEX_.*mws.wifi.safeChannels: setValue(\[")
            else:
                # BT and BLE have same source for parsing
                self._retrieve_aplog_regexps.append("CWS_CELLCOEX_.*BT Safe frequency range")


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
        SysDebug.stats(self, start, stop)
        domain = self._domains
        xmltree = etree.Element("SafeRange%s" % domain)
        match_log = ""
        for cur_log in reversed(self._sys_debug_infos):
            if "%s_safe_range_min" % domain.lower() in cur_log.keys():
                xmlrange = etree.Element("SafeRange")
                xmlrange.attrib["min"] = cur_log["%s_safe_range_min" % domain.lower()]
                xmlrange.attrib["max"] = cur_log["%s_safe_range_max" % domain.lower()]
                xmltree.append(xmlrange)
                match_log = cur_log
                break

        self._logger.debug("SafeRange%s stats : %s" % (domain, str(match_log)))
        return xmltree
