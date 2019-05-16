"""
:copyright: (c)Copyright 2015, Intel Corporation All Rights Reserved.
The source code contained or described here in and all documents related to the source code ("Material") are owned by
Intel Corporation or its suppliers or licensors. Title to the Material remains with Intel Corporation or its suppliers
and licensors. The Material contains trade secrets and proprietary and confidential information of Intel or its
suppliers and licensors.

The Material is protected by worldwide copyright and trade secret laws and treaty provisions. No part of the Material
may be used, copied, reproduced, modified, published, uploaded, posted, transmitted, distributed, or disclosed
in any way without Intel's prior express written permission.

No license under any patent, copyright, trade secret or other intellectual property right is granted to or conferred
upon you by disclosure or delivery of the Materials, either expressly, by implication, inducement, estoppel or
otherwise. Any license under such intellectual property rights must be express and approved by Intel in writing.

:organization: INTEL MCG
:summary: This file implements an UECmd SysDebug class for Android device
          This catch the occurrence of the softap used channel.
:since: 2015-01-09
:author: emarchan

"""

from lxml import etree
from acs_test_scripts.Device.UECmd.Imp.Android.Common.SysDebug.SysDebug import SysDebug
from ErrorHandling.DeviceException import DeviceException
from UtilitiesFWK.Utilities import Global


class WifiSoftApChannel(SysDebug):

    def __init__(self, device, config=None):
        """
        Constructor.

        :type config: dict
        :param config: Defines what this instance will return: Use 'frequencies' to have the safe frequencies range or
        'channels' for the frequencies converted in channels.
        """
        SysDebug.__init__(self, device)

        self._trigger_group = [["soft_ap_channel", ".*Wi-Fi Tethering running on channel ", "\d+", "$"]]

        self._aplog_level = "I"
        self._retrieve_aplog_regexps.append("CWS_CELLCOEX_.*Wi-Fi Tethering running on channel")


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
        xmltree = etree.Element("WifiSoftApChannel")
        match_log = ""
        for cur_log in reversed(self._sys_debug_infos):
            if "soft_ap_channel" in cur_log.keys():
                xmltree.attrib["value"] = cur_log["soft_ap_channel"]
                match_log = cur_log
                break

        self._logger.debug("WifiSoftApChannel stats : %s" % (str(match_log)))
        return xmltree
