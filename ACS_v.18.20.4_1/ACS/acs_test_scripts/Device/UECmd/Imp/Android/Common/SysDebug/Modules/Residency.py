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
          This catch the occurrence of reboots
:since: 28/02/2013
:author: pbluniex
"""

from lxml import etree
from acs_test_scripts.Device.UECmd.Imp.Android.Common.SysDebug.SysDebug import SysDebug


class Residency(SysDebug):

    """
    :summary: System UEcommands operations for Android platforms
    """

    def __init__(self, device, _config=None):
        """
        Constructor.

        :type device: DeviceBase
        :param device: The DUT
        """
        SysDebug.__init__(self, device)

        self.__residencies_api = self._device.get_uecmd("Residencies")
        self._use_aplog = False

    def init(self):
        """
        Initialization
        """
        return self.__residencies_api.device_has_residencies()

    def _retrieve_sysinfo(self):
        """
        Retrieve sysinfo from device: Nothing to do
        """
        pass

    def fetch(self):
        """
        Fetch sysdebug information on a connected device
        """
        self._sys_debug_infos = self.__residencies_api.get()

    def reset(self, delay_s=0):
        """
        Reset residency
        """
        self.__residencies_api.clear(delay_s)

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

        xmltree = etree.Element("Residencies")
        for item in self._sys_debug_infos:
            residency = etree.Element("Residency")
            residency.attrib["mode"] = item["mode"].lower()
            residency.attrib["wakup_count"] = str(item.get("count", ""))
            residency.attrib["sleep_time"] = str(item.get("time", ""))
            residency.attrib["residency"] = str(item.get("residency", ""))
            xmltree.append(residency)

        self._logger.debug("Residency stats : %s" % str(self._sys_debug_infos))
        return xmltree
