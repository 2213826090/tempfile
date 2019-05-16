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
:summary: This file implements Networking UECmds for Android JB device
:since: 12 sep 2012
:author: sfusilie
"""
import re
from acs_test_scripts.Device.UECmd.Imp.Android.ICS.Networking.Networking import Networking as NetworkingICS
from acs_test_scripts.Device.UECmd.UECmdDecorator import need
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException


class Networking(NetworkingICS):

    """
    Class that handle all networking operations
    """

    # Wifi frequencies bands supported by JB.
    SUPPORTED_WIFI_BANDS = {"auto": 0, "5GHz": 1, "2.4GHz": 2}

    def __init__(self, phone):
        """
        Constructor
        """
        NetworkingICS.__init__(self, phone)

    @need('wifi')
    def set_wifi_frequency_band(self, freq_band, silent_mode=False, _interface="wlan0"):
        """
        Set the Wifi Frequency Band

        :type freq_band: String
        :param freq_band: Frequency Band to set (auto, 2.4GHz, 5GHz)

        :type silent_mode: boolean
        :param silent_mode: if True, do not raise an exception
                            if the device does not support this method
        :type interface: str
        :param interface: interface name (wlan0/wlan1 etc...)
        """
        self._logger.info("Set Wifi frequency band to: %s" % str(freq_band))
        # Send the intent to set the Wifi Frequency Band selection
        if freq_band not in self.SUPPORTED_WIFI_BANDS:
            msg = "Invalid value for frequency band"
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
        cmd = "adb shell wpa_cli DRIVER SETBAND %s" % \
            self.SUPPORTED_WIFI_BANDS[freq_band]
        result = self._exec(cmd, timeout=5, wait_for_response=True)
        if "OK" not in result:
            msg = "Error executing setband command"
            self._logger.error(msg)
            if not silent_mode:
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

    @need('wifi')
    def get_wifi_frequency_band(self, silent_mode=False, _interface="wlan0"):
        """
        Gets the band selection (bands of frequencies)
        0 means dual
        1 means 5Ghz
        2 means 2.4Ghz

        :type silent_mode: boolean
        :param silent_mode: if True, do not raise an exception if the device does not support this method
        :type interface: str
        :param interface: interface name (wlan0/wlan1 etc...)

        :rtype: String
        :return: The band text (JB or later).

        """
        cmd = "adb shell wpa_cli DRIVER GETBAND"
        results = self._exec(cmd, timeout=5, wait_for_response=True)
        band_id = None
        for line in results.split("\n"):
            m = re.search(r'^Band ([0-2])', line)
            if m is not None:
                band_id = int(m.group(1))
                break

        match = [k for k, v in self.SUPPORTED_WIFI_BANDS.iteritems() if v == band_id]
        if match:
            result = match[0]
        else:
            result = None

        self._logger.debug("Wifi frequency band: %s" % str(result))
        return result

    def disable_ip_filtering(self, phy):
        """
        Disables all IP filtering

        :type phy: String
        :param: phy the physical interface to disable the filter to.
        """
        cmd = "adb shell iw phy %s wowlan disable" % phy
        self._exec(cmd)

    def enable_ip_filtering(self, phy):
        """
        Enables all IP filtering

        :type phy: String
        :param: phy the physical interface to enable the filter to.
        """
        cmd = "adb shell iw phy %s wowlan enable" % phy
        self._exec(cmd)
