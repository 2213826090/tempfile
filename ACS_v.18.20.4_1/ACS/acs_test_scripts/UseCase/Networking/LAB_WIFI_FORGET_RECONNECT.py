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
:summary: This file implements the LAB WIFI FORGET and RECONNECT UC
:since: 28/02/2014
:author: jfranchx
"""

import time
from acs_test_scripts.UseCase.Networking.LAB_WIFI_BASE import LabWifiBase
from UtilitiesFWK.Utilities import Global
from ErrorHandling.DeviceException import DeviceException


class LabWifiForgetReconnect(LabWifiBase):

    """
    Lab Wifi Connect/Forget and reconnect & check AP can be forgotten.
    """

    # Time to wait before check
    WAITING_TIME = 120

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        LabWifiBase.__init__(self, tc_name, global_config)

        # Parameter for ping controls
        self._packetsize = 32
        self._packnb = 4
        self._lossrate = 25

        # Record whether Wifi is already connected to handle B2B continuous mode values
        self._is_wifi_already_connected = None

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        # pylint: disable=E1101
        LabWifiBase.set_up(self)

        # Checks connection using Ping
        packetloss = self._networking_api.ping(self._wifirouter_ip,
                                               self._packetsize,
                                               self._packnb)

        if packetloss.value > self._lossrate:
            msg = "Ping command fails (%s%% loss)" % str(packetloss.value)
            self._logger.error(msg)
            raise DeviceException(DeviceException.PROHIBITIVE_MEASURE, msg)

        self._is_wifi_already_connected = True

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        LabWifiBase.run_test(self)

        # Reconnect only if needed (B2B continuous mode = True)
        if self._is_wifi_already_connected is not True:
            self._networking_api.set_wificonfiguration(self._ssid,
                                                       self._passphrase,
                                                       self._security,
                                                       self._ip_setting,
                                                       self._ip_address,
                                                       self._netmask,
                                                       self._gateway,
                                                       self._dns1,
                                                       self._dns2)
            self._networking_api.wifi_connect(self._ssid)


        # Check connection to the SSID
        self._networking_api.check_connection_state(self._ssid)

        # Check Wifi connection is available
        packetloss = self._networking_api.ping(self._wifirouter_ip,
                                               self._packetsize,
                                               self._packnb)

        if packetloss.value > self._lossrate:
            msg = "Fail to recover Wifi connection after enabling WiFi interface " + \
                "(ping NOK: %s%% packet loss)" % str(packetloss.value)
            self._logger.error(msg)
            raise DeviceException(DeviceException.PROHIBITIVE_MEASURE, msg)

        # Forget AP SSID
        self._networking_api.wifi_remove_config(self._ssid)
        self._is_wifi_already_connected = False
        time.sleep(self.WAITING_TIME)

        # Check if SSID is remembered
        connected_wifi_list = self._networking_api.list_connected_wifi()
        if self._ssid in connected_wifi_list:
            msg = "WiFi is connected to %s network and it shouldn't!" % self._ssid
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        # Confirm connection is still possible
        self._networking_api.set_wificonfiguration(self._ssid,
                                                   self._passphrase,
                                                   self._security,
                                                   self._ip_setting,
                                                   self._ip_address,
                                                   self._netmask,
                                                   self._gateway,
                                                   self._dns1,
                                                   self._dns2)
        self._networking_api.wifi_connect(self._ssid, True)
        self._is_wifi_already_connected = True

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """
        LabWifiBase.tear_down(self)

        return Global.SUCCESS, "No errors"
