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
:summary: This file implements the LAB WIFI CONNECT and FORGET UC
:since: 16/04/2012 BZ2836
:author: apairex
"""

import time
from acs_test_scripts.UseCase.Networking.LAB_WIFI_BASE import LabWifiBase
from UtilitiesFWK.Utilities import Global
from ErrorHandling.DeviceException import DeviceException


class LabWifiConnectForget(LabWifiBase):

    """
    Lab Wifi Connect After Wifi interface off & on & check AP can be forgotten.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        LabWifiBase.__init__(self, tc_name, global_config)

        # Parameter for ping controls
        self._packetsize = 32
        self._packnb = 4
        self._lossrate = 25

        # Time to wait for wifi fully switch off
        self._wifi_switchoff_time = 10
        # Time to wait for wifi fully switch on
        self._wifi_switchon_time = 25

        # Get UECmdLayer for bluetooth
        self._bt_api = self._device.get_uecmd("LocalConnectivity")

        # Record whether Wifi is already connected to handle B2B continuous mode values
        self._is_wifi_already_connected = None

        # Time to wait for SSID list refresh
        self._waiting_check_time = 120

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        # pylint: disable=E1101
        LabWifiBase.set_up(self)

        # Enable Wifi autoconnect on our SSID
        self._networking_api.\
            set_autoconnect_mode(self._ssid,
                                 self._uecmd_types.AUTO_CONNECT_STATE.on)

        # Wait 3 sec minimun after connect
        if self._wait_btwn_cmd <= 3:
            time.sleep(3)
        else:
            time.sleep(self._wait_btwn_cmd)

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

        # Check Wifi connection is available
        packetloss = self._networking_api.ping(self._wifirouter_ip,
                                               self._packetsize,
                                               self._packnb)

        if packetloss.value > self._lossrate:
            msg = "Fail to recover Wifi connection after enabling wifi interface " + \
                "(ping NOK: %s%% packet loss)" % str(packetloss.value)
            self._logger.error(msg)
            raise DeviceException(DeviceException.PROHIBITIVE_MEASURE, msg)

        # Disable Wifi interface
        self.__set_wifi_off()
        time.sleep(self._wifi_switchoff_time)

        # Enable Wifi interface
        self.__set_wifi_on()
        time.sleep(self._wifi_switchon_time)

        # Check connection to the SSID
        self._networking_api.check_connection_state(self._ssid)

        # Check Wifi connection is back
        packetloss = self._networking_api.ping(self._wifirouter_ip,
                                               self._packetsize,
                                               self._packnb)

        if packetloss.value > self._lossrate:
            msg = "Fail to recover Wifi connection after enabling wifi interface " \
                  + "(ping NOK: %s%% packet loss)" % str(packetloss.value)
            self._logger.error(msg)
            raise DeviceException(DeviceException.PROHIBITIVE_MEASURE, msg)

        # Forget AP SSID
        self._networking_api.wifi_remove_config(self._ssid)

        if self._hidden:
            # SSID not broadcasted. Check that the SSID disappears from the list.
            self._logger.info("Waiting %d seconds for SSID to disappear from scan list"
                              % self._waiting_check_time)
            time.sleep(self._waiting_check_time)

            # Check AP is well forgotten and no connection is made on it
            all_network_list = self._networking_api.list_ssids()

            if self._ssid in all_network_list:
                msg = "SSID %s is unexpectedly present in the list" % str(self._ssid)
                self._logger.error(msg)
                raise DeviceException(DeviceException.PROHIBITIVE_MEASURE, msg)
        else:
            # SSID broadcasted. Check that the Connection is not established.
            connected_network_list = self._networking_api.list_connected_wifi()

            if self._ssid in connected_network_list:
                msg = "DUT is unexpectedly connected to %s " % str(self._ssid)
                self._logger.error(msg)
                raise DeviceException(DeviceException.PROHIBITIVE_MEASURE, msg)

        self._is_wifi_already_connected = False

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """
        LabWifiBase.tear_down(self)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def __set_wifi_off(self):
        """
        Set Wifi off using the appropriate method:
        - direct Wifi interface control
        - using flight mode control
        """
        if self._use_flight_mode == 1:
            self._networking_api.set_flight_mode("on")
        else:
            self._networking_api.set_wifi_power("off")

#------------------------------------------------------------------------------

    def __set_wifi_on(self):
        """
        Set Wifi on using the appropriate method:
        - direct Wifi interface control
        - using flight mode control
        """
        if self._use_flight_mode == 1:
            self._networking_api.set_flight_mode("off")
        else:
            self._networking_api.set_wifi_power("on")
