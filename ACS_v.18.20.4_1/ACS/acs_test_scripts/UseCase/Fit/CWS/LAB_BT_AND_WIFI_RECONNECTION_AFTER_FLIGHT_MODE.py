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
:summary: This file implements the LIVE_BT_AND_WIFI_RECONNECTION_AFTER_FLIGHT_MODE
:author: jfranchx
:since:01/08/2013
"""

import time

from acs_test_scripts.UseCase.LocalConnectivity.LIVE_BT_BASE import LiveBTBase
from acs_test_scripts.UseCase.Networking.LIVE_WIFI_BASE import LiveWifiBase
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.Utilities.LocalConnectivityUtilities import a2dp_pair_connect_to_headset, a2dp_unpair_headset
from acs_test_scripts.Device.UECmd.UECmdTypes import BtProfile, BtConState
from ErrorHandling.DeviceException import DeviceException


class LabBTAndWiFiReconnectionAfterFlightMode(LiveBTBase, LiveWifiBase):
    """
    Lab BT and WiFi reconnection after switch flight mode on/off
    """

    WAIT_BT_WIFI_AUTO_RECONNECTION = 20.0
    WAIT_TIME_IN_FLIGHT_MODE = 5.0
    MAX_PING_PACKET_LOST = 25.0

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call LiveBTBase init function
        LiveBTBase.__init__(self, tc_name, global_config)
        LiveWifiBase.__init__(self, tc_name, global_config)

        self._wifi_ip = self._wifirouter.get_param_value("IP")
        self._bt_headset = self._em.get_bluetooth_headset("BT_HEADSET")

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        LiveBTBase.set_up(self)
        LiveWifiBase.set_up(self)

        # Connect BT to Headset
        a2dp_pair_connect_to_headset(self._bt_api, self._bt_headset)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """

        # Call UseCase base run_test function
        LiveBTBase.run_test(self)

        # Switch flight mode
        self._networking_api.set_flight_mode(1)
        time.sleep(LabBTAndWiFiReconnectionAfterFlightMode.WAIT_TIME_IN_FLIGHT_MODE)
        self._networking_api.set_flight_mode(0)
        time.sleep(LabBTAndWiFiReconnectionAfterFlightMode.WAIT_BT_WIFI_AUTO_RECONNECTION)

        # Check BT connection
        bt_state = self._bt_api.get_bt_connection_state(self._bt_headset.get_bdaddress(), BtProfile.A2DP)
        if bt_state != BtConState.d[BtConState.CONNECTED]:
            msg = "Error BT reconnection - State %s" % str(bt_state)
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        # Check WiFi connection
        packet_loss = self._networking_api.ping(self._wifi_ip, 16, 16)
        if packet_loss.value > LabBTAndWiFiReconnectionAfterFlightMode.MAX_PING_PACKET_LOST:
            msg = "Error WiFi reconnection - Ping %s fail with %s lost" % self._wifi_ip % packet_loss
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        Finish the test and clear environment
        """

        # Call LiveWifiBase base tear_down function
        LiveWifiBase.tear_down(self)

        # Disable BT
        a2dp_unpair_headset(self._bt_api, self._bt_headset)
        time.sleep(self._wait_btwn_cmd)
        self._bt_api.set_bt_power(0)

        return Global.SUCCESS, "No errors"
