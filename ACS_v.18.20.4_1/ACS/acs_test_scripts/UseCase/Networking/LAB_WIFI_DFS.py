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
:summary: Test the DFS frequency change capabilities
:since: 03/01/2013
:author: smaurelx
"""

from acs_test_scripts.Device.UECmd.UECmdTypes import AUTO_CONNECT_STATE
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.Networking.LAB_WIFI_BASE import LabWifiBase
import time
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException
from acs_test_scripts.Utilities.NetworkingUtilities import AcsWifiFrequencies


class LabWifiDfs(LabWifiBase):

    """
    Lab Wifi Connect Test class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        LabWifiBase.__init__(self, tc_name, global_config)

        # Retrieve UC parameter
        if self._channel.isdigit() and int(self._channel) in AcsWifiFrequencies.WIFI_CHANNELS_FREQUENCIES_5G:
            self._first_freq = AcsWifiFrequencies.WIFI_CHANNELS_FREQUENCIES_5G[int(self._channel)]
        else:
            self._first_freq = None

        self._second_channel = str(self._tc_parameters.
                                   get_param_value("SECOND_WIFI_CHANNEL"))
        if self._second_channel.isdigit() and int(self._second_channel) in AcsWifiFrequencies.WIFI_CHANNELS_FREQUENCIES_5G:
            self._second_freq = AcsWifiFrequencies.WIFI_CHANNELS_FREQUENCIES_5G[int(self._second_channel)]
        else:
            self._second_freq = None

        self._nbr_beacon_to_wait = str(self._tc_parameters.
                                       get_param_value("NBR_BEACON_TO_WAIT"))

        self._dfs_mode = str(self._tc_parameters.
                                       get_param_value("DFS_MODE"))

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        LabWifiBase.set_up(self)
        self._set_up()
        return Global.SUCCESS, "No errors"

    def _set_up(self):

        """
        Initialize the test
        """
        # Checking parameters data
        if self._first_freq is None or (self._channel.isdigit() and int(self._channel) == 0):
            msg = "First frequency not found."
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
        if self._second_freq is None or (self._second_channel.isdigit() and int(self._second_channel) == 0):
            msg = "Second frequency not found."
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
        if self._nbr_beacon_to_wait.lower() in ["", "none"]:
            msg = "NBR_BEACON_TO_WAIT parameter missing or set to none."
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        if self._dfs_mode.lower() in ["on", "enabled", "1"]:
            self._dfs_mode = "enable"
        else:
            msg = "_dfs_mode parameter missing or set to none."
            self._dfs_mode = "disable"
            self._logger.warning(msg)
        if self._beacon is None:
            self._beacon = self._ns.get_beacon_period()

        self._networking_api.set_autoconnect_mode(self._ssid,
                                                  AUTO_CONNECT_STATE.on)  # pylint: disable=E1101

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        LabWifiBase.run_test(self)
        self._trigger_dfs()
        return Global.SUCCESS, "No errors"

    def _trigger_dfs(self):
        """
        Trigger DFS on AP
        """
        # Check if connected on good SSID and good Channel / Frequency
        self._networking_api.check_connection_state(self._ssid)
        self._networking_api.request_wifi_scan()
        # TODO remove time.sleep(30) once the request_wifi_scan() is changed to synchronized call
        time.sleep(30)
        ssids, frequencies = self._networking_api.list_ssids_and_frequency(
            "wifi", "connected")
        current_freq = ""
        for current in range(len(ssids)):
            if self._ssid == ssids[current]:
                current_freq = frequencies[current]
                break

        if current_freq == self._first_freq:
            self._logger.info('Device connected to %s %s' % (self._ssid,
                                                             current_freq))
        else:
            msg = "Bad SSID frequency %s %s" % (self._ssid, current_freq)
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        # Switch AP from First Channel to Second channel
        self._ns.init()
        self._ns.simulate_dfs(self._channel, self._second_channel,
                             self._nbr_beacon_to_wait, self._dfs_mode)
        self._ns.release()

        # Wait before check automatic reconnection
        # +5 seconds to ensure all is right
        time.sleep(float(self._nbr_beacon_to_wait) *
                       (float(self._beacon) / 1000.0) + 5.0)

        # Check if connected on good SSID and new good Channel / Frequency
        self._networking_api.check_connection_state(self._ssid)
        self._networking_api.request_wifi_scan()
        ssids, frequencies = self._networking_api.list_ssids_and_frequency(
            "wifi", "connected")
        current_freq = ""
        for current in range(len(ssids)):
            if self._ssid == ssids[current]:
                current_freq = frequencies[current]
                break

        if current_freq == self._second_freq:
            self._logger.info('Device connected to %s %s' % (self._ssid,
                                                             current_freq))
        else:
            msg = "Bad SSID frequency %s %s" % (self._ssid, current_freq)
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)
