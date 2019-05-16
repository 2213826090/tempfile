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
:summary: This file implements the LAB_DUAL_PHONE_BT_TETHERING_PING
Establish bluetooth connection between 2 PHONES in order to use bluetooth tethering profile
and share Wifi connection to a LAB access point
:author: apairex
:since:17/04/2013
"""

import time

from LIVE_DUAL_PHONE_BT_BASE import LiveDualPhoneBTBase
from acs_test_scripts.UseCase.Networking.LAB_WIFI_BASE import LabWifiBase
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from UtilitiesFWK.Utilities import Global, str_to_bool, is_number
from acs_test_scripts.Utilities.NetworkingUtilities import check_connection_lost
from acs_test_scripts.Utilities.LocalConnectivityUtilities import disconnect_tethering, \
    connect_to_nap
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsConfigException import AcsConfigException


class LabDualPhoneBTTetheringPing(LiveDualPhoneBTBase, LabWifiBase):

    """
    Live Dual phone BT Connect, and send  message
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call LiveDualPhoneBTBase and LabWifiBase init functions
        LiveDualPhoneBTBase.__init__(self, tc_name, global_config)
        LabWifiBase.__init__(self, tc_name, global_config)

        # Reset unused parameter:
        self._wrong_passphrase = None
        self._key_exchange_should_fail = False

        # Read TC parameters
        self._nap_or_pan_test = \
            str(self._tc_parameters.get_param_value("NAP_OR_PAN_TEST")).upper()
        self._pairing_initiator = \
            str(self._tc_parameters.get_param_value("PAIRING_INITIATOR")).upper()
        self._who_disconnect = \
            str(self._tc_parameters.get_param_value("WHO_DISCONNECT")).upper()
        self._who_restarts_bt = \
            str(self._tc_parameters.get_param_value("WHO_RESTARTS_BT_BEFORE_TEST")).upper()
        self._bt_tethering_deactivation_test = \
            str(self._tc_parameters.get_param_value("ENABLE_BT_TETHERING_DEACTIVATION_TEST", ""))
        self._lola_test = \
            str(self._tc_parameters.get_param_value("ENABLE_LOLA_TEST", ""))
        self._packet_count = \
            str(self._tc_parameters.get_param_value("PACKET_COUNT"))
        self._packet_size = \
            str(self._tc_parameters.get_param_value("PACKET_SIZE"))
        self._target_packet_loss_rate = \
            str(self._tc_parameters.get_param_value("TARGET_PACKET_LOSS_RATE"))

        # Server to ping
        self._server_to_ping = self._wifirouter_ip

        self._nap_api = None
        self._panu_api = None
        self._nap_addr = None
        self._panu_addr = None
        self._wifi_api = None
        self._panu_net_api = None
        self._paired_api = None
        self._pair_requester_api = None

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        LiveDualPhoneBTBase.set_up(self)

        # Check TC Parameters
        self.__check_tc_parameters()

        # defines NAP / PAN-User roles
        if self._nap_or_pan_test == "NAP":
            self._nap_api = self._bt_api
            self._nap_addr = self._phone1_addr
            self._panu_api = self._bt_api2
            self._panu_addr = self._phone2_addr
            self._wifi_api = self._networking_api
            self._panu_net_api = self._networking_api2
        else:
            self._nap_api = self._bt_api2
            self._nap_addr = self._phone2_addr
            self._panu_api = self._bt_api
            self._panu_addr = self._phone1_addr
            self._wifi_api = self._networking_api2
            self._panu_net_api = self._networking_api

        # Connection Wifi LAB Access Point
        LabWifiBase.set_up_without_flightmode(self, self._wifi_api)

        # Deactivate DATA on PAN User device
        self._panu_net_api.deactivate_pdp_context()

        # Establish bluetooth pairing
        self._establish_bt_pairing(self._pairing_initiator)

        # Set NAP back to discoverable and connectable
        # as _establish_bt_pairing might have set it to "noscan"
        # breaking the test
        self._nap_api.set_bt_discoverable("both", 0)

        # Enable BT tethering
        self._nap_api.set_bt_tethering_power("1")

        # Handle LOng LAsting test
        if self._lola_test:
            connect_to_nap(self._panu_api, self._nap_api,
                           self._nap_addr, self._who_restarts_bt, self._wait_btwn_cmd)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        # Call UseCase base run_test function
        LiveDualPhoneBTBase.run_test(self)

        # BT Tethering Connection
        if not self._lola_test:
            connect_to_nap(self._panu_api, self._nap_api,
                           self._nap_addr, self._who_restarts_bt, self._wait_btwn_cmd)
        else:
            self._logger.info("Sleeping 60 seconds")
            time.sleep(60)

        # Check connection with ping command
        packet_loss = self._panu_net_api.ping(self._server_to_ping,
                                              self._packet_size,
                                              self._packet_count)

        msg = "Measured Packet Loss: %.0f%s (Target: %.0f%s)" \
            % (packet_loss.value, packet_loss.units,
               self._target_packet_loss_rate, packet_loss.units)
        self._logger.info(msg)

        if packet_loss.value > self._target_packet_loss_rate:
            msg = "Ping packet loss is not acceptable [%s]" \
                % str(packet_loss.value)
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        # BT Tethering Disconnection
        if not self._lola_test:
            self.__disconnect()

        return (Global.SUCCESS,
                "BT Tethering ping %s OK" % self._server_to_ping)

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """
        UseCaseBase.tear_down(self)

        # Handle LOng LAsting test
        if self._lola_test:
            self.__disconnect()

        # Disable BT tethering
        self._nap_api.set_bt_tethering_power("0")

        # Bluetooth unpairing
        self._logger.info("Unpair both devices")
        self._bt_api.unpair_bt_device(self._phone2_addr)
        self._bt_api2.unpair_bt_device(self._phone1_addr)

        # Disconnect and clear WiFi networks
        LabWifiBase.tear_down_without_flightmode(self, self._wifi_api)

        LiveDualPhoneBTBase.tear_down(self)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def __check_tc_parameters(self):
        """
        Check validity of TC parameters
        """
        self._bt_tethering_deactivation_test = \
            str_to_bool(str(self._bt_tethering_deactivation_test))
        self._lola_test = str_to_bool(str(self._lola_test))

        if self._nap_or_pan_test in ["PAN", "PANU", "PAN-U"]:
            self._nap_or_pan_test = "PAN-U"
        if self._nap_or_pan_test not in ["NAP", "PAN-U"]:
            msg = "Wrong TC parameter NAP_OR_PAN_TEST"
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
        if self._pairing_initiator not in ["PHONE1", "PHONE2"]:
            msg = "Wrong TC parameter PAIRING_INITIATOR"
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
        if self._who_disconnect in ["PAN", "PANU", "PAN-U"]:
            self._who_disconnect = "PAN-U"
        if self._who_disconnect not in ["NAP", "PAN-U"]:
            msg = "Wrong TC parameter WHO_DISCONNECT"
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
        if self._who_restarts_bt in ["PAN", "PANU", "PAN-U"]:
            self._who_restarts_bt = "PAN-U"
        if self._who_restarts_bt not in ["NONE", "NAP", "PAN-U"]:
            msg = "Wrong TC parameter WHO_RESTARTS_BT_BEFORE_TEST"
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
        if not self._packet_count.isdigit():
            msg = "Wrong TC parameter PACKET_COUNT"
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
        if not self._packet_size.isdigit():
            msg = "Wrong TC parameter PACKET_SIZE"
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
        if not is_number(self._target_packet_loss_rate):
            msg = "Wrong TC parameter TARGET_PACKET_LOSS_RATE"
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
        else:
            self._target_packet_loss_rate = \
                float(self._target_packet_loss_rate)

    def __disconnect(self):
        """
        Disconnect the BT tethering link
        """
        # Handle "PANU got kicked" test
        if self._bt_tethering_deactivation_test:
            # Disable BT Tethering on NAP Device
            self._nap_api.set_bt_tethering_power("off")

            # Checks that the connection is broken
            check_connection_lost(self._panu_net_api,
                                  self._server_to_ping,
                                  self._packet_size,
                                  self._packet_count,
                                  self._logger)

            # Enable BT Tethering on NAP Device
            self._nap_api.set_bt_tethering_power("on")
            time.sleep(self._wait_btwn_cmd)

        # Disconnect Bluetooth PAN profile
        disconnect_tethering(self._who_disconnect, self._nap_api, self._panu_api,
                             self._nap_addr, self._panu_addr)
        time.sleep(self._wait_btwn_cmd)
