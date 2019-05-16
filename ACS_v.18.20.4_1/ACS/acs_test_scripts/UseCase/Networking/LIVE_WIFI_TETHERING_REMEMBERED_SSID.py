"""
:copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
The source code contained or described herein and all documents related
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
:summary: This file implements LIVE WIFI TETHERING REMEMBERED SSID UC
:since: 16/10/2012
:author: apairex RTC14451
"""
import time

from LIVE_WIFI_TETHERING_BASE import LiveWifiTetheringBase
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from Device.DeviceManager import DeviceManager
from UtilitiesFWK.Utilities import Global, str_to_bool_ex
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException


class LiveWifiTetheringRememberedSsid(LiveWifiTetheringBase):

    """
    Live wifi Tethering ping.
    """

    def __init__(self, tc_name, global_config):  # pylint: disable=W0231
        """
        Constructor
        """
        # Call USECASE BASE Init function
        LiveWifiTetheringBase.__init__(self, tc_name, global_config)  # pylint: disable=W0233

        # Get host spot configuration according HOTSPOT_SSID_BASENAME
        self._hotspot_ssid_basename = \
            str(self._tc_parameters.get_param_value("HOTSPOT_SSID_BASENAME"))

        # Get host spot configuration according HOTSPOT_SECURITIES
        self._hotspot_securities = \
            str(self._tc_parameters.get_param_value("HOTSPOT_SECURITIES"))
        self._hotspot_securities = self._hotspot_securities.split("|")

        # Get UECmdLayer
        self._networking_api = self._device.get_uecmd("Networking")

        # Get PHONE2
        self._networking_api2 = None
        self._phone2 = DeviceManager().get_device("PHONE2")
        if self._phone2 is not None:
            self._networking_api2 = self._phone2.get_uecmd("Networking")

        # init original wifi power status for phone1
        self._original_wifi_power_status = 0
        # init original flight mode for phone1
        self._original_flight_mode = 0
        self._interface_ip = None

        self._hotspot_ext_interface = \
            str(self._dut_config.get("hotspotExtInterface"))

        # Read the number of pings to do
        self._nb_pings = self._tc_parameters.get_param_value("PACKET_COUNT")

        # Read the data size of a packet
        self._packet_size = self._tc_parameters.get_param_value("PACKET_SIZE")

        # Get target % of received packet for ping
        self._target_ping_packet_loss_rate = \
            float(self._tc_parameters.get_param_value("TARGET_PACKET_LOSS_RATE"))

        self._waiting_time = \
            int(self._tc_parameters.get_param_value("DELAY_AFTER_PING"))

        # Get the IP server to ping
        self._server_ip = self._tc_parameters.\
            get_param_value("SERVER_TO_PING")
        self._server_2_ping = None

        # Set BT prerequisite (set ON, set OFF or nothing) for FIT BT/WIFI tests
        self._bt_fit_used, self._bt_wished_value = LiveWifiTetheringBase._get_bt_fit_config(self)
        # Instantiate generic UECmd
        self._localconnectivity_api = self._device.get_uecmd("LocalConnectivity")
        self._bt_initial_state = 'STATE_ON'

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        UseCaseBase.set_up(self)

        # Check if we have second wifi interface or second phone available
        if self._networking_api2 is None:
            msg = "PHONE2 is required in BenchConfig"
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        # store original wifi power status
        time.sleep(self._wait_btwn_cmd)
        self._original_wifi_power_status = \
            self._networking_api.get_wifi_power_status()

        # store original flight mode
        time.sleep(self._wait_btwn_cmd)
        self._original_flight_mode = self._networking_api.get_flight_mode()

        # Enable/Disable flight mode
        if str_to_bool_ex(self._original_flight_mode) != str_to_bool_ex(self._flight_mode):
            time.sleep(self._wait_btwn_cmd)
            self._networking_api.set_flight_mode(self._flight_mode)

        # Enable Cellular Data connection
        if self._flight_mode == False and self._is_pdp_context_activated:
            time.sleep(self._wait_btwn_cmd)
            self._networking_api.activate_pdp_context()

        # disable wifi for Phone1
        time.sleep(self._wait_btwn_cmd)
        self._networking_api.set_wifi_power("off")
        time.sleep(self._wait_btwn_cmd)
        self._networking_api.set_wifi_hotspot("off")

        # Boot the other phone (the DUT is already booted)
        if self._phone2.get_state() != "alive" \
                or not self._phone2.is_available():
            DeviceManager().boot_device("PHONE2")

        # If Bt to be (de-)activated for FIT tests
        if self._bt_fit_used:

            self._bt_initial_state = self._localconnectivity_api.get_bt_power_status()

            # if Bt is not at the wished value, set it to the correct one.
            if self._bt_initial_state != self._bt_wished_value:
                self._localconnectivity_api.set_bt_power(self._bt_wished_value)
                time.sleep(self._wait_btwn_cmd)

        # set wifi power on for Phone2
        time.sleep(self._wait_btwn_cmd)
        self._networking_api2.set_wifi_power("on")

        # disconnect all wifi for Phone2
        time.sleep(self._wait_btwn_cmd)
        self._networking_api2.wifi_remove_config("all")

        # Set networks as remembered SSIDs
        for security in self._hotspot_securities:

            # Build the SSID name
            ssid = self._hotspot_ssid_basename + "_" + security

            # set ssid for Phone2 as remembered network
            time.sleep(self._wait_btwn_cmd)
            self._networking_api2.set_wificonfiguration(ssid,
                                                        self._hotspot_passphrase,
                                                        security)

        # Need to restart Wifi interface to enable SSIDs
        self._networking_api2.set_wifi_power(0)
        time.sleep(self._wait_btwn_cmd)
        self._networking_api2.set_wifi_power(1)
        time.sleep(self._wait_btwn_cmd)
        self._networking_api2.set_wifi_sleep_policy(
            self._networking_api.WIFI_SLEEP_POLICY["NEVER"])

        return Global.SUCCESS, "No error"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        UseCaseBase.run_test(self)

        self._error.Msg = ""
        self._error.Code = Global.SUCCESS

        for security in self._hotspot_securities:

            # Build the SSID name
            ssid = self._hotspot_ssid_basename + "_" + security

            # enable Portable wifi hotspot for Phone1
            time.sleep(self._wait_btwn_cmd)
            self._networking_api.set_wifi_hotspot("off")
            time.sleep(self._wait_btwn_cmd)
            self._networking_api.set_wifi_hotspot("on", ssid, security,
                                                  self._hotspot_passphrase)

            # Wait for the PHONE2 to auto connect
            self._logger.info("Waiting 60 sec for the connection to establish")
            time.sleep(60)
            self._networking_api2.check_connection_state(ssid)

            # Correct the IP server 2 ping if necessary, once
            if self._server_2_ping is None:
                if str(self._server_ip) in ["None", ""] or self._flight_mode:
                    # Get the IP of the softAP wifi interface
                    self._server_2_ping = self._networking_api.\
                        get_interface_ipv4_address(self._hotspot_ext_interface)

                    if str(self._server_ip) not in ["None", ""] \
                            and self._flight_mode:
                        # Display a warning because the given parameter
                        # cannot be used
                        self._logger.warning("IP server set as parameter " +
                                             "is ignored, as test is run " +
                                             "in flight mode. Using SoftAP" +
                                             " IP instead (%s)"
                                             % self._server_2_ping)
                else:
                    self._server_2_ping = self._server_ip

            # ping phone1 from phone2
            packet_loss = self._networking_api2.ping(self._server_2_ping,
                                                     self._packet_size,
                                                     self._nb_pings)

            msg = "[%s] Measured Packet Loss: %.0f%s (Target: %.0f%s) - " \
                % (security,
                  packet_loss.value,
                  packet_loss.units,
                  self._target_ping_packet_loss_rate,
                  packet_loss.units)
            self._logger.info(msg)
            self._error.Msg += msg

            # Compute verdict depending on % of packet loss
            if packet_loss.value > self._target_ping_packet_loss_rate:
                msg = "Ping packet loss is not acceptable [%s]" \
                    % str(packet_loss.value)
                self._logger.error(msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

            if self._waiting_time > 0:
                self._logger.info("Waiting for %s sec" % str(self._waiting_time))
                time.sleep(self._waiting_time)

        return self._error.Code, self._error.Msg

    def tear_down(self):
        """
        End and dispose the test
        """
        LiveWifiTetheringBase.tear_down(self)

        if self._networking_api2 is not None:
            # disconnect from hotspot for Phone2
            time.sleep(self._wait_btwn_cmd)
            self._networking_api2.wifi_remove_config("all")

            # set wifi power off for phone2
            time.sleep(self._wait_btwn_cmd)
            self._networking_api2.set_wifi_power("off")

        # disable Portable wifi hotspot for Phone1
        time.sleep(self._wait_btwn_cmd)
        self._networking_api.set_wifi_hotspot("off")


        return Global.SUCCESS, "No error"
