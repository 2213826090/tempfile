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
:summary: This file implements the LIVE WIFI CONNECT UC
:since: 22/08/2011
:author: szhen11
"""

from acs_test_scripts.Device.UECmd.UECmdTypes import AUTO_CONNECT_STATE
from UtilitiesFWK.Utilities import Global
from UtilitiesFWK.Utilities import str_to_bool
from acs_test_scripts.UseCase.Networking.LAB_WIFI_BASE import LabWifiBase
import time
import re
import numpy
from string import hexdigits  # pylint: disable=W0402
from random import choice as ramdom_choice
from acs_test_scripts.Utilities.NetworkingUtilities import compute_verdict
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsBaseException import AcsBaseException


class LabWifiConnect(LabWifiBase):

    """
    Lab Wifi Connect Test class.
    """
    DEFAULT_TOLERANCE = .25

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        LabWifiBase.__init__(self, tc_name, global_config)

        # Retrieve UC parameter
        self._restart_dut = str(self._tc_parameters.
                                get_param_value("RESTART_DUT", "")).upper()
        if self._restart_dut in ["", "OFF", "NO", "NONE", "FALSE"]:
            self._restart_dut = "OFF"
        self._restart_ap_radio = str_to_bool(str(self._tc_parameters.
                                                 get_param_value("RESTART_AP_RADIO")))
        self._mac_filter = str(self._tc_parameters.
                               get_param_value("MAC_FILTER", "OFF")).upper()
        if self._mac_filter == "":
            self._mac_filter = "OFF"
        self._mac_filter_parameter = str(self._tc_parameters.get_param_value("MAC_FILTER_PARAMETER", "OFF"))
        if self._mac_filter_parameter == "":
            self._mac_filter_parameter = "OFF"
        self._dut_in_mac_filter = False

        self._connection_time_list = list()
        self._current_iteration_num = 0

        self._expected_connection_time = str(self._tc_parameters.get_param_value("REF_CONNECTION_TIME"))

        self._tolerance = str(self._tc_parameters.get_param_value("TOLERANCE"))

        if re.match(r'\d+\.?\d*', self._tolerance) is not None:
            self._tolerance = float(self._tolerance) / 100
        else:
            self._tolerance = self.DEFAULT_TOLERANCE
            self._logger.warning("Tolerance not found, set to default value")
        self._logger.debug("Tolerance set to %2.2f%%" % (self._tolerance * 100))

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        LabWifiBase.set_up(self)

        # Control RESTART_DUT TC parameter
        if self._restart_dut not in ["OFF", "PHONE", "INTERFACE"]:
            msg = "Wrong value for RESTART_DUT: %s" % self._restart_dut
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        # MAC address filter management
        if self._mac_filter != "OFF":
            # Retrieve DUT MAC address
            dut_mac = self._networking_api.get_interface_mac_addr()

            if self._mac_filter == "DUT":
                self._mac_filter = dut_mac
            elif self._mac_filter == "!DUT":
                # Create a MAC address different from DUT one
                last_char = dut_mac[-1:]
                new_char = last_char
                while new_char == last_char:
                    new_char = ramdom_choice(hexdigits)
                self._mac_filter = dut_mac[:-1] + new_char

            if self._mac_filter == dut_mac:
                self._dut_in_mac_filter = True

            if self._mac_filter_parameter not in ["OFF", "DISCONNECTED", "DISCONNECTED_WAIT_DISABLE"]:
                msg = "Wrong value for MAC_FILTER_PARAMETER: %s" % self._mac_filter_parameter
                self._logger.error(msg)
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        if str(self._expected_connection_time).upper() in ["NONE", "", "0", "NO"]:
            self._monitor_connection_time = False
        elif str(self._expected_connection_time).isdigit() and int(self._expected_connection_time) > 0:
            self._monitor_connection_time = True
            self._expected_connection_time = int(self._expected_connection_time)
        else:
            msg = "Wrong parameter for REF_CONNECTION_TIME: read value %s" % str(self._expected_connection_time)
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        # Update back to back iteration and the wifi connection time list
        if self._monitor_connection_time:
            msg = "DUT connected in %d seconds" % self._connection_time
            self._logger.info(msg)
            self._current_iteration_num += 1

            if self._connection_time != -1:
                self._connection_time_list.append(self._connection_time)
            else:
                self._logger.warning("unable to measure connection time")

        return Global.SUCCESS, "no_error"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        # pylint: disable=E1101
        LabWifiBase.run_test(self)
        # monitor wifi connection time if requested
        mean = -1
        if self._monitor_connection_time and self.get_b2b_iteration() == self._current_iteration_num:
            # This is the last iteration of back to back test
            # compute standard deviation, mean and verdict
            mean = float(numpy.mean(self._connection_time_list))
            std_deviation = float(numpy.std(self._connection_time_list))
            compute_verdict(self._expected_connection_time, self._tolerance, mean, std_deviation, self._logger)

        # Switch OFF the AP if the TC associated parameter is enabled
        if self._restart_ap_radio:
            # Forget the SSID
            self._networking_api.wifi_remove_config(self._ssid)

            # Disable radio
            self._ns.init()
            self._ns.disable_wireless()

            # Restart the DUT wifi interface
            self._networking_api.set_wifi_power("off")
            time.sleep(self._wait_btwn_cmd)
            self._networking_api.set_wifi_power("on")
            time.sleep(self._wait_btwn_cmd)

            # Register to the "out of range" network
            if str(self._wrong_passphrase).lower() == 'none':
                self._networking_api.set_wificonfiguration(self._ssid,
                                                           self._passphrase,
                                                           self._security,
                                                           self._ip_setting,
                                                           self._ip_address,
                                                           self._netmask,
                                                           self._gateway,
                                                           self._dns1,
                                                           self._dns2)
            else:
                self._networking_api.set_wificonfiguration(self._ssid,
                                                           self._wrong_passphrase,
                                                           self._security,
                                                           self._ip_setting,
                                                           self._ip_address,
                                                           self._netmask,
                                                           self._gateway,
                                                           self._dns1,
                                                           self._dns2)
            self._networking_api.set_autoconnect_mode(self._ssid,
                                                      AUTO_CONNECT_STATE.on)

        # Restart the Wifi interface if required
        if self._restart_dut == "INTERFACE":
            # Power cycle the Wifi interface
            self._networking_api.set_wifi_power("off")
            time.sleep(self._wait_btwn_cmd)
            self._networking_api.set_wifi_power("on")
            time.sleep(self._wait_btwn_cmd)
        elif self._restart_dut == "PHONE":
            # Power cycle the phone
            self._device.reboot()

        # Switch ON the AP if the TC associated parameter is enabled
        if self._restart_ap_radio:
            # enable_wireless includes a time sleep
            self._ns.enable_wireless()
            self._ns.release()

        if self._restart_dut == "INTERFACE" \
                or self._restart_dut == "PHONE" \
                or self._restart_ap_radio \
                or self._mac_filter != "OFF" \
                or str(self._wrong_passphrase).lower() != 'none':
            if str(self._wrong_passphrase).lower() == 'none' and \
                    not self._key_exchange_should_fail:
                # Wait for the connection to establish
                self._networking_api.check_connection_state(self._ssid)
            else:
                # Wait for the connection to try to establish
                self._logger.debug("Waiting for the connection list to be updated")
                self._phone_system_api.display_on()
                time.sleep(30)
                self._phone_system_api.display_off()

        # MAC address filter management
        if self._mac_filter != "OFF":
            if self._mac_filter_parameter != "OFF":
                self._networking_api.wifi_disconnect(self._ssid)
            self._add_mac_filter_to_ap()
            # Try to reconnect DUT
            self._networking_api.wifi_connect(self._ssid, False)
            try:
                self._networking_api.check_connection_state(self._ssid, 20)
            except AcsBaseException as e:
                if not self._dut_in_mac_filter or \
                        e.get_generic_error_message() != DeviceException.TIMEOUT_REACHED:
                    raise

        # List connected SSIDs to check if the right ssid is connected
        connected_wifi_list = self._networking_api.list_connected_wifi()

        if str(self._wrong_passphrase).lower() == 'none' and \
                not self._key_exchange_should_fail and \
                not self._dut_in_mac_filter:
            if self._ssid in connected_wifi_list:
                result = "Wifi connected to network %s with correct password." % str(self._ssid)
                if mean > 0:
                    result += " Mean connection time is %2.2f sec" % mean

                if self._ip_setting_enable:
                    if (self._ip_address ==
                            self._networking_api.get_wifi_ip_address()):
                        result += "Static ip successfully set."
                    else:
                        msg = "obtained ip address is different from the " + \
                            "static ip address"
                        self._logger.error(msg)
                        raise DeviceException(DeviceException.OPERATION_FAILED, msg)
            else:
                msg = "Wifi did not connect to network " \
                    + "[%s] with correct password. " % str(self._ssid)
                self._logger.error(msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)
        else:
            if self._ssid not in connected_wifi_list:
                result = "Wifi not connected to network " \
                    + "%s" % str(self._ssid)
                if str(self._wrong_passphrase).lower() != 'none' or \
                        self._key_exchange_should_fail:
                    result += ", with wrong password"
                if self._dut_in_mac_filter:
                    result += ", MAC add filtered"
            else:
                msg = "Wifi connected to network " \
                    + "[%s]" % str(self._ssid)
                if str(self._wrong_passphrase).lower() != 'none' or \
                        self._key_exchange_should_fail:
                    msg += ", with wrong password"
                if self._dut_in_mac_filter:
                    msg += ", MAC add filtered"
                self._logger.error(msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        if self._mac_filter != "OFF":
            if self._mac_filter_parameter != "OFF":
                self._networking_api.wifi_disconnect(self._ssid)
            # Remove MAC filter
            self._remove_mac_filter_on_ap()
            # Connect DUT
            self._networking_api.wifi_connect(self._ssid)

        return Global.SUCCESS, result

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """
        try:
            LabWifiBase.tear_down(self)
        finally:
            # Always remove MAC address filter
            if self._mac_filter != "OFF":
                self._remove_mac_filter_on_ap()

        return Global.SUCCESS, "no_error"

#------------------------------------------------------------------------------

    def _add_mac_filter_to_ap(self):
        """
        Add MAC address to the AP MAC filter
        """
        self._ns.init()
        self._ns.add_mac_address_to_acl(self._mac_filter)
        self._ns.set_acl_mode("enable")
        self._ns.release()

    def _remove_mac_filter_on_ap(self):
        """
        Remove MAC address on the AP MAC filter
        """
        self._ns.init()
        self._ns.set_acl_mode("disable")
        self._ns.del_mac_address_from_acl(self._mac_filter)
        self._ns.release()
