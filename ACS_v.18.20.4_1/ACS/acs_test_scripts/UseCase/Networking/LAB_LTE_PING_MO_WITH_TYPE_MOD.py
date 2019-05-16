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
:summary:  This file implements usecase that do a MO PING over LTE network
:since: 09/09/2013
:author: razzix
.. note:: PING MO over LTE network with modification of IP version
"""


import time
from LAB_LTE_BASE import LabLteBase
from UtilitiesFWK.Utilities import Global, str_to_bool
import acs_test_scripts.Utilities.NetworkingUtilities as NetworkingUtil
from acs_test_scripts.Utilities.PhoneOnOffUtilities import PhoneOnOff


class LabLtePingMoWithTypeMod(LabLteBase):

    """
    Lab LTE mobile originated ping with modification of IP version.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call LAB_LTE_BASE Init function
        LabLteBase.__init__(self, tc_name, global_config)

        # Read mode from test case xml file
        self._switch_mode = self._tc_parameters.get_param_value("SWITCH_MODE")
        # Read the IP version from test case xml file
        self._ip_version = self._tc_parameters.get_param_value("IP_VERSION", "IPV4")

        # Read the IP version from test case xml file
        self._retry_different_ip_version = str_to_bool(
            self._tc_parameters.get_param_value("RETRY_ALTERNATIVE_VERSION", "False"))

        if self._retry_different_ip_version:
            if self._ip_version == "IPV4":
                self._alternate_ip_version = "IPV6"
                self._logger.info("The alternative IP version is V6 ")
            elif self._ip_version == "IPV6":
                self._alternate_ip_version = "IPV4"
                self._logger.info("the alternative IP version is V4")
            else:
                self._logger.error("Alternate only if IP version is V4 or V6 and not in DUAL MODE")

        # Read the number of pings to do
        self._nb_pings = self._tc_parameters.get_param_value("PACKET_COUNT")

        # Read the data size of a packet
        self._packet_size = self._tc_parameters.get_param_value("PACKET_SIZE")

        # Get target % of received packet for ping
        self._target_ping_packet_loss_rate = \
            float(self._tc_parameters.get_param_value("TARGET_PACKET_LOSS_RATE"))

        # Read the network registration timeout
        self._network_reg_timeout = self._tc_parameters.get_param_value("NETWORK_REG_TIMEOUT", 0, int)
        if self._network_reg_timeout == 0:
            self._network_reg_timeout = self._registration_timeout

        # Instantiate Phone On/OFF utilities
        self.phoneonoff_util = PhoneOnOff(self._networking_api, self._device, self._logger)

        # Instantiate the IP version
        self._nw_ip_version = ""

#------------------------------------------------------------------------------
    def set_up(self):
        """
        Initialize the test
        """
        # Depending on IP version, set DUT IP address with IPV4 or IPV6 address
        if self._ip_version == "IPV6":
            if self._ns_dut_ipv6_Address not in (None, ""):
                self._ns_dut_ip_Address = self._ns_dut_ipv6_Address
                self._logger.info("IP version is %s, set DUT IP to: %s" % (self._ip_version, self._ns_dut_ip_Address))
            else:
                self._logger.error("IPV6 address for DUT is not defined in Callbox section of bench config!! Please update it")
                return Global.FAILURE, "IPV6 address for DUT is not defined in Callbox section of bench config!! Please update it"
        else:
            self._logger.info("IP version is %s, keep DUT IP to: %s" % (self._ip_version, self._ns_dut_ip_Address))
        # Call LAB_LTE_BASE set_up function
        LabLteBase.set_up(self)

        self._check_ip_version()

        # Set Cell on
        self._ns_cell_4g.set_cell_on(self._mimo)

        # Flight mode deactivation after LTE BASE Setup
        self._networking_api.set_flight_mode("off")

        if self._switch_mode in ("hardshutdown", "softshutdown"):
            # Switch off according to the mode chosen in XML file
            self.phoneonoff_util.switch_off(self._switch_mode)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------
    def run_test(self):
        """
        Execute the test
        """
        LabLteBase.run_test(self)

        if self._switch_mode == "airplane":
            # Switch off according to the mode chosen in XML file
            self.phoneonoff_util.switch_off(self._switch_mode)

            # check phone is unregistered
            self._modem_api.check_cdk_no_registration_bfor_timeout(self._registration_timeout)
            self._networking_api.check_no_ip_address()

            self._check_ip_version()

        # Switch on according to the mode chosen
        self.phoneonoff_util.switch_on(self._switch_mode)

        # Check registration state is connected using
        # registrationTimeout from Device_Catalog.xml or timeout parameter from TC xml file
        self._logger.info("Check network registration status is %s on DUT, before %d seconds" %
                          (self._wanted_reg_state, self._network_reg_timeout))

        # Check registration on HPLMN
        self._modem_api.check_cdk_state_bfor_timeout(
            self._wanted_reg_state,
            self._network_reg_timeout)

        # Set APN for LTE and/or IMS depending on protocol IPv4 or IPv6
        self._set_apn_for_lte_and_ims()

        # PDP activation
        time.sleep(self._wait_btwn_cmd)
        self._networking_api.activate_pdp_context(check=False)

        # Check data connection state is "CON"
        self._check_data_connection_state("CON", self._network_reg_timeout)

        # Get RAT from Equipment
        network_type = self._ns_data_4g.get_network_type()
        # Check that DUT is registered on the good RAT
        self._modem_api.check_network_type_before_timeout(network_type,
                                                          self._registration_timeout)

        (self._error.Code, self._error.Msg) = self._start_ping(self._nw_ip_version)

        if self._error.Code == Global.SUCCESS and self._retry_different_ip_version:

            if self._nw_ip_version in ("IPV6", "IPV4"):
                self._logger.info("RETRY with %s version " % self._alternate_ip_version)
                # Switch off according to the mode chosen in XML file
                self.phoneonoff_util.switch_off(self._switch_mode)
                # check phone is unregistered
                self._modem_api.check_cdk_no_registration_bfor_timeout(self._registration_timeout)
                # stop scenario in order to change version
                self._ns.stop_scenario()
                # Set the new IP version assigned by the NW
                self._ns_data_4g.set_ip_address_type(self._alternate_ip_version)
                # Start scenario
                self._ns.start_scenario()
                # Switch off according to the mode chosen in XML file
                self.phoneonoff_util.switch_on(self._switch_mode)
                # Check data connection state is "CON"
                self._check_data_connection_state("CON", self._network_reg_timeout)
                # Check registration state is connected using
                # registrationTimeout from Device_Catalog.xml
                self._logger.info("Check network registration status is %s on DUT, before %d seconds" %
                                  (self._wanted_reg_state, self._network_reg_timeout))
                # Check registration on HPLMN
                self._modem_api.check_cdk_state_bfor_timeout(
                    self._wanted_reg_state,
                    self._network_reg_timeout)
                # Get RAT from Equipment
                network_type = self._ns_data_4g.get_network_type()
                # Check that DUT is registered on the good RAT
                self._modem_api.check_network_type_before_timeout(network_type,
                                                                  self._registration_timeout)
                (self._Code, self._Msg) = self._start_ping(self._alternate_ip_version)

                if self._Code == Global.SUCCESS:
                    # ping with both version has succeeded
                    self._error.Code = Global.SUCCESS
                    self._error.Msg = "TEST IS PASS ALL EXPECTED RESULTS ARE MET"
                    return self._error.Code, self._error.Msg

                else:
                    # ping with alternative version has failed
                    self._error.Code = Global.FAILURE
                    self._error.Msg += " and Ping with Alternative %s has failed"
                    return self._error.Code, self._error.Msg

            else:
                # ALternate between IPV4 & IPV6 only
                self._error.Code = Global.FAILURE
                self._error.Msg = "IP Version is set to DUAL MODE"
                return self._error.Code, self._error.Msg

        else:
            # No Need to retry
            return self._error.Code, self._error.Msg

#--------------------------------------------------------------------------------------
    def _start_ping(self, nw_version):
        """
        Start ping depending on version chosen

        :type nw_version: str
        :param nw_version: IP version that Network Simulator assigned

        :rtype: int
        :return: 0 if PASS, -1 if Failed, -2 if Blocked

        :rtype: str
        :return: Error Message in case of failure
        """

        if nw_version == "IPV6":
            # Check scope link IPV6 address is well allocated
            (self._error.Code, self._error.Msg) = self._check_ipv6_address()
            if self._error.Code == Global.FAILURE:
                return self._error.Code, self._error.Msg
            # Ping with IPV6 only address
            (self._error.Code, self._error.Msg) = self._pingv6only()
            return self._error.Code, self._error.Msg

        elif nw_version == "IPV4":
            # Ping with IPV4 only address
            (self._error.Code, self._error.Msg) = self._pingv4only()
            return self._error.Code, self._error.Msg

        elif nw_version == "IPV4V6":
            # Check scope link IPV6 address is well allocated
            (self._error.Code, self._error.Msg) = self._check_ipv6_address()
            if self._error.Code == Global.FAILURE:
                return self._error.Code, self._error.Msg

            # Ping with IPV4 & IPV6 addresses
            (self._error.Code, self._error.Msg) = self._pingv4v6()
            return self._error.Code, self._error.Msg

        else:
            # The value to set for IP version is invalid
            self._error.Code = Global.FAILURE
            self._error.Msg = "NW IP version needs to be IPV4, IPV4 or IPV4V6"
            return self._error.Code, self._error.Msg

#--------------------------------------------------------------------------------------
    def _check_ipv6_address(self):
        """
        Set and check scope link IPV6 address is well allocated

        :rtype: int
        :return: Global.FAILURE (-1) if Failed

        :rtype: str
        :return: Error Message in case of failure
        """
        # Check scope link IPV6 address is well allocated
        ip_v6_local = self._networking_api.get_interface_ipv6_scopelink_address(self._device.get_cellular_network_interface())
        # check if IPV6 allocated to the device is a valid IP address
        if NetworkingUtil.is_valid_ipv6_address(ip_v6_local):
            # check if IPV6 allocated to the device is a valid IP address
            self._logger.info("The local IPv6 address assigned to the DUT is %s " % ip_v6_local)
            Code = Global.SUCCESS
            Msg = "The local IPv6 address assigned to the DUT is %s " % ip_v6_local
        else:
            Code = Global.FAILURE
            Msg = "DUT does not have a local IPv6 assigned"
            self._logger.info(Msg)
        return Code, Msg

#--------------------------------------------------------------------------------------
    def _pingv6only(self):
        """
        ping with IPV6 address v6 and also verify that ping IPv4 fails

        :rtype: int
        :return: 0 if PASS, -1 if Failed, -2 if Blocked

        :rtype: str
        :return: Error Message in case of failure
        """
        # Ping IPV4
        packet_lossv4 = self._networking_api.\
            ping(self._server_ip_address,
                 self._packet_size,
                 self._nb_pings,
                 blocking=False)
        # Check if IP V4 Fails
        if packet_lossv4.value == -1:
            # Check if DUT have an address IP V4 allocated
            self.ip_v4 = self._ns_data_4g.get_ip_address("IPV4", blocking=False)

            if NetworkingUtil.is_valid_ipv4_address(str(self.ip_v4)):
                self._error.Code = Global.FAILURE
                self._error.Msg = "DUT have a valid IPV4 address when NW is set to IPV6 only"
                self._logger.info(self._error.Msg)
                return self._error.Code, self._error.Msg

            else:
                msg = "PING IPV4: FAILED (as expected) because DUT does not have an IPV4 address allocated"
                self._logger.info(msg)
                self._error.Msg = msg
        else:
            self._error.Code = Global.FAILURE
            self._error.Msg = "ERROR :PING with IPV4 SHOULD FAIL, Measured IPV4 Packet Loss: %.0f%s (Target: %.0f%s)"\
                % (packet_lossv4.value,
                   packet_lossv4.units,
                   self._target_ping_packet_loss_rate,
                   packet_lossv4.units)
            self._logger.error(self._error.Msg)
            return self._error.Code, self._error.Msg
        # Ping IPV6
        packet_lossv6 = self._networking_api.\
            ping6(self._server_ip_v6_address,
                  self._packet_size,
                  self._nb_pings)
        # Compute verdict depending on % of packet loss
        if packet_lossv6.value > self._target_ping_packet_loss_rate:
            self._error.Code = Global.FAILURE
        else:
            self._error.Code = Global.SUCCESS

        msg = "Ping IPV6: SUCCEEDED with Measured Packet Loss: %.0f%s (Target: %.0f%s)"\
            % (packet_lossv6.value,
               packet_lossv6.units,
               self._target_ping_packet_loss_rate,
               packet_lossv6.units)
        self._logger.info(msg)
        self._error.Msg += " & " + msg

        return self._error.Code, self._error.Msg

#--------------------------------------------------------------------------------------
    def _pingv4only(self):
        # Ping IPV6
        packet_lossv6 = self._networking_api.\
            ping6(self._server_ip_v6_address,
                  self._packet_size,
                  self._nb_pings,
                  blocking=False)

        # Check if IP V6 Fails
        if packet_lossv6.value == -1:
            # Check if DUT have an address IP V6 allocated
            self.ip_v6 = self._ns_data_4g.get_ip_address("IPV6", blocking=False)

            if NetworkingUtil.is_valid_ipv6_address(str(self.ip_v6)):
                self._error.Code = Global.FAILURE
                self._error.Msg = "DUT have a valid IPV6 address when NW is set to IPV4 only"
                self._logger.info(self._error.Msg)
                return self._error.Code, self._error.Msg

            else:
                msg = "PING IPV6: FAILED (as expected) because DUT does not have an IPV6 address allocated"
                self._logger.info(msg)
                self._error.Msg = msg
        else:
            self._error.Code = Global.FAILURE
            self._error.Msg = "ERROR :PING with IPV6 SHOULD FAIL, Measured IPV6 Packet Loss: %.0f%s (Target: %.0f%s)"\
                % (packet_lossv6.value,
                   packet_lossv6.units,
                   self._target_ping_packet_loss_rate,
                   packet_lossv6.units)
            self._logger.error(self._error.Msg)

        # Ping IPV4
        packet_lossv4 = self._networking_api.\
            ping(self._server_ip_address,
                 self._packet_size,
                 self._nb_pings)

        # Compute verdict depending on % of packet loss
        if packet_lossv4.value > self._target_ping_packet_loss_rate:
            self._error.Code = Global.FAILURE
        else:
            self._error.Code = Global.SUCCESS

        msg = "Ping IPV4: SUCCEEDED with Measured Packet Loss: %.0f%s (Target: %.0f%s)"\
            % (packet_lossv4.value,
               packet_lossv4.units,
               self._target_ping_packet_loss_rate,
               packet_lossv4.units)

        self._logger.info(msg)

        self._error.Msg += " & " + msg

        return self._error.Code, self._error.Msg

#--------------------------------------------------------------------------------------
    def _pingv4v6(self):
        # Ping IPV4
        packet_lossv4 = self._networking_api.\
            ping(self._server_ip_address,
                 self._packet_size,
                 self._nb_pings)
        if packet_lossv4.value > self._target_ping_packet_loss_rate:
            self._error.Code = Global.FAILURE
            self._error.Msg = "Ping with IPV4 address failed with Measured Packet Loss: %.0f%s (Target: %.0f%s)"\
                % (packet_lossv4.value,
                   packet_lossv4.units,
                   self._target_ping_packet_loss_rate,
                   packet_lossv4.units)
            return self._error.Code, self._error.Msg
        else:
            self._error.Code = Global.SUCCESS
            msg_v4 = "Ping IPV4: SUCCEEDED with Measured Packet Loss: %.0f%s (Target: %.0f%s)"\
                % (packet_lossv4.value,
                   packet_lossv4.units,
                   self._target_ping_packet_loss_rate,
                   packet_lossv4.units)
            self._logger.info(msg_v4)
        # Ping IPV6
        packet_lossv6 = self._networking_api.\
            ping6(self._server_ip_v6_address,
                  self._packet_size,
                  self._nb_pings)
        if packet_lossv6.value > self._target_ping_packet_loss_rate:
            self._error.Code = Global.FAILURE
            self._error.Msg = "Ping with IPV6 address failed with Measured Packet Loss: %.0f%s (Target: %.0f%s)"\
                % (packet_lossv6.value,
                   packet_lossv6.units,
                   self._target_ping_packet_loss_rate,
                   packet_lossv6.units)
            return self._error.Code, self._error.Msg
        else:
            self._error.Code = Global.SUCCESS
            msg_v6 = "Ping IPV6: SUCCEEDED with Measured Packet Loss: %.0f%s (Target: %.0f%s)"\
                % (packet_lossv6.value,
                   packet_lossv6.units,
                   self._target_ping_packet_loss_rate,
                   packet_lossv6.units)
            self._logger.info(msg_v6)

        self._error.Msg = msg_v4 + " & " + msg_v6

        return self._error.Code, self._error.Msg
