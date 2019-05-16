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
:summary:  This file implements usecase that do registration and PING over LTE network
:since: 03/04/2013
:author: lvacheyx
"""

import time

from LAB_LTE_BASE import LabLteBase
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.Utilities.PhoneOnOffUtilities import PhoneOnOff


class LabLteReg(LabLteBase):

    """
    Lab LTE registration and mobile originated ping.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call LAB_LTE_BASE Init function
        LabLteBase.__init__(self, tc_name, global_config)

        # Read mode from test case xml file (str)
        self._switch_mode = self._tc_parameters.get_param_value("SWITCH_MODE", "hardshutdown")

        # Read the number of pings to do
        self._nb_pings = self._tc_parameters.get_param_value("PACKET_COUNT")

        # Read the data size of a packet
        self._packet_size = self._tc_parameters.get_param_value("PACKET_SIZE")

        # Get target % of received packet for ping
        self._target_ping_packet_loss_rate = \
            float(self._tc_parameters.get_param_value("TARGET_PACKET_LOSS_RATE"))

        self._camp_timeout = self._registration_timeout

        # Instantiate Phone On/OFF utilities
        self.phoneonoff_util = PhoneOnOff(self._networking_api, self._device, self._logger)

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        # Call LAB_LTE_BASE set_up function
        LabLteBase.set_up(self)

        # Set Cell on
        self._ns_cell_4g.set_cell_on(self._mimo)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        # Call LAB_LTE_BASE run_test function
        LabLteBase.run_test(self)

        if self._switch_mode == "airplane":
            # Switch on according to the mode chosen
            self.phoneonoff_util.switch_on(self._switch_mode)
        elif self._switch_mode in ("hardshutdown", "softshutdown"):
            # Reboot according to the mode chosen
            self.phoneonoff_util.reboot(self._switch_mode)
             # Turn Off Airplane mode - Phone might still be ON due to previous test execution
            self._networking_api.set_flight_mode("off")
        else:
            self._logger.info("No actions required to do a switch On/Off")

        # Check registration state is connected using
        # registrationTimeout from Device_Catalog.xml
        self._logger.info("Check network registration status is %s on DUT" %
                          self._wanted_reg_state)

        self._modem_api.check_cdk_state_bfor_timeout(
            self._wanted_reg_state,
            self._camp_timeout)

        # Set APN for LTE and/or IMS depending on protocol IPv4 or IPv6
        self._set_apn_for_lte_and_ims()

        # Activate PDP context
        time.sleep(self._wait_btwn_cmd)
        self._logger.info("Active PDP Context...")
        self._networking_api.activate_pdp_context(self._ssid, False)

        # Check data connection state is "CON"
        self._ns_data_4g.check_data_connection_state("CON",
                                                  self._camp_timeout,
                                                  blocking=False,
                                                  cell_id=self._cell_id)

        # Get RAT from Equipment
        network_type = self._ns_data_4g.get_network_type()

        # Check that DUT is registered on the good RAT
        self._modem_api.check_network_type_before_timeout(network_type,
                                                          self._camp_timeout)

        # Wait between two commands sending
        time.sleep(self._wait_btwn_cmd)
        try:
            # Compute packet loss value
            packet_loss = self._networking_api.\
                ping(self._server_ip_address,
                     self._packet_size,
                     self._nb_pings,
                     source_address=self._ns_dut_ip_Address)

            # Compute verdict depending on % of packet loss
            if packet_loss.value > self._target_ping_packet_loss_rate:
                self._error.Code = Global.FAILURE
            else:
                self._error.Code = Global.SUCCESS

            self._error.Msg = "Measured Packet Loss: %.0f%s (Target: %.0f%s)"\
                % (packet_loss.value,
                   packet_loss.units,
                   self._target_ping_packet_loss_rate,
                   packet_loss.units)
        except:
            self._logger.error("PING FAILURE!!!!")
            self._error.Code = Global.FAILURE
            self._error.Msg = "PING FAILURE!!!!"

        if self._switch_mode == "airplane":
            # Switch off according to the switch mode chosen
            self.phoneonoff_util.switch_off(self._switch_mode)
            # Check data connection is IDLE
            self._check_data_connection_state("IDLE")
            # Check that DUT is no longer camped on Network
            self._modem_api.check_cdk_no_registration_bfor_timeout(self._camp_timeout)

        if self._switch_tdd_fdd:
            # Switch configuration between TDD and FDD
            if self._duplex_type == "FDD":
                self._duplex_type = "TDD"
            elif self._duplex_type == "TDD":
                self._duplex_type = "FDD"

        return self._error.Code, self._error.Msg
