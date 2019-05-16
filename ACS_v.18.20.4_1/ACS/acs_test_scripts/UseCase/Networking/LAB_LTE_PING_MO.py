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
:summary:  This file implements usecase that do a PING over LTE network
:since: 23/04/2012
:author: Lvacheyx
.. note:: BZ3071 - PING MO over LTE network
"""

import time
from LAB_LTE_BASE import LabLteBase
from UtilitiesFWK.Utilities import Global


class LabLtePingMo(LabLteBase):

    """
    Lab LTE mobile originated ping.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call LAB_LTE_BASE Init function
        LabLteBase.__init__(self, tc_name, global_config)

        # Read the number of pings to do
        self._nb_pings = self._tc_parameters.get_param_value("PACKET_COUNT")

        # Read the data size of a packet
        self._packet_size = self._tc_parameters.get_param_value("PACKET_SIZE")

        # Get target % of received packet for ping
        self._target_ping_packet_loss_rate = \
            float(self._tc_parameters.get_param_value("TARGET_PACKET_LOSS_RATE"))

        self._rrc_state = self._tc_parameters.get_param_value("RRC_STATE", "RRC_CONNECTED")

#------------------------------------------------------------------------------
    def set_up(self):
        """
        Initialize the test
        """
        # Call LAB_LTE_BASE set_up function
        LabLteBase.set_up(self)

        # Set Cell on
        self._ns_cell_4g.set_cell_on(self._mimo)

        # Flight mode deactivation
        self._networking_api.set_flight_mode("off")


        # Check data connection state is "CON"
        self._check_data_connection_state("CON")
        # Check registration state is connected using
        # registrationTimeout from Device_Catalog.xml
        self._logger.info("Check network registration status is %s on DUT" %
                          self._wanted_reg_state)

        self._modem_api.check_cdk_state_bfor_timeout(
            self._wanted_reg_state,
            self._registration_timeout)

        # Set APN for LTE and/or IMS depending on protocol IPv4 or IPv6
        self._set_apn_for_lte_and_ims()

        # move to her to wait for register then activate pdp
        time.sleep(self._wait_btwn_cmd)
        self._networking_api.activate_pdp_context(check = False)

        # Get RAT from Equipment
        network_type = self._ns_data_4g.get_network_type()
        # Check that DUT is registered on the good RAT
        self._modem_api.check_network_type_before_timeout(network_type,
                                                          self._registration_timeout)

        # Set rrc state to the value defined in the TC
        current_rrc_state = self._ns_cell_4g.get_rrc_state()
        if self._rrc_state == "RRC_IDLE":
            self._networking_api.disable_output_traffic()
            self._ns_data_4g.ue_detach()
            self._networking_api.enable_output_traffic()
            current_rrc_state = self._ns_cell_4g.get_rrc_state()
            self._logger.info("The ping will start from %s state" % current_rrc_state)
        elif self._rrc_state == "RRC_CONNECTED":
            self._logger.info("The ping will start from %s state" % current_rrc_state)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        LabLteBase.run_test(self)

        # Compute packet loss value
        packet_loss = self._networking_api.\
            ping(self._server_ip_address,
                 self._packet_size,
                 self._nb_pings)

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

        self._logger.info(self._error.Msg)

        return self._error.Code, self._error.Msg
