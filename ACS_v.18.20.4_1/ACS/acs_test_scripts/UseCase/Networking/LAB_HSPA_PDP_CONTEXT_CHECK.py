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
:summary: This file implements HSPA PDP activation and PDP deactivation
:author: lvacheyx
:since:05/03/2013
"""
import time
from LAB_HSPA_BASE import LabHspaBase
from UtilitiesFWK.Utilities import Global, str_to_bool
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsBaseException import AcsBaseException
from ErrorHandling.AcsConfigException import AcsConfigException


class LabHspaPdpContextCheck(LabHspaBase):

    """
    Lab HSPA PDP activation and deactivation
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Set the default number of pings to do
        self._nb_pings = 10

        # Set the default data size of a packet
        self._packet_size = 32

        # Call LAB_HSPA_BASE Init function
        LabHspaBase.__init__(self, tc_name, global_config)

        # Getting the parameter WITH_PING from the testcase.
        # Default value is FALSE.
        self._do_ping = str(self._tc_parameters.get_param_value("WITH_PING",
                                                                "FALSE"))
        # Getting the TARGET_PACKET_LOSS_RATE from the testcase.
        self._target_ping_packet_loss_rate = \
            self._tc_parameters.get_param_value("TARGET_PACKET_LOSS_RATE")

        # Getting the RRC_STATE parameter from testcase. Default value
        # is NONE.
        self._rrc_state = str(self._tc_parameters.get_param_value("RRC_STATE", "NONE"))

        self._msg = ""

#------------------------------------------------------------------------------

    def set_up(self):
        # Calling the LabHspase set_up function.
        (code, msg) = LabHspaBase.set_up(self)
        if code == Global.FAILURE:
            # If the base setup fails return its fail, and exit.
            return code, msg
        # Check if ping is needed for the test.
        if self._do_ping.upper() in ("TRUE", "FALSE"):
            self._do_ping = str_to_bool(self._do_ping)
        else:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "The DO_PING parameter should be TRUE or "
                                     "FALSE not %s" % self._do_ping)
        if self._do_ping:
            self._logger.info("The test will use ping to test the PDP during"
                              " this test.")
            # If yes check the target ping packet lost as a correct value.
            if isinstance(self._target_ping_packet_loss_rate, (int, float)):
                self._target_ping_packet_loss_rate = \
                    float(self._target_ping_packet_loss_rate)
            else:
                # if user doesn't enter the correct value, set the default
                # value 0
                self._target_ping_packet_loss_rate = 0
        else:
            self._logger.info("The test won't use ping to test the PDP during"
                              " this test.")
        # Checking the value of the RRC_STATUS parameter is in the expected
        # range.
        if not self._rrc_state in ("NONE", "IDLE", "DCH", "FACH"):
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "The RRC_STATUS parameter should be "
                                     "NONE, IDLE, DCH, FACH not %s"
                                     % self._rrc_state)
        return Global.SUCCESS, "No error"

    def run_test(self):
        """
        Execute the test
        """
        # Call LAB_HSPA_BASE Run function
        LabHspaBase.run_test(self)

        # Activate PDP context
        time.sleep(self._wait_btwn_cmd)
        self._logger.info("Active PDP Context...")
        self._networking_api.activate_pdp_context(self._ssid)

        # Check Data Connection State => PDP Active before timeout
        self._ns_data_3g.check_data_connection_state("PDP_ACTIVE",
                                                  self._registration_timeout,
                                                  blocking=False)

        # Get RAT from Equipment
        network_type = self._ns_data_3g.get_network_type()

        # Check that DUT is registered on the good RAT
        self._modem_api.check_network_type_before_timeout(network_type,
                                                          self._registration_timeout)

        # Check IP protocol
        self._msg = self._networking_api.check_ip_protocol(self._ip_version)
        self._logger.info(self._msg)

        if self._rrc_state == "FACH":
            # If the RRC_STATE was set to FACH, set the RRC transition to
            # FACH
            self._logger.info("Setting the rrc_state transition to FACH")
            self._ns_cell_3g.set_rrc_state_transition("FACH")
            time.sleep(5)

        if self._do_ping:
            # Ping FTP server to verify PDP context activated.
            # Ping should be successful
            packet_loss = self._networking_api.ping(
                    self._server_ip_address,
                    self._packet_size,
                    self._nb_pings)

            self._error.Msg = "Measured Packet Loss: %.0f%s (Target: %.0f%s)"\
                    % (packet_loss.value, packet_loss.units,
                       self._target_ping_packet_loss_rate,
                       packet_loss.units)

            # Compute verdict depending on % of packet loss
            # PDP Context active, ping should be successful
            if packet_loss.value > self._target_ping_packet_loss_rate:
                raise DeviceException(DeviceException.OPERATION_FAILED,
                                       self._error.Msg)
            else:
                self._logger.info("Ping successful:  %s" % self._error.Msg)

        # Deactivate PDP context
        self._logger.info("Deactivation of the PDP Context")
        self._networking_api.deactivate_pdp_context(self._ssid)

        # Check ATTACHED Data Connection State
        # We can't use check_data_connection_state function, because the UE
        # should switch from PDP_ACTIVE state to ATTACHED state, but in this function
        # "PDP_ACTIVE" means already "ATTACHED" in telephony context.
        timer = self._registration_timeout
        while (timer > 0) and (self._ns_data_3g.get_data_connection_status() != "ATTACHED"):
            timer -= 1
            time.sleep(1)

        # Check that DUT does not have any IP address
        self._networking_api.check_no_ip_address()

        if self._do_ping:
            # Ping FTP server to verify PDP context deactivated.
            # The test should be failed
            try:
                packet_loss = self._networking_api.ping(
                    self._server_ip_address,
                    self._packet_size,
                    self._nb_pings)
            except AcsBaseException as error:
                self._logger.info(error.get_error_message())
            else:
                raise DeviceException(DeviceException.OPERATION_FAILED,
                                       "PDP context isn't correctly "
                                       "deactivated, ping packets reach the"
                                       " server.")
        # Reactivate PDP context
        self._networking_api.activate_pdp_context(self._ssid)

        self._ns_data_3g.check_data_connection_state("PDP_ACTIVE",
                                                  self._registration_timeout,
                                                  blocking=False)

        if self._do_ping:
            packet_loss = self._networking_api.ping(
                self._server_ip_address,
                self._packet_size,
                self._nb_pings)
            # Compute verdict depending on % of packet loss
            # PDP Context active, ping should be successful
            if packet_loss.value > self._target_ping_packet_loss_rate:
                raise DeviceException(DeviceException.OPERATION_FAILED,
                                       self._error.Msg)
            else:
                self._logger.info("Ping successful:  %s" % self._error.Msg)
        return Global.SUCCESS, "no errors"
