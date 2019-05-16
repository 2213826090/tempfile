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
:summary: Several cell reselections LTE/3G UC
:since: 26/06/2013
:author: /sjamaoui
"""
from UtilitiesFWK.Utilities import Global, str_to_bool
from LAB_MOBILITY_LTE_3GSM_BASE import LabMobilityLte3gsmBase
import time


class LabMobilityLteCreselIdle3gsmLte(LabMobilityLte3gsmBase):

    """
    Usecase base for mobility LTE handover use cases
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        LabMobilityLte3gsmBase.__init__(self, tc_name, global_config)

        # Get LTE server parameters for ping
        self._server = \
            global_config.benchConfig.get_parameters("LAB_LTE_SERVER")
        self._server_ip_address = self._server.get_param_value("IP")
        self._username = self._server.get_param_value("username")
        self._password = self._server.get_param_value("password")

        # Read GSM from testcase xml parameters
        self._back_and_forth = \
            str_to_bool(self._tc_parameters.get_param_value("BACK_AND_FORTH", "FALSE"))

        # Read CRESEL_LIMIT_POWER from testcase xml parameters
        self._cresel_limit_power = \
            float(self._tc_parameters.get_param_value("CRESEL_LIMIT_POWER"))
        # Read CRESEL_POWER from testcase xml parameters
        self._cresel_power = \
            float(self._tc_parameters.get_param_value("CRESEL_POWER"))

        # Read DECREMENTATION_STEP_POWER from testcase xml parameters
        self._decrementation_step_power = \
            int(self._tc_parameters.get_param_value("DECREMENTATION_STEP_POWER"))

        # Read DECREMENTATION_STEP_TIMER from testcase xml parameters
        self._decrementation_step_timer = \
            int(self._tc_parameters.get_param_value("DECREMENTATION_STEP_TIMER"))

        # Read the number of pings to do
        self._nb_pings = self._tc_parameters.get_param_value("PACKET_COUNT")

        # Read the data size of a packet
        self._packet_size = self._tc_parameters.get_param_value("PACKET_SIZE")

        # Get target % of received packet for ping
        self._target_ping_packet_loss_rate = \
            float(self._tc_parameters.get_param_value("TARGET_PACKET_LOSS_RATE"))

        # init wanted registration parameters to a value that
        self._wanted_reg_state = "None"
        self._ns_lte_cell_service = "None"

        self._ns_neighbour_power = self._ns_lte_cell_power_rf1
        self._ns_camped_power = self._ns_3gsm_cell_power

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Set up the test configuration
        """
        # Call LabMobilityLte3gsmBase set_up function
        LabMobilityLte3gsmBase.set_up(self)

        # Set LTE cell on
        self._ns_lte_cell.set_cell_on(self._ns_lte_mimo)

        # Set Network Simulator 3GSM cell on
        self._ns_3gsm_cell.set_cell_on()

        #  Set External EPC connection
        self._ns_3gsm_cell.set_external_epc_connection(self._ns_lte_ip_lan1,
                                                       self._ns_lte_dl_earfcn)
        # Set entries in the equivalent PLMN list
        self._ns_3gsm_cell.set_equivalent_plmn_list_points(self._ns_lte_mcc, self._ns_lte_mnc)

        # Disable flight mode
        self._networking_api.set_flight_mode("off")

        # Check registration state is connected using
        # registrationTimeout from Device_Catalog.xml
        self._modem_api.check_cdk_registration_bfor_timeout(self._registration_timeout)

        # Set the APN
        time.sleep(self._wait_btwn_cmd)
        self._logger.info("Setting APN " + str(self._apn) + "...")
        self._networking_api.set_apn(self._ssid, self._apn)

        # Activate PDP context
        time.sleep(self._wait_btwn_cmd)
        self._logger.info("Active PDP Context...")
        self._networking_api.activate_pdp_context(self._ssid, check=False)

        return Global.SUCCESS, "No errors"
#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """

        # Call LabMobilityLte3gsmBase run_test function
        LabMobilityLte3gsmBase.run_test(self)

        # Check Data Connection State => CON before timeout
        self._ns_3gsm_data.check_data_connection_state("PDP_ACTIVE",
                                                       self._registration_timeout,
                                                       blocking=True)

        # Check registration state is connected using
        # registrationTimeout from Device_Catalog.xml
        self._logger.info("Check network registration status is %s on DUT" %
                          self._wanted_reg_state)
        self._modem_api.check_cdk_state_bfor_timeout(self._wanted_reg_state, self._registration_timeout)
        # Check that DUT is registered on the good RAT
        self._modem_api.check_network_type_before_timeout(self._ns_3gsm_data.get_network_type(),
                                                          self._registration_timeout)
        if self._ns_3gsm_cell_tech == "3G":
            # transition the DUT to RRC_Idle if the UE is on a PDP Context in CELL_DCH or CELL_FACH state
            self._ns_3gsm_cell.set_rrc_state_transition("IDLE")
        # set the resel power
        self._ns_lte_cell.set_cell_power(self._cresel_power)

        msg = "Begin to decrease Camped cell power from %.2f dBm "
        msg += "to %.2f dBm each %d seconds by step of %.2f dBm while "
        msg += "cell reselection isn't performed."
        self._logger.info(msg,
                          self._ns_camped_power,
                          self._cresel_limit_power,
                          self._decrementation_step_timer,
                          self._decrementation_step_power)

        # Decrease cell power on 3GSM and wait for DUT to be camped on LTE
        # and log the result in msg
        msg = self.decrease_cell_power_while_idle(self._ns_3gsm_cell,
                                                  self._ns_camped_power,
                                                  self._ns_lte_cell,
                                                  self._ns_lte_data,
                                                  self._ns_lte_cell_service,
                                                  self._decrementation_step_power,
                                                  self._decrementation_step_timer,
                                                  self._cresel_limit_power,
                                                  self._cresel_power,
                                                  self._ns1_model)

        self._ns_neighbour_power = self._cresel_limit_power
        self._ns_camped_power = self._cresel_power

        # Display Cell reselection result
        self._logger.info(msg)

        # Check Data Connection State => CON before timeout
        self._ns_lte_data.check_data_connection_state("CON",
                                                      self._registration_timeout,
                                                      blocking=True,
                                                      cell_id=self._ns_lte_cell_id)
        # Display Cell reselection result
        self._logger.info(msg)

        self._logger.info("wait 10 sec after registering on LTE Cell")
        time.sleep(10)

        # in all other cell reselection cases a ping is performed
        packet_loss = self._networking_api.ping(self._server_ip_address,
                                                self._packet_size,
                                                self._nb_pings)

        # Compute verdict depending on % of packet loss
        if packet_loss.value > self._target_ping_packet_loss_rate:
            result_verdict = Global.FAILURE
        else:
            result_verdict = Global.SUCCESS

        message = "Measured Packet Loss: %.0f%s (Target: %.0f%s)"\
            % (packet_loss.value,
               packet_loss.units,
               self._target_ping_packet_loss_rate,
               packet_loss.units)
        self._logger.info(message)

        result_message = "cell reselection is done"

        # if we want to do a return cell reselection (go back)
        if self._back_and_forth:
            msg = "Begin to decrease Camped cell power from %.2f dBm "
            msg += "to %.2f dBm each %d seconds by step of %.2f dBm while "
            msg += "cell reselection isn't performed."
            self._logger.info(
                msg,
                self._ns_camped_power,
                self._cresel_limit_power,
                self._decrementation_step_timer,
                self._decrementation_step_power)

            # Decrease cell power on NS1 and wait for DUT to be camped on NS2
            # and log the result in msg
            msg = self.decrease_cell_power_while_idle(self._ns_lte_cell,
                                                      self._ns_camped_power,
                                                      self._ns_3gsm_cell,
                                                      self._ns_3gsm_data,
                                                      self._ns_3gsm_cell_service,
                                                      self._decrementation_step_power,
                                                      self._decrementation_step_timer,
                                                      self._cresel_limit_power,
                                                      self._cresel_power,
                                                      self._ns2_model)

            self._ns_neighbour_power = self._cresel_limit_power
            self._ns_camped_power = self._cresel_power

            # Display Cell reselection result
            self._logger.info(msg)

            # Check Data Connection State => CON before timeout
            self._ns_3gsm_data.check_data_connection_state("ATTACHED",
                                                           self._registration_timeout,
                                                           blocking=True,
                                                           cell_id=self._ns_lte_cell_id)
            self._logger.info("wait 10 sec after registering on 3GSM Cell")
            time.sleep(10)

            # in all other cell reselection cases a ping is performed
            packet_loss = self._networking_api.\
                ping(self._server_ip_address,
                     self._packet_size,
                     self._nb_pings)

            # Compute verdict depending on % of packet loss
            if packet_loss.value > self._target_ping_packet_loss_rate:
                result_verdict = Global.FAILURE
                message = "Ping after cell reselection failed & Measured Packet Loss: %.0f%s (Target: %.0f%s)"\
                % (packet_loss.value,
                   packet_loss.units,
                   self._target_ping_packet_loss_rate,
                   packet_loss.units)
                return (result_verdict, message)
            else:
                result_verdict = Global.SUCCESS
                result_message = "cell reselection is done"

            message = "Measured Packet Loss: %.0f%s (Target: %.0f%s)"\
                % (packet_loss.value,
                   packet_loss.units,
                   self._target_ping_packet_loss_rate,
                   packet_loss.units)
            self._logger.info(message)

        return result_verdict, result_message
