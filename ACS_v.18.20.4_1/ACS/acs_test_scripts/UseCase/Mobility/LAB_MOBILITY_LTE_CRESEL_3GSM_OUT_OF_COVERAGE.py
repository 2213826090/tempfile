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
:summary: Several cell reselections LTE/3G/LTE UC
:since: 20/06/2014
:author: /Mihai Rebrisoreanu
"""
from UtilitiesFWK.Utilities import Global, str_to_bool
from LAB_MOBILITY_LTE_3GSM_BASE import LabMobilityLte3gsmBase
from acs_test_scripts.Utilities.SmsUtilities import SmsMessage
import time


class LabMobilityLteCrese3gsmOutOfCoverage(LabMobilityLte3gsmBase):

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

        self._cresel_power = \
            float(self._tc_parameters.get_param_value("CRESEL_POWER"))

        # Read CRESEL_TIME_OUT from testcase xml parameters
        self._cresel_timeout = \
            float(self._tc_parameters.get_param_value("CRESEL_TIME_OUT"))

        # Read CRESEL_TIME_OUT from testcase xml parameters
        self._cresel_nocoverage_time = \
            float(self._tc_parameters.get_param_value("CRESEL_NO_COVERAGE_TIME"))

        # Read the number of pings to do
        self._nb_pings = self._tc_parameters.get_param_value("PACKET_COUNT")

        # Read the data size of a packet
        self._packet_size = self._tc_parameters.get_param_value("PACKET_SIZE")

        # Get target % of received packet for ping
        self._target_ping_packet_loss_rate = \
            float(self._tc_parameters.get_param_value("TARGET_PACKET_LOSS_RATE"))
        # Read SMS_TEXT from testcase xml file
        self._sms_text = self._tc_parameters.get_param_value("SMS_TEXT", "aaaabbbbbcccc")
        # Read PHONE_NUMBER from testcase xml parameters
        self._phone_number = \
            str(self._tc_parameters.get_param_value("PHONE_NUMBER", "0675529935"))
        if self._phone_number.upper() == "[PHONE_NUMBER]":
            self._phone_number = str(self._device.get_phone_number())

        # Instantiate Messaging UECmd for SMS UseCases
        self._messaging_api = self._device.get_uecmd("SmsMessaging")

        # init wanted registration parameters to a value that
        self._wanted_reg_state = "None"
        self._ns_lte_cell_service = "None"
        self._ns_neighbour_power = self._ns_3gsm_cell_power
        self._ns_camped_power = self._ns_lte_cell_power_rf1

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Set up the test configuration
        """
        # Call LabMobilityLte3gsmBase set_up function
        LabMobilityLte3gsmBase.set_up(self)

        if self._ns_3gsm_cell_service == "GSM":
            # Enable message queuing
            self._ns_3gsm_messaging.set_sms_message_queuing_state("ON")
            # MO SMS instance
            self.sms = SmsMessage(self._sms_text, self._phone_number, "GSM")

        # Flight mode deactivation
        self._networking_api.set_flight_mode("off")

        # Set LTE cell on
        self._ns_lte_cell.set_cell_on(self._ns_lte_mimo)

        # Set Network Simulator 3GSM cell on
        self._ns_3gsm_cell.set_cell_on()

        # Set entries in the equivalent PLMN list
        self._ns_3gsm_cell.set_equivalent_plmn_list_points(self._ns_lte_mcc, self._ns_lte_mnc)

        #  Set External EPC connection
        self._ns_3gsm_cell.set_external_epc_connection(self._ns_lte_ip_lan1,
                                                       self._ns_lte_dl_earfcn)


        # Set parameters to go from cell 1 to cell 2
        self._1_to_2_cell_parms = (self._ns_lte_cell,
                                       self._ns_3gsm_cell,
                                       self._ns_3gsm_data,
                                       self._ns_3gsm_cell_service,
                                       self._cresel_power,
                                       self._ns2_model,
                                       self._cresel_timeout,
                                       self._cresel_nocoverage_time)

        # Set parameters to go from cell 2 to cell 1
        self._2_to_1_cell_parms = (self._ns_3gsm_cell,
                                   self._ns_lte_cell,
                                   self._ns_lte_data,
                                   self._ns_lte_cell_service,
                                   self._cresel_power,
                                   self._ns1_model,
                                   self._cresel_timeout,
                                   self._cresel_nocoverage_time)

        return Global.SUCCESS, "No errors"
#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """

        # Call LabMobilityLte3gsmBase run_test function
        LabMobilityLte3gsmBase.run_test(self)

        # Set Network Simulator 3GSM cell off
        self._ns_3gsm_cell.set_cell_off()
        # Check Data Connection State => CON before timeout
        self._ns_lte_data.check_data_connection_state("CON",
                                                      self._registration_timeout,
                                                      blocking=True,
                                                      cell_id=self._ns_lte_cell_id)
        # Check registration state is connected using
        # registrationTimeout from Device_Catalog.xml
        self._logger.info("Check network registration status is %s on DUT" %
                          self._wanted_reg_state)
        self._modem_api.check_cdk_state_bfor_timeout(self._wanted_reg_state, self._registration_timeout)
        # Check that DUT is registered on the good RAT
        self._modem_api.check_network_type_before_timeout(self._ns_lte_data.get_network_type(),
                                                          self._registration_timeout)

        # Set Network Simulator 3GSM cell on
        self._ns_3gsm_cell.set_cell_on()

        # Disconnect from external EPC
        self._ns_3gsm.disconnect_from_external_epc()
        #  Set External EPC connection
        self._ns_3gsm_cell.set_external_epc_connection(self._ns_lte_ip_lan1,
                                                       self._ns_lte_dl_earfcn)

        # Wait 5 seconds
        time.sleep(5)

        # Set primary cell to first cell
        self._cell_in_use = 1
        cell_parms = self._1_to_2_cell_parms
        self._logger.info("Go out of coverage then switch to cell %d",
                           self._cell_in_use)
        self.go_out_of_coverage_and_cresel(*cell_parms)
        self._logger.info("End out of coverage procedure")

        # Case LTE to GSM
        # if the reselection is performed to GSM cell, send an MO SMS
        if self._ns_3gsm_cell_service == "GSM":
            # Send MO SMS
            self._messaging_api.send_sms(self.sms.sender, self.sms.message)
            # Check the MO SMS received On Network Simulator
            (result_verdict, result_message) = \
                self._ns_3gsm_messaging.check_sms_delivery_state(self.sms, 1, self._registration_timeout)
            # Test LTE to GSM cell reselection is finished
            return result_verdict, result_message

        # Case LTE to EGPRS or UTRA
        else:
            # Activate PDP context
            self._logger.info("Active PDP Context...")
            self._networking_api.activate_pdp_context(self._ssid)
            # Check Data Connection State => PDP Active before timeout
            if (self._ns_3gsm_data.check_data_connection_state("PDP_ACTIVE",
                                                           self._registration_timeout)) == False:
                #PDP not activated in time, return error
                message = "Exit - PDP NOT activated"
                self._logger.error(message)
                return Global.FAILURE, message

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

            message = "Measured Packet Loss: %.0f%s (Target: %.0f%s)"\
                % (packet_loss.value,
                   packet_loss.units,
                   self._target_ping_packet_loss_rate,
                   packet_loss.units)
            self._logger.info(message)
            result_message = "cell reselection is done"

        # -------------------------------------------------------------------

        # if we want to do a return cell reselection (go back)
        if self._back_and_forth:

            message = "---- BACK and FORTH ---- Going from 3G cell to 4G cell"
            self._logger.info(message)

            # Set primary cell to first cell
            if self._cell_in_use == 1:
                self._cell_in_use = 2
                cell_parms = self._2_to_1_cell_parms

            self._logger.info("Go out of coverage then switch to cell %d",
                               self._cell_in_use)
            self.go_out_of_coverage_and_cresel(*cell_parms)

            self._logger.info("End out of coverage procedure")

            # Check Data Connection State => CON before timeout
            self._ns_lte_data.check_data_connection_state("CON",
                                                          self._registration_timeout,
                                                          blocking=True,
                                                          cell_id=self._ns_lte_cell_id)
            self._logger.info("wait 10 sec after registering on LTE Cell")
            time.sleep(10)

            # in all other cell reselection cases a ping is performed
            packet_loss = self._networking_api.\
                ping(self._server_ip_address,
                     self._packet_size,
                     self._nb_pings)

            # Compute verdict depending on % of packet loss
            if packet_loss.value > self._target_ping_packet_loss_rate:
                result_verdict = Global.FAILURE
                message = "Ping after second cell reselection failed & Measured Packet Loss: %.0f%s (Target: %.0f%s)"\
                % (packet_loss.value,
                   packet_loss.units,
                   self._target_ping_packet_loss_rate,
                   packet_loss.units)
                return (result_verdict, message)
            else:
                result_verdict = Global.SUCCESS

            message = "Measured Packet Loss: %.0f%s (Target: %.0f%s)"\
                % (packet_loss.value,
                   packet_loss.units,
                   self._target_ping_packet_loss_rate,
                   packet_loss.units)
            self._logger.info(message)
            result_message = "cell reselection is done"

        return result_verdict, result_message
