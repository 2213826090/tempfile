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
:summary: Test The Circuit Switched Fallback from LTE to UTRAN
:since: 15/04/2013
:author: lvacheyx
"""
import time
import acs_test_scripts.Utilities.RegistrationUtilities as RegUtil
from acs_test_scripts.UseCase.Mobility.LAB_MOBILITY_LTE_3GSM_BASE import LabMobilityLte3gsmBase
from acs_test_scripts.Utilities.NetworkingUtilities import ping
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsBaseException import AcsBaseException
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.Utilities.NetworkingUtilities import wait_for_dut_ipv4_address


class LabMobilityLteCsfb(LabMobilityLte3gsmBase):

    """
    Usecase base for mobility LTE handover use cases
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        LabMobilityLte3gsmBase.__init__(self, tc_name, global_config)

        # Get FTP server parameters
        self._server = \
            global_config.benchConfig.get_parameters("LAB_LTE_SERVER")
        self._server_ip_address = self._server.get_param_value("IP")
        self._username = self._server.get_param_value("username")
        self._password = self._server.get_param_value("password")
        if self._server.has_parameter("ftp_path"):
            self._ftp_path = self._server.get_param_value("ftp_path")
        else:
            self._ftp_path = ""

        # Read 3GSM_CELL_POWER from testcase xml parameters
        self._ns_3gsm_cell_power = \
            float(self._tc_parameters.get_param_value("3GSM_CELL_POWER"))

        # Read LOSS_COVERAGE_SIDE from testcase xml parameters
        self._loss_coverage_side = \
            str(self._tc_parameters.get_param_value("LOSS_COVERAGE_SIDE"))

        # Read LOSS_COVERAGE_TECHNO from testcase xml parameters
        self._loss_coverage_type = \
            str(self._tc_parameters.get_param_value("LOSS_COVERAGE_TYPE"))

        # Read PHONE_NUMBER from testcase xml parameters
        self._phone_number = \
            str(self._tc_parameters.get_param_value("PHONE_NUMBER"))
        if self._phone_number.upper() == "[PHONE_NUMBER]":
            self._phone_number = str(self._device.get_phone_number())

        # Read VC_TYPE from testcase xml file
        self._vc_type = str(self._tc_parameters.get_param_value("VC_TYPE"))

        # Read RELEASE_VC_SIDE from testcase xml file
        self.release_csfb_vc_type = str(self._tc_parameters.get_param_value("RELEASE_VC_TYPE"))

        # Read CALL_DURATION from test case xml file
        self._call_duration = \
            int(self._tc_parameters.get_param_value("CALL_DURATION"))

        # Read the number of pings to do
        self._nb_pings = self._tc_parameters.get_param_value("PACKET_COUNT")

        # Read the data size of a packet
        self._packet_size = self._tc_parameters.get_param_value("PACKET_SIZE")

        # Get target % of received packet for ping
        self._target_ping_packet_loss_rate = \
            self._tc_parameters.get_param_value("TARGET_PACKET_LOSS_RATE")

        # Get the "DISABLE_DATA" parameter
        self._disable_data_during_voice_call = \
            self._tc_parameters.get_param_value("DISABLE_DATA", "FALSE")

        self._epc_conn_active = False
        self._ns_cell_tech_neighbour = None

# ------------------------------------------------------------------------------

    def set_up(self):
        """
        Set up the test configuration
        """
        if self._target_ping_packet_loss_rate in ("", None):
            self._target_ping_packet_loss_rate = float(0)
        else:
            self._target_ping_packet_loss_rate = float(
                self._target_ping_packet_loss_rate)

        # Call LabMobilityLte3gsmBase set_up function
        LabMobilityLte3gsmBase.set_up(self)

        if self._ns_3gsm_cell_tech == "2G":
            # Erasing all neighbor cells.
            self._ns_3gsm_cell.remove_all_eutran_neighbor_cells()
            self._ns_3gsm_cell.remove_all_utran_neighbor_cells()
            self._ns_3gsm_cell.remove_all_gsm_pbcch_neighbor_cells()
            self._ns_3gsm_cell.remove_all_gsm_bch_neighbor_cells()
            # Adding only one LTE neighbor cell.
            self._ns_3gsm_cell.set_eutran_neighbor_cell([300], [1], [3], [1],
                                                        [6], [4], [1], [2],
                                                        [1], [-100])

        # if we have to perform ping on 3G cell, "Follow-On Proceed" will be present in the Routing Area Update Request Message control to MANUAL
        if self._loss_coverage_type != "DECREASE" and self._ns_3gsm_cell_tech == "3G":
            self._ns_3gsm_cell.set_rau_fop_control(0, 1)
        elif self._ns_3gsm_cell_tech == "3G":
            self._ns_3gsm_cell.set_rau_fop_control(1, 0)
        # Wait 30 sec
        self._logger.info("Wait 30 seconds before powering on LTE cell to"
                          " ensure DUT is unregistered")
        self._modem_api.check_cdk_no_registration_bfor_timeout(30)

        # Set LTE cell on
        self._ns_lte_cell.set_cell_on(self._ns_lte_mimo)

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

        # Check Data Connection State => CON before timeout
        self._ns_lte_data.check_data_connection_state("CON",
                                                      self._registration_timeout,
                                                      blocking=False,
                                                      cell_id=self._ns_lte_cell_id)

        # Check that DUT is registered on the good RAT
        self._modem_api.check_network_type_before_timeout(self._ns_lte_data.get_network_type(),
                                                          self._registration_timeout)

        # wait until DUT get an IP address from NW
        wait_for_dut_ipv4_address(self._registration_timeout, self._networking_api, self._device.get_cellular_network_interface())

        # Disable traffic
        self._networking_api.disable_output_traffic_but_ping()

        # Set Network Simulator 3GSM cell on
        self._ns_3gsm_cell.set_cell_on()
        # Set entries in the equivalent PLMN list
        self._ns_3gsm_cell.set_equivalent_plmn_list_points(self._ns_lte_mcc, self._ns_lte_mnc)

        # Set External EPC connection
        self._ns_3gsm_cell.set_external_epc_connection(self._ns_lte_ip_lan1, self._ns_lte_dl_earfcn)

        return Global.SUCCESS, "No errors"
# ------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        # Call LabMobilityLte3gsmBase run_test function
        LabMobilityLte3gsmBase.run_test(self)

        # Establish MO or MT voice call
        self.originate_csfb_vc()

        if self._loss_coverage_side not in ("LTE", "3GSM") and \
                self._loss_coverage_type not in ("CELL_OFF", "DECREASE"):

            # If NS Cell Tech is 3G or 4G, ping during Voice call
            # will be possible. If NS Cell Tech is 2G,
            # ping can not be performed as data will be SUSPENDED
            # if voice call is on-going
            if self._ns_3gsm_cell_tech != "2G":
                time.sleep(10)
                # Check that DUT is registered on the good RAT
                self._modem_api.check_network_type_before_timeout(self._ns_3gsm_data.get_network_type(),
                                                                  self._registration_timeout)

                self.ping_with_retry()

            if self._ns_3gsm_cell_service == "GSM":
                if self._ns_3gsm_data.get_data_connection_status() == "PDP_ACTIVE":
                    self._logger.error("PDP is active while cell service is GSM")
                    return (Global.FAILURE, "PDP is active while cell service is GSM")
            # Check call state "CONNECTED" on NW and DUT
            self.check_call_connection()

            # Check if mobile data shall be disabled/enabled, only in case of UTRAN cell
            if (self._disable_data_during_voice_call in ("TRUE", "BOTH") and self._ns_3gsm_cell_tech != "2G"):
                if self._disable_data_during_voice_call == "TRUE":
                    # Disable the PDP context
                    self._logger.info("Disable PDP context")
                    self._networking_api.deactivate_pdp_context(self._ssid, check=True)
                    # Wait 10 seconds
                    time.sleep(10)

                if self._disable_data_during_voice_call == "BOTH":
                    # Disable the PDP context
                    self._logger.info("Disable PDP context")
                    self._networking_api.deactivate_pdp_context()
                    # Wait 10 seconds
                    time.sleep(10)

                    # Activate again the PDP context
                    self._networking_api.activate_pdp_context(self._ssid, check=True)

                    # Ping the server on 3GSM cell
                    self._logger.info("PING on 3GSM cell")
                    self.ping_with_retry()

            # Release MO or MT voice call
            self.release_csfb_vc()

            # In case the PDP remains active before voice call was ended, the DUT should go back to LTE cell.
            # Raise an error otherwise.
            if (self._disable_data_during_voice_call in ("FALSE", "BOTH")) and (self._modem_api.get_network_type() != "LTE"):
                self._error.Msg = "DUT's RAT is wrong - should be in LTE"
                self._logger.error(self._error.Msg)
                return (Global.FAILURE, self._error.Msg)

            # In case the PDP was disabled before the voice call was ended, the DUT should remain on 3G cell without PDP active.
            # Raise an error otherwise.
            if (self._disable_data_during_voice_call == "TRUE") and (self._modem_api.get_network_type() != "WCDMA"):
                self._error.Msg = "DUT's RAT is wrong - should be camped on UTRAN cell"
                self._logger.error(self._error.Msg)
                return (Global.FAILURE, self._error.Msg)

            self.ping_with_retry()

        else:

            # Loss Coverage on dedicated side (LTE or 3GSM) using a cell off
            # or by decreasing cell power
            self._error.Code = self.loose_coverage(self._loss_coverage_side, self._loss_coverage_type)

        # Reset 3G cell
        self._ns_3gsm_cell.set_cell_power(self._ns_3gsm_cell_power)

        return (self._error.Code, self._error.Msg)

# ------------------------------------------------------------------------------
    def tear_down(self):

        self._ns_3gsm_cell.set_cell_power(self._ns_3gsm_cell_power)
        # Enable traffic
        self._networking_api.enable_output_traffic()
        # Call LabMobilityLte3gsmBase tear_down function
        LabMobilityLte3gsmBase.tear_down(self)

        return (Global.SUCCESS, "No errors")

# ------------------------------------------------------------------------------
    def ping_with_retry(self):
        """
        Try to ping server, if it fails do it again as PS connection can be really active after first ping
        """
        try:
            ping(self._networking_api,
                 self._server_ip_address,
                 self._packet_size,
                 self._nb_pings,
                 self._target_ping_packet_loss_rate,
                 self._logger,
                 blocking=True)
        except:
            self._logger.debug("PING_STAT: ping has been lost on first try")
            # Ping the Server again
            ping(self._networking_api,
                 self._server_ip_address,
                 self._packet_size,
                 self._nb_pings,
                 self._target_ping_packet_loss_rate,
                 self._logger,
                 blocking=True)

# ------------------------------------------------------------------------------
    def check_call_connection(self):
        """
        Check call state "CONNECTED"
        """
        # Check call state "CONNECTED" before callSetupTimeout seconds
        self._ns_3gsm_vc.check_call_connected(2 * self._call_setup_time, blocking=False)

        # Check call status before callSetupTimeout (CDK)
        self._voicecall_api.wait_for_state(self._uecmd_types.VOICE_CALL_STATE.ACTIVE,  # pylint: disable=E1101
                                           self._call_setup_time)

# ------------------------------------------------------------------------------
    def originate_csfb_vc(self, lte_cell_state=True):
        """
        Establish Voice Call
        """
        # Release any previous call (Robustness)
        self._voicecall_api.release()
        time.sleep(self._wait_btwn_cmd)

        if self._vc_type == "MT":

            # Since we are testing CSFB MT, we need to HO from LTE cell to 3GSM
            # using "DL Info CS Service Notify" and message number equals to 1 in this case
            if lte_cell_state:
                self._ns_lte.send_ho_message("1")
            # if LTE cell is OFF perform a MT call
            else:
                self._ns_3gsm_vc.mt_originate_call()
            # pylint: disable=E1101
            # Check call status is incoming before 2*callSetupTimeout as 8960 is a bit stressed at that moment
            self._voicecall_api.wait_for_state(self._uecmd_types.VOICE_CALL_STATE.INCOMING,
                                               2 * self._call_setup_time)

            # Answer incoming call
            self._voicecall_api.answer()

            # Check call state "CONNECTED" on NW and DUT
            self.check_call_connection()

            if self._ns_3gsm_cell_tech == "3G":
                # Check Data Connection State on 3GSM cell => ATTACHED before timeout
                self._ns_3gsm_data.check_data_connection_state("ATTACHED",
                                                               self._registration_timeout,
                                                               blocking=False)

            # Check call is connected for CALL_DURATION seconds
            self._ns_3gsm_vc.is_voice_call_connected(self._call_duration)

        else:
            # Initiate a LTE RRC Connection Release when performing MO Call to go back to IDLE
            self._ns_lte_data.ue_detach()

            # Dial using a dummy hard-coded phone number
            self._logger.info("Calling %s ..." % self._phone_number)
            self._voicecall_api.dial(self._phone_number)

            # Check call state "CONNECTED" on NW and DUT
            self.check_call_connection()

            # Check call is connected for CALL_DURATION seconds
            self._ns_3gsm_vc.is_voice_call_connected(self._call_duration)

# ------------------------------------------------------------------------------
    def release_csfb_vc(self, time_to_wait_before_3g_idle=5):
        """
        Release Voice Call
        """

        if self.release_csfb_vc_type == "NR":
            # Release previous call from the network
            self._ns_3gsm_vc.voice_call_network_release()
        else:
            # Release previous call from the DUT
            self._voicecall_api.release()

        # If loss coverage of LTE cell is performed,
        # need to check that DUT stays registered on 3GSM cell
        if self._loss_coverage_side == "LTE":

            if self._ns_3gsm_cell_tech == "3G":
                # Check Data Connection State on 3GSM cell => PDP Active before timeout
                # Check that DUT is registered on the good RAT
                RegUtil.check_rat_and_data_connection_state(self._ns_3gsm_data,
                                                            "PDP_ACTIVE",
                                                            self._modem_api,
                                                            self._registration_timeout)
        else:
            # Force transition to CELL IDLE in order to release PDP context on 3G cell
            if self._ns_3gsm_cell_tech == "3G":
                # Wait 5 seconds while VC is ended on 8960
                time.sleep(time_to_wait_before_3g_idle)
                self._ns_3gsm_data.set_rrc_transition("IDLE")
            # Check Data Connection State on LTE cell => CON before timeout
            # Check that DUT is registered on the good RAT
            RegUtil.check_rat_and_data_connection_state(self._ns_lte_data,
                                                        "CON",
                                                        self._modem_api,
                                                        self._registration_timeout)

# ------------------------------------------------------------------------------
    def loose_coverage(self,
                       loss_side,
                       loss_type):
        """
        Loose coverage on the asked Cell (LTE or 3GSM)

        :type ns_cell: str
        :param ns_data: Network Simulator on which one
        loss coverage will be performed
        :type loss_type: str
        :param loss_type: Type of loss that will be performed
        """

        if loss_side == "LTE":
            ns_cell = self._ns_lte_cell
            # Anticipation on LTE Voice call API
            # that is not yet present in ACS
            #
            # ns_vc = self._ns_lte_vc
            ns_data2 = self._ns_3gsm_data
            ns_cell_power = self._ns_lte_cell_power_rf1
            expected_data_connection_state2 = "PDP_ACTIVE"
            self._ns_cell_tech_neighbour = self._ns_3gsm_cell_tech
        else:
            ns_cell = self._ns_3gsm_cell
            ns_vc = self._ns_3gsm_vc
            ns_data2 = self._ns_lte_data
            ns_cell_power = self._ns_3gsm_cell_power
            expected_data_connection_state2 = "CON"
            self._ns_cell_tech_neighbour = "4G"

        if loss_type == "CELL_OFF":

            # If NS Cell Tech is 3G or 4G, ping during Voice call
            # will be possible. If NS Cell Tech is 2G,
            # ping can not be performed as data will be SUSPENDED
            # if voice call is on-going
            self._logger.info("CELL TECH NEIGHBOUR is %s" % self._ns_cell_tech_neighbour)
            time.sleep(10)
            # Check that DUT is registered on the good RAT
            self._modem_api.check_network_type_before_timeout(ns_data2.get_network_type(),
                                                              self._registration_timeout)
            if self._ns_cell_tech_neighbour != "2G":
                # Ping the Server
                self.ping_with_retry()
            else:
                try:
                    # Ping the Server, as we are in 2G ping shall be lost
                    packet_loss = self._networking_api.ping(self._server_ip_address, self._packet_size, self._nb_pings)
                except (AcsBaseException, DeviceException) as uecmdex:
                    self._logger.info("CELL TECH is 2G so ping is lost")
                else:
                    if packet_loss.value == float(100):
                        self._logger.info("CELL TECH is 2G so ping is lost")
                    else:
                        self._logger.error("CELL TECH is 2G but ping is not lost Measured Packet Loss: %.0f%s"
                                           % (packet_loss.value, packet_loss.units))
                        return Global.FAILURE
            # Set cell off
            ns_cell.set_cell_off()

            # Release MO or MT voice call
            self.release_csfb_vc()
            if self._ns_cell_tech_neighbour != "2G":
                # Establish MO or MT voice call
                self.originate_csfb_vc(False)

            if self._ns_cell_tech_neighbour == "3G" or \
               self._ns_cell_tech_neighbour == "2G":

                # Check Data Connection State on 3GSM cell => PDP Active before timeout
                ns_data2.check_data_connection_state(expected_data_connection_state2,
                                                     self._registration_timeout,
                                                     blocking=False)

            elif self._ns_cell_tech_neighbour == "4G":

                if self._ns_3gsm_cell_tech == "3G":

                    # Check Data Connection State on 3GSM cell => PDP Active before timeout
                    ns_data2.check_data_connection_state(expected_data_connection_state2,
                                                         self._registration_timeout,
                                                         blocking=False)
            if self._ns_cell_tech_neighbour != "2G":
                # Release MO or MT voice call
                self.release_csfb_vc()
            else:
                try:
                    # Ping the Server
                    self.ping_with_retry()
                except DeviceException as uecmdex:
                    self._logger.error("Ping has failed")
                    return Global.FAILURE

        elif loss_type == "DECREASE":
            self.decrease_cell_power_while_csfb_vc(ns_cell,
                                                   ns_vc,
                                                   ns_data2,
                                                   ns_cell_power,
                                                   expected_data_connection_state2)

            # Ping the Server
            self.ping_with_retry()

        return Global.SUCCESS

# ------------------------------------------------------------------------------
    def decrease_cell_power_while_csfb_vc(self,
                                          ns_cell,
                                          ns_vc,
                                          ns_data,
                                          ns_cell_power,
                                          expected_data_connection_state):
        """
        Decrease Network Simulator Cell Power until voice call is dropped

        :type ns_cell: str
        :param ns_cell: Network Simulator Cell Api
        :type ns_vc: str
        :param ns_vc: Network Simulator Voice Call Api
        :type ns_data: str
        :param ns_data: Network Simulator Data Api
        :type ns_cell_power: float
        :param ns_cell_power: Network Simulator cell power
        on which one cell power decreasing will be done
        :type expected_data_connection_state: str
        :param expected_data_connection_state
        """
        cell_ns_limit_power = -110
        decrementation_step_timer = 5
        decrementation_step_power = 5
        connection_time = 1
        blocking = False

        msg = "Begin to decrease NS cell power from %.2f dBm "
        msg += "to %.2f dBm each %d seconds by step of %.2f dBm until "
        msg += "voice call is dropped."
        self._logger.info(
            msg,
            ns_cell_power,
            cell_ns_limit_power,
            decrementation_step_timer,
            decrementation_step_power)

        # WHILE DUT is registered to the LTE cell
        # cell with Voice Call on-going
        # AND cell_power <= self._cresel_limit_power
        while True:
            # Check call state "CONNECTED"
            if not ns_vc.is_voice_call_connected(connection_time, blocking):
                # If voice call no longer on-going, leave the loop
                break

            # Wait during DECREMENTATION_STEP_TIMER
            time.sleep(decrementation_step_timer)

            # Decrement 3GSM cell power
            ns_cell_power -= decrementation_step_power
            ns_cell.set_cell_power(ns_cell_power)

            # IF ns_cell_power < self._cresel_limit_power
            if ns_cell_power < cell_ns_limit_power:

                # Log that Loss of coverage power limit has been reached
                self._logger.info("The Power's limit has been reached %.2f dBm ", cell_ns_limit_power)
                break

        # IF DUT is registered to the Neighbour cell before registration timeout
        if(ns_data.check_data_connection_state(expected_data_connection_state,
                                               self._registration_timeout,
                                               blocking=False,
                                               cell_id=self._ns_lte_cell_id)):

            # Check registration state is connected using
            # registrationTimeout from Device_Catalog.xml
            self._modem_api.check_cdk_state_bfor_timeout(["registered", "roaming"],
                                                         self._registration_timeout)

            # Check that DUT is registered on the good RAT
            self._modem_api.check_network_type_before_timeout(ns_data.get_network_type(),
                                                              self._registration_timeout)

            # Log that cell reselection is done
            msg = "Voice call has been dropped."
            self._logger.info(msg)

            # Return message and quit the method
            return Global.SUCCESS, msg

        else:
            # Log that Cell Reselection has failed
            msg = "Voice call still on-going."
            self._logger.info(msg)

            # Return message and quit the method
            return Global.FAILURE, msg
