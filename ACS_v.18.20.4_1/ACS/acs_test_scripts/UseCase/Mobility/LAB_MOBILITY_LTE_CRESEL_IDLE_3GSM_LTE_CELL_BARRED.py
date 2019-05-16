"""
:copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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
:summary: cell reselections 3G/3G barred/LTE UC
:since: 20/06/2014
:author: /Mihai Rebrisoreanu
"""

import time
from UtilitiesFWK.Utilities import Global
from UseCase.Mobility.LAB_MOBILITY_LTE_3GSM_BASE import LabMobilityLte3gsmBase
import acs_test_scripts.Utilities.RegistrationUtilities as regutil

class LabMobilityLteCreselIdle3gsmLteCellBarred(LabMobilityLte3gsmBase):

    """
    Usecase base for mobility LTE-3GSM use cases
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

        # Check registration state is connected using
        # registrationTimeout from Device_Catalog.xml
        self._registration_timeout = 180

        self._phone_system = self._device.get_uecmd("PhoneSystem")

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Set up the test configuration
        """
        # Call LabMobilityLte3gsmBase set_up function
        LabMobilityLte3gsmBase.set_up(self)

        # Setting the EPC OFF on PXT
        self._ns_lte_data.set_epc_off()

        # Set Network Simulator 3GSM cell on
        self._ns_3gsm_cell.set_cell_on()

        # Set the LTE cell on
        self._ns_lte_cell.set_cell_on(self._ns_lte_mimo)

        # Set entries in the equivalent PLMN list
        self._ns_3gsm_cell.set_equivalent_plmn_list_points(self._ns_lte_mcc, self._ns_lte_mnc)

        # Disconnect from external EPC
        self._ns_3gsm.disconnect_from_external_epc()

        #  Set External EPC connection
        self._ns_3gsm_cell.set_external_epc_connection(self._ns_lte_ip_lan1,
                                                       self._ns_lte_dl_earfcn)

        # Set EPC on
        self._ns_lte_data.set_epc_on()

        # STOP the scenario for PXT for allowing the DUT to camp on 3G/2G cell before
        self._ns_lte.stop_scenario()

        # Flight mode deactivation
        self._networking_api.set_flight_mode("off")

        # Set primary cell to first cell
        self._cell_in_use = 1

        self._logger.info("Check if DUT is attached to cell %d (DUT check)",
                          self._cell_in_use)
        self._modem_api.check_cdk_registration_bfor_timeout(self._registration_timeout)

        # Check Data Connection State => ATTACHED before timeout
        self._logger.info("Check if DUT is attached to cell %d (NW check)",
                          self._cell_in_use)
        self._ns_3gsm_data.check_data_connection_state("ATTACHED",
                                                   self._registration_timeout,
                                                   blocking=False)

        # Activate PDP context
        time.sleep(self._wait_btwn_cmd)
        self._logger.info("Active PDP Context...")
        self._networking_api.activate_pdp_context(self._ssid, False)

        # Check Data Connection State => PDP_ACTIVE before timeout
        wait_pdp_active_state_timeout = 100
        regutil.check_dut_data_connection_state_before_timeout("PDP_ACTIVE",
                                                               self._ns_3gsm_cell,
                                                               self._networking_api,
                                                               self._logger,
                                                               wait_pdp_active_state_timeout,
                                                               flightmode_cycle=False,
                                                               blocking=False)
        # Check that DUT is registered on the good RAT
        self._logger.info("Check if DUT is attached to cell %d with the good RAT",
                          self._cell_in_use)
        self._modem_api.check_network_type_before_timeout(self._ns_3gsm_data.get_network_type(),
                                                          self._registration_timeout)

        # Because the DUT is normally 4G preferred, run the scenario on PXT only after the DUT
        # is already registered on UTRAN cell
        self._ns_lte.start_scenario()

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """

        # Call LabMobilityLte3gsmBase run_test function
        LabMobilityLte3gsmBase.run_test(self)

        # Reset the stay on setting if plugged
        self._phone_system.set_stay_on_while_plugged_in(0)

        # Set screen off timeout to 5 seconds
        self._phone_system.set_screen_timeout(5)

        self.get_logger().info("Waiting for DUT to go in IDLE mode")
        cell_barred_ok = False
        # Wait until DUT is in IDLE mode for barring the 3G cell
        while cell_barred_ok is False:
            rrc_state = str(self._ns_3gsm_data.get_rrc_states()).lower()
            if "idle" in rrc_state:
                msg = "DUT reach IDLE mode"
                self._logger.info(msg)
                # Set the 3G cell as BARRED
                self._logger.info("Setting the 3G cell as BARRED")
                self._ns_3gsm_cell.set_barred_state("ON")
                # Query the barred status
                barred_status = self._ns_3gsm_cell.get_barred_state()
                if str(barred_status) != "1":
                    msg = "Operation has failed - the cell is not barred"
                    self.get_logger().info(msg)
                else:
                    cell_barred_ok = True
            time.sleep(1)

        # Check if DUT has selected the LTE cell before loosing the registration
        if self._ns_lte_data.get_data_connection_status(self._ns_lte_cell_id) in ("REG", "IDLE", "CON"):
            msg = "DUT has immediately selected the 4G cell, didn't lose the registration before"
            self.get_logger().error(msg)
            result_verdict = Global.FAILURE
            return result_verdict, msg

        # Screen wake up
        self._phone_system.wake_screen()

        # Check if DUT goes to "No Service" state
        self._modem_api.check_cdk_no_registration_bfor_timeout(self._registration_timeout)

        # NEXT:
        # Checking if DUT is reselecting the 4G cell

        # Check Data Connection State => CON before timeout
        self._ns_lte_data.check_data_connection_state("CON",
                                                  self._registration_timeout,
                                                  blocking=True,
                                                  cell_id=self._ns_lte_cell_id)

        # Activate PDP context
        time.sleep(self._wait_btwn_cmd)
        self._logger.info("Active PDP Context...")
        self._networking_api.activate_pdp_context(self._ssid, False)

        self._logger.info("Wait 10 sec after registering on LTE Cell")
        time.sleep(10)

        # In all other cell reselection cases a ping is performed
        packet_loss = self._networking_api.ping(self._server_ip_address,
                                                self._packet_size,
                                                self._nb_pings)

        message = "Measured Packet Loss: %.0f%s (Target: %.0f%s)"\
            % (packet_loss.value,
               packet_loss.units,
               self._target_ping_packet_loss_rate,
               packet_loss.units)
        self._logger.info(message)

        # Compute verdict depending on % of packet loss
        if packet_loss.value > self._target_ping_packet_loss_rate:
            result_verdict = Global.FAILURE
            return result_verdict, message
        else:
            result_verdict = Global.SUCCESS

        return result_verdict, "No error"

    def tear_down(self):
        """
        End and dispose the test
        """

        # Reset the access restriction to 3G cell
        self._logger.info("Set the 3G cell as NOT barred")
        self._ns_3gsm_cell.set_barred_state("OFF")

        # Set the initial screen off timeout
        self._phone_system.set_screen_timeout(60)

        # Call LabMobilityLte3gsmBase "tear_down" function
        LabMobilityLte3gsmBase.tear_down(self)

        return Global.SUCCESS, "No errors"
