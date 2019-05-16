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
:summary: LTE network registration after a switch On/Off Airplane Mode, with data disabled
:since: 18/12/2014
:author: gcharlex
"""
import time
from LAB_MOBILITY_LTE_SMS import LabMobilityLteSms
from UtilitiesFWK.Utilities import Global


class LabMobilityLteSmsDataInteraction(LabMobilityLteSms):

    """
    Usecase base for 'LTE Network Registration - Interaction with Data option' use cases
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        LabMobilityLteSms.__init__(self, tc_name, global_config)

        # Read the number of pings to do
        self._nb_pings = self._tc_parameters.get_param_value("PACKET_COUNT")

        # Read the data size of a packet
        self._packet_size = self._tc_parameters.get_param_value("PACKET_SIZE")

        # Get server parameters
        self._server = \
            global_config.benchConfig.get_parameters("LAB_SERVER")
        self._server_ip_address = self._server.get_param_value("IP")

        # Get computer type
        self._computer = self._tc_parameters.get_param_value("COMPUTER")
        if self._computer == "":
            self._computer = None
        # Load computer equipment
        if self._computer is not None:
            self._computer = self._em.get_computer(self._computer)

        # Instantiate Messaging UECmd for SMS UseCases
        self._messaging_api = self._device.get_uecmd("SmsMessaging")
# ------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """

        # Clear old SMS on phone
        self._messaging_api.delete_all_sms()

        # Clear old SMS on Network Simulator
        self._ns_3gsm_messaging.clear_message_data()

        # Disable the data service on the DUT
        self._networking_api.deactivate_pdp_context()

        # Check that DUT is still registered on the good RAT
        self._modem_api.check_network_type_before_timeout(self._ns_lte_data.get_network_type(),
                                                          self._registration_timeout)

        # Initiate MO ping - should failed
        self._logger.info("Initiate MO ping - should failed")
        packet_loss = self._networking_api.ping(self._server_ip_address,
                                                self._packet_size,
                                                self._nb_pings,
                                                blocking=False)
        if packet_loss.value != -1 and packet_loss.value < 100:
            return Global.FAILURE, "Ping succeeded when it should not!"
        else:
            self._logger.info("MO Ping failed - OK")

        self._logger.info("Initiate MT ping - should failed")
        packet_loss = self._computer.ping(self._ns1_ip_dut,
                                          self._packet_size,
                                          self._nb_pings)
        if packet_loss.value < 100:
            return Global.FAILURE, "Ping succeeded when it should not!"
        else:
            self._logger.info("MT Ping failed - OK")

        # Turn flight mode ON
        self._logger.info("Set flight mode ON...")
        self._networking_api.set_flight_mode("ON")

        # Check that DUT is no longer camped on Network
        self._modem_api.check_cdk_no_registration_bfor_timeout(self._registration_timeout)

        # Initiate MO ping - should failed
        self._logger.info("Initiate MO ping - should failed")
        packet_loss = self._networking_api.ping(self._server_ip_address,
                                                self._packet_size,
                                                self._nb_pings,
                                                blocking=False)
        if packet_loss.value != -1 and packet_loss.value < 100:
            return Global.FAILURE, "Ping succeeded when it should not!"
        else:
            self._logger.info("Ping failed - OK")

        # Send a short MO SMS
        self._logger.info("Send a short MO SMS")
        time.sleep(self._wait_btwn_cmd)
        try:
            self._messaging_api.send_sms(self.sms.sender,
                                         self.sms.message)
            time.sleep(10)
        except Exception as e:
            # Check that SMS cannot be sent
            self._logger.info("Sms cannot be send - OK")
        else:
            return Global.FAILURE, "Sms sent when it should not"

        # Turn flight mode OFF
        self._logger.info("Set flight mode OFF...")
        self._networking_api.set_flight_mode("OFF")

        # Check DUT correctly selects the LTE cell within a max time of 2 min
        self._logger.info("DUT correctly selects the LTE cell")
        self._modem_api.check_network_type_before_timeout(self._ns_lte_data.get_network_type(),
                                                          self._registration_timeout)

        # Initatiate a MO ping - should failed
        self._logger.info("Initiate MO ping - should failed")
        packet_loss = self._networking_api.ping(self._server_ip_address,
                                                self._packet_size,
                                                self._nb_pings,
                                                blocking=False)
        if packet_loss.value != -1 and packet_loss.value < 100:
            return Global.FAILURE, "Ping succeeded when it should not!"
        else:
            self._logger.info("MO Ping failed - OK")

        # Send a short MO SMS - should succeed
        self._logger.info("Send a short MO SMS")
        self._messaging_api.send_sms(self.sms.sender,
                                     self.sms.message)
        # Check that the SMS is well sent and correctly received at its destination
        (result_verdict, result_message) = \
            self._ns_3gsm_messaging.check_sms_delivery_state(self.sms, self.nb_segments,
                                                             self._sms_transfer_timeout)
        if result_verdict == Global.FAILURE:
            return result_verdict, result_message
        else:
            self._logger.info("SMS sent with succeed")

        # Enable the data service on the DUT
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

        # Initiate a MO ping - should succeeded
        self._logger.info("Initiate MO ping - should succeeded")
        packet_loss = self._networking_api.ping(self._server_ip_address,
                                                self._packet_size,
                                                self._nb_pings)
        if packet_loss >= 0:
            return Global.FAILURE, "Ping failed when it should not!"
        else:
            self._logger.info("Ping succeed")

        return Global.SUCCESS, "No errors"
