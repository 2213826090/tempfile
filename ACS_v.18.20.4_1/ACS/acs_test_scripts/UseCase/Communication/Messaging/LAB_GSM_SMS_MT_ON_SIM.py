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
:summary: implements a usecase to check the capacity of sms stored on sim card.
:since: 30/07/2014
:author: Mihai Rebrisoreanu
"""

from acs_test_scripts.Utilities.SmsUtilities import \
    compute_sms_equals, compute_sms_segments, SmsMessage
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.Utilities.CommunicationUtilities import SerialHandler, ATCommandAnalyser
from LAB_GSM_SMS_CS_BASE import LabGsmSmsCsBase
from ErrorHandling.DeviceException import DeviceException


class LabGsmSmsMtOnSim(LabGsmSmsCsBase):

    """
    GSM Mobile Terminated SMS over CS on network simulator class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        LabGsmSmsCsBase.__init__(self, tc_name, global_config)
        # Set an artificial phone number
        self._destination_number = "123456789"
        # Read the test case params from test case xml
        # Get the max sms we can stored in sim card
        self._sms_amount_to_send = \
            int(self._tc_parameters.get_param_value("SMS_AMOUNT_TO_SEND"))
        # Retrieve testcase parameters for AT proxy usage
        self._launch_mode = 2
        self._command = "AT+CPMS?"
        self._index = 0
        self._expected_result = ""
        self._command_timeout = 20

        # Instantiate the PhoneSystem UE Command category
        self._phone_system = self._device.get_uecmd("PhoneSystem")

        # Instantiate the modem UE commands
        self._modem_flashing_api = self._device.get_uecmd("ModemFlashing")

        # Get Serial Handler instance
        self._serial_handler = SerialHandler()

        # Read SMS_TRANSFER_TIMEOUT from xml UseCase parameter file
        self._sms_transfer_timeout = \
            int(self._tc_parameters.get_param_value("SMS_TRANSFER_TIMEOUT"))

#------------------------------------------------------------------------------
    def set_up(self):
        """
        Set up the test configuration
        """
        # Call the LAB_GSM_SMS_CS_BASE Setup function
        LabGsmSmsCsBase.set_up(self)

        #Start up the proxy for sending AT commands
        self.at_proxy_com_port = self._modem_flashing_api.start_at_proxy_from_mos(
                                                                        int(self._launch_mode))
        #check serial connection
        self._serial_handler.set_data_analyser(
            ATCommandAnalyser(self._expected_result))
        self._serial_handler.set_default_timeout(self._command_timeout)
        self._logger.info("Connecting to the port " + str(self.at_proxy_com_port))

        # Connect to the at proxy
        self._serial_handler.connect(self.at_proxy_com_port)

        # Check that the modem is available
        modem_status = self._modem_flashing_api.ping_modem_from_serial(
            self._serial_handler.get_serial())

        if modem_status:
            self._logger.info("AT Proxy correctly respond to ping command")
            self.modem_available = True
        else:
            raise DeviceException(DeviceException.CONNECTION_LOST,
                                         "Connection to AT proxy failed")

        #Send AT+CPMS to get storage SIM information
        interrogate_at_command = "AT+CPMS?"
        self._logger.info("Send AT command: %s" % interrogate_at_command)
        self._at_inter_nvm_em_verdict, response =\
                self._serial_handler.send_at_command_and_get_result(interrogate_at_command, self._command_timeout)

        try:
            sm_index = response.split(",").index('"SM"')
        except ValueError:
            self._error.Msg = "Storage SIM information error"
            raise DeviceException(DeviceException.PHONE_OUTPUT_ERROR, self._error.Msg)

        self.max_storage = response.split(",")[sm_index + 2]

        # Clear NS and Phone side sms
        self._ns_messaging_2g.clear_message_data()

        self._messaging_api.delete_all_sms()

        self._nb_segments = \
                compute_sms_segments(self._sms_text,
                                 self._nb_bits_per_char)

        self.sms_sent = SmsMessage(self._sms_text,
                                    self._destination_number,
                                    "GSM",
                                    self._ns_messaging_2g,
                                    self._messaging_api,
                                    self._data_coding_sheme,
                                    self._nb_bits_per_char,
                                    self._sms_transfer_timeout,
                                    self._content_type,
                                    "MT")

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------
    def run_test(self):
        """
        Execute the test
        """

        # Call the LAB_GSM_SMS_CS_BASE run_test function
        LabGsmSmsCsBase.run_test(self)

        # Send sms_amount_to_send SMS to sim card.
        for i in xrange(1, self._sms_amount_to_send):

            if(self._nb_segments > 1):
                # [SEND MO SMS PROCESS]
                # Activate loopback on equipment
                self._ns_messaging_2g.set_sms_mo_loopback_state("ON")

                # Enable message queuing
                self._ns_messaging_2g.set_sms_message_queuing_state("ON")

                # Send SMS to equipment using SMS parameters :
                # - SMS_TEXT
                # - DESTINATION_NUMBER
                # Also ask the UE Command to use synchronization
                self._messaging_api.send_sms(
                    self._destination_number,
                    self._sms_text,
                    True)

                # Wait all incoming sms from DUT to Network Simulator
                self._ns_messaging_2g.\
                check_sms_delivery_state(self.sms_sent,
                                         self._nb_segments,
                                         self._sms_transfer_timeout)

                # Read all messages in the queue
                self._ns_messaging_2g.read_all_received_sms()

            else:

                # retrieve instance in order to get sent messages
                self._ns_messaging_2g.\
                        select_sms_content(self._content_type)

                if(self._content_type == "CTEX"):
                    self._ns_messaging_2g.\
                        set_custom_sms_text(self._sms_text)
                elif (self._content_type == "CDAT"):
                    self._ns_messaging_2g.\
                        set_custom_sms_data(self._sms_text)

                # Send SMS to CDK using SMS parameters :
                # - SMS_TEXT
                self.sms_sent.send_sms()

                # Check sms acknowledged by network simulator
                self._ns_messaging_2g.check_sms_state(
                    'ACK', self._sms_transfer_timeout)

            # get sms received
            sms_received = self._messaging_api.wait_for_incoming_sms(self._sms_transfer_timeout)

            # Compare sent and received SMS (Text,
            # Destination number)
            (result_verdict, result_message) = \
                compute_sms_equals(self.sms_sent, sms_received)

            if result_verdict == Global.FAILURE:
                self._logger.error(result_message)
                # Exit loop
                break

        return result_verdict, result_message

    def tear_down(self):
        """
        End and dispose the test
        """

        #Stop AT Proxy
        self._logger.info("Stop AT Proxy")
        self._modem_flashing_api.stop_at_proxy_from_mos()

        #Close the serial connection
        self._logger.info("Close the serial connection")
        self._logger.info("Disconnecting from the port " +
                            str(self._serial_handler.get_port()))
        self._serial_handler.disconnect()

        return Global.SUCCESS, "No errors"
