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
:summary: WCDMA Mobile Originated SMS over CS on network simulator
:since: 23/08/2010
:author: dgo
"""

import time

from UtilitiesFWK.Utilities import Global
from acs_test_scripts.Utilities.SmsUtilities import compute_sms_equals, compute_sms_segments, SmsMessage
from LAB_WCDMA_SMS_PS_BASE import LabWcdmaSmsPsBase


class LabWcdmaSmsPsMt(LabWcdmaSmsPsBase):

    """
    WCDMA Mobile Terminated SMS over PS on network simulator class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        LabWcdmaSmsPsBase.__init__(self, tc_name, global_config)
        # Set an artificial phone number
        self._destination_number = "123456789"

#------------------------------------------------------------------------------
    def set_up(self):
        """
        Set up the test configuration
        """

        # Call the LAB_WCDMA_SMS_PS_BASE Setup function
        LabWcdmaSmsPsBase.set_up(self)

        # Set the preferred connection type to PS Domain ("PSD")
        self._ns_messaging_3g.select_sms_transportation("PSD")

        # Set the Data coding scheme using DATA_CODING_SCHEME value
        self._ns_messaging_3g.set_sms_data_coding_scheme(
            int(self._data_coding_sheme, 16))

        # Set sender address using DESTINATION_NUMBER on Network simulator
        self._ns_messaging_3g.set_sms_sender_address(self._destination_number)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------
    def run_test(self):
        """

        Execute the test
        """

        # Call the LAB_WCDMA_SMS_PS_BASE run_test function
        LabWcdmaSmsPsBase.run_test(self)

        # Calculate how many SMS will be sent to equipment
        time.sleep(self._wait_btwn_cmd)
        nb_segments = compute_sms_segments(
            self._sms_text, self._nb_bits_per_char)

        sms_sent = SmsMessage(self._sms_text, self._destination_number, "PSD")

        # register on intent to receive incoming sms
        self._messaging_api.register_for_sms_reception()

        if nb_segments > 1:
            # [SEND MO SMS PROCESS]

            # Activate loopback on equipment
            self._ns_messaging_3g.set_sms_mo_loopback_state("ON")

            # Enable message queuing
            self._ns_messaging_3g.set_sms_message_queuing_state("ON")

            # Send SMS to equipment using SMS parameters :
            # - SMS_TEXT
            # - DESTINATION_NUMBER
            # Also ask the UE Command to use synchronization
            time.sleep(self._wait_btwn_cmd)
            self._messaging_api.send_sms(
                self._destination_number,
                self._sms_text,
                True)

            # Wait all incoming sms from DUT to Network Simulator
            self._ns_messaging_3g.check_sms_delivery_state(sms_sent, nb_segments,
                                                           self._sms_transfer_timeout)

            # Read all messages in the queue
            self._ns_messaging_3g.read_all_received_sms()

        else:
            # [SEND SMS DIRECTLY BY EQUIPMENT]

            # Configure the type of the message to send CUSTOM TEXT.
            self._ns_messaging_3g.select_sms_content(self._content_type)

            # Set the custom text message to send, using SMS_TEXT parameter
            if self._content_type == "CTEX":
                self._ns_messaging_3g.set_custom_sms_text(self._sms_text)
            elif self._content_type == "CDAT":
                self._ns_messaging_3g.set_custom_sms_data(self._sms_text)

            # Send SMS to CDK using SMS parameters :
            # - SMS_TEXT
            self._ns_messaging_3g.send_sms()

            # Check sms acknowledged by network simulator
            self._ns_messaging_3g.check_sms_state(
                'ACK', self._sms_transfer_timeout)

        # get sms received
        sms_received = self._messaging_api.wait_for_incoming_sms(self._sms_transfer_timeout)

        # Compare sent and received SMS (Text,
        # Destination number)
        (result_verdict, result_message) = \
            compute_sms_equals(sms_sent, sms_received)

        self._logger.info(result_message)

        return result_verdict, result_message
