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
:summary: Use Case Live SMS loopback
:since: 06/05/2011
:author: fhu2
"""

import time
from acs_test_scripts.UseCase.Communication.Messaging.LIVE_MESSAGING_BASE import LiveMessagingBase
from acs_test_scripts.Utilities.SmsUtilities import compute_sms_equals, SmsMessage

class LiveSmsLoopback(LiveMessagingBase):

    """
    Use Case Live SMS loopback class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        # Call UseCase base Init function
        LiveMessagingBase.__init__(self, tc_name, global_config)

        # Read SMS_TRANSFER_TIMEOUT from xml UseCase parameter file
        self._sms_transfer_timeout = \
            int(self._tc_parameters.get_param_value("SMS_TRANSFER_TIMEOUT"))

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        # Call UseCase base Run function
        LiveMessagingBase.run_test(self)

        # Clear old SMS (Non blocking for this test if function isn't
        # implemented on CDK)
        time.sleep(self._wait_btwn_cmd)
        self._sms_api.delete_all_sms()

        time.sleep(self._wait_btwn_cmd)

        sms_sent = SmsMessage(self._message, self._destination_number)

        # register on intent to receive incoming sms
        self._sms_api.register_for_sms_reception()

        # Send SMS to equipment using SMS parameters :
        # - SMS_TEXT
        # - DESTINATION_NUMBER
        self._sms_api.send_sms(self._destination_number,
                               self._message)

        # Get received sms
        sms_received = self._sms_api.wait_for_incoming_sms(self._sms_transfer_timeout)

        # Compare sent and received SMS (Text,
        # Destination number)
        (result_verdict, result_message) = \
            compute_sms_equals(sms_sent, sms_received)

        self._logger.info(result_message)

        # Check the DUT is camped on a compatible network with the selected
        # preferred network.
        if self._network_pref is not None:
            self._modem_api.check_rat_with_pref_network(self._network_pref, self._registration_timeout)

        return result_verdict, result_message
