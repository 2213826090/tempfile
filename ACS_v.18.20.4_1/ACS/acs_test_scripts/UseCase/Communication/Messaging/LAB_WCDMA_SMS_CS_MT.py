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
:summary: WCDMA Mobile Terminated SMS over CS on network simulator
:since: 23/08/2010
:author: dgo
"""

import time

from UtilitiesFWK.Utilities import Global
from acs_test_scripts.Utilities.SmsUtilities import SmsMessage
from LAB_WCDMA_SMS_CS_BASE import LabWcdmaSmsCsBase


class LabWcdmaSmsCsMt(LabWcdmaSmsCsBase):

    """
    WCDMA Mobile Terminated SMS over CS on network simulator class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        LabWcdmaSmsCsBase.__init__(self, tc_name, global_config)
        # Set an artificial phone number
        self._destination_number = "123456789"
        self._sms_sent = SmsMessage(self._sms_text,
                                    self._destination_number,
                                    "CSD",
                                    self._ns_messaging_3g,
                                    self._messaging_api,
                                    self._data_coding_sheme,
                                    self._nb_bits_per_char,
                                    self._sms_transfer_timeout,
                                    self._content_type,
                                    "MT")

# ------------------------------------------------------------------------------
    def set_up(self):
        """
        Set up the test configuration
        """

        # Call the LAB_GSM_SMS_CS_BASE Setup function
        LabWcdmaSmsCsBase.set_up(self)

        self._sms_sent.configure_sms()

        return Global.SUCCESS, "No errors"

# ------------------------------------------------------------------------------
    def run_test(self):
        """
        Execute the test
        """

        # Call the LAB_GSM_SMS_CS_BASE run_test function
        LabWcdmaSmsCsBase.run_test(self)

        # Calculate how many SMS will be sent to equipment
        time.sleep(self._wait_btwn_cmd)

        # Send SMS
        self._sms_sent.send_sms()

        # get sms received and Compare sent and received SMS (Text, Destination number)
        (result_verdict, result_message) = self._sms_sent.get_sms()

        self._logger.info(result_message)

        return result_verdict, result_message
