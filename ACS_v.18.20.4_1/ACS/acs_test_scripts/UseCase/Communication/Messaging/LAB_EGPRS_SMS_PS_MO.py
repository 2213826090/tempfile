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
:summary: EGPRS Mobile Originated SMS over PS on network simulator
:since: 10/02/2011
:author: ssavrimoutou
"""

import time

from acs_test_scripts.Utilities.SmsUtilities import SmsMessage
from UtilitiesFWK.Utilities import Global
from LAB_EGPRS_SMS_PS_BASE import LabEgprsSmsPsBase


class LabEgprsSmsPsMo(LabEgprsSmsPsBase):

    """
    EGPRS Mobile Originated SMS over PS on network simulator class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call LAB_EGPRS_SMS_PS_BASE init function
        LabEgprsSmsPsBase.__init__(self, tc_name, global_config)

        # Read SMS Service Center address
        self._sms_service_center_address = global_config.benchConfig.\
            get_parameters("CELLULAR_NETWORK").get_param_value("SMSC", "default")

        # Read DESTINATION_NUMBER from xml UseCase parameter file
        self._destination_number = \
            self._tc_parameters.get_param_value("DESTINATION_NUMBER")
        if self._destination_number.upper() == "[PHONE_NUMBER]":
            self._destination_number = str(self._device.get_phone_number())

        self._sms = SmsMessage(self._sms_text,
                               self._destination_number,
                               "GPRS",
                               self._ns_messaging_2g,
                               self._messaging_api,
                               self._data_coding_sheme,
                               self._nb_bits_per_char,
                               self._sms_transfer_timeout,
                               self._content_type,
                               "MO")

    def set_up(self):
        """
        Set up the test configuration
        .. warning:: Set the Data coding scheme using DATA_CODING_SCHEME value
        """

        # Call the LAB_GSM_SMS_CS_BASE Setup function
        LabEgprsSmsPsBase.set_up(self)

        # Set the default SMS service center
        time.sleep(self._wait_btwn_cmd)
        self._messaging_api.set_service_center_address(self._sms_service_center_address)

        return Global.SUCCESS, "No errors"

# ------------------------------------------------------------------------------
    def run_test(self):
        """
        Execute the test
        """

        # Call LAB_EGPRS_SMS_PS_BASE run_test function
        LabEgprsSmsPsBase.run_test(self)

        # Calculate how many SMS will be sent to equipment
        time.sleep(self._wait_btwn_cmd)
        self._sms.configure_sms()

        # Send SMS to equipment using SMS parameters :
        # - SMS_TEXT
        # - DESTINATION_NUMBER
        time.sleep(self._wait_btwn_cmd)
        self._sms.send_sms()

        # Check SMS delivery status OK before timeout using
        # SMS_TRANSFER_TIMEOUT value and number of SMS to be received
        return self._sms.get_sms()
