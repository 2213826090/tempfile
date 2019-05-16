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
:summary: LTE SMS over SGS on network simulator
:since: 27/05/2013
:author: hbian
"""

import time

from UtilitiesFWK.Utilities import Global
from acs_test_scripts.Utilities.SmsUtilities import compute_sms_equals, SmsMessage
from acs_test_scripts.UseCase.Networking.LAB_LTE_BASE import LabLteBase
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsConfigException import AcsConfigException


class LabLteSmsSgs(LabLteBase):

    """
    LTE SMS over SGS on network simulator
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call LAB_LTE_BASE Init function
        LabLteBase.__init__(self, tc_name, global_config)

        self._sms_text = self._tc_parameters.get_param_value("SMS_TEXT")

        self._sms_transfer_timeout = \
            int(self._tc_parameters.get_param_value("SMS_TRANSFER_TIMEOUT"))

        # the incoming number
        # The  default number is for CMW500, we can't change it for the moment
        self._incoming_number = \
            self._tc_parameters.get_param_value("INCOMING_NUMBER")
        if self._incoming_number is None or self._incoming_number == "":
            self._incoming_number = "+498941290"

        self._sms_direction = self._tc_parameters.get_param_value("SMS_DIRECTION")

        self._messaging_4g = self._ns.get_cell_4g().get_messaging()
        self._messaging_api = self._device.get_uecmd("SmsMessaging")

#------------------------------------------------------------------------------
    def set_up(self):
        """
        Set up the test configuration
        """

        # Call the LabLteBase Setup function
        LabLteBase.set_up(self)

        # Set Cell on
        self._ns_cell_4g.set_cell_on(self._mimo)

        # Flight mode deactivation
        self._networking_api.set_flight_mode("off")

        # Set APN for LTE and/or IMS depending on protocol IPv4 or IPv6
        self._set_apn_for_lte_and_ims()

        # Activate PDP context
        time.sleep(self._wait_btwn_cmd)
        self._logger.info("Active PDP Context...")
        self._networking_api.activate_pdp_context(self._ssid)

        # Check data connection state is "CON"
        self._check_data_connection_state("CON")

        # Check registration state is connected using
        # registrationTimeout from Device_Catalog.xml
        self._modem_api.check_cdk_registration_bfor_timeout(self._registration_timeout)

        # Get RAT from Equipment
        network_type = self._ns_data_4g.get_network_type()

        # Check that DUT is registered on the good RAT
        self._modem_api.check_network_type_before_timeout(
            network_type,
            self._registration_timeout)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------
    def run_test(self):
        """

        Execute the test
        """

        # Call the LabLteBase run_test function
        LabLteBase.run_test(self)
        self._messaging_api.delete_all_sms()

        if self._sms_direction == "MT":
            sms_sent = SmsMessage(self._sms_text, self._incoming_number, "PSD")

            # register on intent to receive incoming sms
            self._messaging_api.register_for_sms_reception()

            # set custom sms text to equipment
            time.sleep(self._wait_btwn_cmd)
            self._messaging_4g.set_custom_sms_text(self._sms_text)

            # Send sms over sgs
            self._messaging_4g.send_sms_over_sgs()

            # get sms received
            sms_received = self._messaging_api.wait_for_incoming_sms(self._sms_transfer_timeout)
        elif self._sms_direction == "MO":
            # TODO: SMS MO PS is not yet supported by the platform
            # BZ:112539
            raise DeviceException(DeviceException.FEATURE_NOT_AVAILABLE,
                                   "SMS MO PS is not yet supported by the platform")

        else:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                   "SMS_DIRECTION should be Only MO or MT")

        # Compare sent and received SMS (Text, Destination number)
        (result_verdict, result_message) = \
            compute_sms_equals(sms_sent, sms_received)

        self._logger.info(result_message)

        return result_verdict, result_message

    def tear_down(self):
        """
        End and dispose the test
        """

        # Call use case base tear_down function
        LabLteBase.tear_down(self)

        # Clear old SMS
        self._messaging_api.delete_all_sms()

        return Global.SUCCESS, "No errors"
