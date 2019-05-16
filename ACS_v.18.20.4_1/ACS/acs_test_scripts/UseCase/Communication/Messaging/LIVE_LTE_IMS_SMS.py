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
:summary:  Use case to validate SMS sending/receiving.
:since: 25/02/2015
:author: amitrofx
"""

import time

from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from UseCase.Networking.LIVE_LTE_IMS_REG import LiveLteImsReg
from Utilities.RegistrationUtilities import ImsRegistrationStatus
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.Utilities.SmsUtilities import compute_sms_equals, SmsMessage
from ErrorHandling.AcsConfigException import AcsConfigException

class LiveLteImsSms(LiveLteImsReg):

    """
    Test of SMS sending/receiving over LTE.

    This test case requires a Com4Innov cell to work.
    Because of this constraint some values that should
    have been given as parameters are actually hard-coded.
    That is to be changed if this test ever has to be
    executed in a different environment.
    """

    def __init__(self, tc_name, global_config):
        """
        Initializes this instance.
        """
        # Call inherited initializer
        LiveLteImsReg.__init__(self, tc_name, global_config)

        # Retrieve the content of the message and check the parameter before going any further
        message_content = self._tc_parameters.get_param_value("MESSAGE_CONTENT")
        if message_content :
            self._message = str(message_content)
        else:
            message = "Invalid parameter value: %s for parameter '%s'." % (
                str(message_content),
                "EXECUTION_TIMEOUT")
            self._logger.error(message)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, message)

        # Initialize the attribute to use as phone number
        self._destination_number = self._tc_parameters.get_param_value("DESTINATION_NUMBER")
        # For a stand alone bench, update the phone number with the one defined in bench configuration file
        if self._destination_number == "BB:phone_number":
            # Try to retrieve the phone number from bench configuration file
            self._destination_number = \
                self._retrive_parameter_bench_config(global_config, "PhoneNumberDut1")

        # Read SMS_TRANSFER_TIMEOUT from xml UseCase parameter file
        self._sms_transfer_timeout = \
            int(self._tc_parameters.get_param_value("SMS_TRANSFER_TIMEOUT"))

        if self._ims_reg_operation == "CHECK_ONLY":
            self._perform_ims_registration = False
        else:
            self._perform_ims_registration = True

        # Instantiate UE categories for first device
        self._sms_api = self._device.get_uecmd("SmsMessaging")
        self._networking_api = self._device.get_uecmd("Networking")

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initializes the test.
        """
        # Call the inherited set_up method
        # Ugly temporary solution before implementing something
        # better using test steps.
        if self._perform_ims_registration:
            # Call inherited setup method
            self._logger.info("Performing IMS registration step as requested.")
            LiveLteImsReg.set_up(self)
            # Disable Flight Mode on the device
            self._networking_api.set_flight_mode("off")
        else:
            # Call the base setup method in order to be
            # compliant with ACS framework
            self._logger.info("Skipping IMS registration step (assumed to be done).")
            UseCaseBase.set_up(self)
            # Simply perform a IMS registration check
            self._networking_api.check_ims_registration_before_timeout(1)

        # Set up is done correctly
        return (Global.SUCCESS, "No errors")

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Runs the test.
        """
        # Call inherited run_test method which will ensure IMS registration
        # Ugly temporary solution before implementing something
        # better using test steps.
        if self._perform_ims_registration:
            LiveLteImsReg.run_test(self)
        else:
            UseCaseBase.run_test(self)

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

        # Because in case of failure the IMS stack may have crashed
        # we need to double-check the IMS registration status.
        in_service_status = ImsRegistrationStatus.in_service()
        registration_status = self._get_registration_status()
        self._logger.info("Registation status: %s (%d)" % (
            str(registration_status),
            registration_status.as_int()))
        # Compute the verdict
        if str(registration_status) != str(in_service_status):
            result_message = "IMS stack has crashed or IMS registration is lost."
            self._logger.error(result_message)
            result_verdict = Global.FAILURE

        # Return the test result
        return (result_verdict, result_message)

    def tear_down(self):
        """
        Disposes this test.
        """
        # Call the inherited tear_down method
        # Ugly temporary solution before implementing something
        # better using test steps.
        if self._perform_ims_registration:
            LiveLteImsReg.tear_down(self)
        else:
            UseCaseBase.tear_down(self)

        # Clear old SMS (Non blocking for this test if function isn't
        # implemented on CDK)
        time.sleep(self._wait_btwn_cmd)
        self._sms_api.delete_all_sms()

        # End of tear_down
        return (Global.SUCCESS, "No errors")
