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
:summary: This file implements the Mobile Terminated SMS + VC UC while DUT in Flight Mode
:since: 07/04/2016
:author: sahoosax
"""

import time
from LIVE_DUAL_PHONE_VC_BASE import LiveDualPhoneVcBase
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.Utilities.SmsUtilities import compute_sms_equals_dual_phone, SmsMessage
from ErrorHandling.DeviceException import DeviceException
from Device.DeviceManager import DeviceManager

class LiveDualPhoneVcSmsFlightmode(LiveDualPhoneVcBase):

    """
    Live SMS + VC while DUT in Flight Mode.
    """

    def __init__(self, tc_name, global_config):

        """
        Constructor
        """
        LiveDualPhoneVcBase.__init__(self, tc_name, global_config)

        # Get phone number of the DUT
        self._phone_number1 = str(self._device.get_phone_number())

        # Get phone number of the reference phone
        if self._phone2 is not None:
            self._phone_number2 = str(self._phone2.get_phone_number())

        # Read SMS_TRANSFER_TIMEOUT from xml UseCase parameter file
        self._sms_transfer_timeout = \
            int(self._tc_parameters.get_param_value("SMS_TRANSFER_TIMEOUT"))

        # The SMS message content
        self._message = \
            str(self._tc_parameters.get_param_value("MESSAGE_CONTENT"))

        self._callduration = \
            int(self._tc_parameters.get_param_value("CALL_DURATION"))

        # Retrieve phone system APIs
        self._phonesystem_api = self._device.get_uecmd("PhoneSystem")

        # Get UECmdLayer
        self._voice_call_api = self._device.get_uecmd("VoiceCall")
        self._messaging_api = self._device.get_uecmd("SmsMessaging")
        self._networking_api = self._device.get_uecmd("Networking")
        self._phone_system_api = self._device.get_uecmd("PhoneSystem")

        # Load instance of the PHONE2
        self._phone2 = DeviceManager().get_device("PHONE2")

        if self._phone2 is not None:
            self._messaging_api2 = self._phone2.get_uecmd("SmsMessaging")
            #self._networking_api2 = self._phone2.get_uecmd("Networking")

        else:
            self._voice_call_api2 = None
            self._networking_api2 = None

#---------------------------------------------------------------------------------------------------------
    def set_up(self):
        """
        Validating the test parameters.
        """
        # Calling Base Class SetUp method
        LiveDualPhoneVcBase.set_up(self)

        # If Flight Mode was set
        if self._networking_api.get_flight_mode():
            # Disable flight mode and wait
            # for the phone to settle down
            self._logger.info("Disabling Flight Mode for secondary DUT.")
            self._networking_api.set_flight_mode("off")

        if self._networking_api2.get_flight_mode():
            # Disable flight mode and wait
            # for the phone to settle down
            self._logger.info("Disabling Flight Mode for main DUT.")
            self._networking_api2.set_flight_mode("off")
            time.sleep(30)
        # Other wise we may have some pending calls
        else:
            # Release any previous call (Robustness)
            self._logger.info("Releasing any previous active Voice Call.")
            self._voice_call_api.release()
            self._voice_call_api2.release()

        return Global.SUCCESS, "No errors"
#---------------------------------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """

        # Call the Base Class run_test method
        UseCaseBase.run_test(self)

        #Enable flight mode and wait for the phone to settle down
        self._networking_api2.set_flight_mode("on")
        time.sleep(30)

        # Perform Voice Call after enabling flight Mode
        self.__perform_voice_call_flight_mode()

        # Perform SMS after enabling flight Mode
        time.sleep(15)
        self._logger.info("Sending SMS in Flight Mode.")  # pylint: disable-msg=E1101
        (verdict, message) = self.__send_sms()
        if verdict == Global.SUCCESS:
            return Global.FAILURE, message

        # We wait a little longer before disabling flight mode
        time.sleep(15)

        #Disable flight mode and wait for the phone to settle down
        self._networking_api2.set_flight_mode("off")
        time.sleep(120)

        #Perform another Voice Call and SMS to check that the Flight Mode has actually been disabled
        self.__perform_voice_call()
        time.sleep(15)

        self._logger.info("Sending SMS in disabled Flight Mode."
        (verdict, message) = self.__send_sms()
        if verdict == Global.FAILURE:
            return Global.FAILURE,message

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        Disposes this test.
        """
        UseCaseBase.tear_down(self)

        # Release the call
        self._logger.info("Releasing any previous active Voice Call.")
        self._voice_call_api.release()
        self._voice_call_api2.release()

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def __perform_voice_call_flight_mode(self):
        """
        Performs a Voice Call with this test's parameters.

        :raise DeviceException: if the Voice Call was not
            successful.
        """

        # Dial from the secondary DUT
        self._voice_call_api.dial(self._phone_number2)

        self._logger.info("Wait for call duration: %ss..." % str(self._callduration))
        time.sleep(self._callduration)

        self.__check_call_state_both_phones(
            self._uecmd_types.VOICE_CALL_STATE.ACTIVE,
            self._uecmd_types.VOICE_CALL_STATE.NOCALL)

        self._voice_call_api.release()
        self._voice_call_api2.release()

#------------------------------------------------------------------------------

    def __perform_voice_call(self):
        """
        Performs a Voice Call with this test's parameters.

        :raise DeviceException: if the Voice Call was not
            successful.
        """

        # Dial from the secondary DUT
        self._voice_call_api.dial(self._phone_number2)

        self._logger.info("Wait for call duration: %ss..." % str(self._callduration))
        time.sleep(self._callduration)

        self.__check_call_state_both_phones(
            self._uecmd_types.VOICE_CALL_STATE.ACTIVE,
            self._uecmd_types.VOICE_CALL_STATE.INCOMING)

        self._voice_call_api.release()
        self._voice_call_api2.release()

#------------------------------------------------------------------------------

    def __send_sms(self):
        """
        Sends a SMS with this test's parameters.

        :raise DeviceException: if the SMS sending was not
            successful.
        """

        # Clear old SMS
        time.sleep(self._wait_btwn_cmd)
        self._messaging_api.delete_all_sms()
        self._messaging_api2.delete_all_sms()

        # Send SMS to equipment using SMS parameters :
        # - SMS_TEXT
        # - DESTINATION_NUMBER
        time.sleep(self._wait_btwn_cmd)
        sms_sent = SmsMessage(self._message, self._phone_number2)

        # register on intent to receive incoming sms on DUT
        self._messaging_api2.register_for_sms_reception()

        self._messaging_api.send_sms(
            self._phone_number2,
            self._message)

        # Get received sms
        sms_received = self._messaging_api2.wait_for_incoming_sms(self._sms_transfer_timeout)

        # Compare sent and received SMS (Text)
        (result_verdict, result_message) = compute_sms_equals_dual_phone(
            sms_sent,
            sms_received)

        return result_verdict, result_message

#-------------------------------------------------------------------------------------------------------------
    def __check_call_state_both_phones(self,
                                       call_state_main_dut,
                                       call_state_secondary_dut):
        """
        Checks the call state for both DUTs

        :type call_state_main_dut: str
        :param call_state_main_dut: State for main DUT
        :type call_state_secondary_dut: str
        :param call_state_secondary_dut: State for secondary DUT

        :raise DeviceException: in case call state is not the expected one
        """

        try:
            # Check call state for PHONE1
            self._voice_call_api.check_state(call_state_main_dut)
        except DeviceException as device_ex:
            err_msg = "For main DUT - " + str(device_ex)
            raise DeviceException(DeviceException.INVALID_DEVICE_STATE, err_msg)

        try:
            # Check call state for PHONE2
            self._voice_call_api2.check_state(call_state_secondary_dut)
        except DeviceException as device_ex:
            err_msg = "For secondary DUT - " + str(device_ex)
            raise DeviceException(DeviceException.INVALID_DEVICE_STATE, err_msg)
