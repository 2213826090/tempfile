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
:summary: Use case to validate MT Voice Calls using two phones setup.
:since: 23/02/2016
:author: sahoosax
"""
from LIVE_DUAL_PHONE_VC_BASE import LiveDualPhoneVcBase
import time
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException

class LiveDualPhoneVcMtCall(LiveDualPhoneVcBase):

    """
    Test of Live Voice Call with two devices.
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

        # Get Test Cases Parameters
        self._screen_state = \
            (self._tc_parameters.get_param_value("STATE"))

        # Get Test Cases Parameters
        self._action = \
            (self._tc_parameters.get_param_value("ACTION"))

        # Retrieve phone system APIs
        self._phonesystem_api = self._device.get_uecmd("PhoneSystem")

#---------------------------------------------------------------------------------------------------------
    def set_up(self):
        """
        Validating the test parameters.
        """
        # Calling Base Class SetUp method
        LiveDualPhoneVcBase.set_up(self)

        # Checking phone Idle State parameter
        if not self._screen_state:
            message = "%s '%s' %s '%s'" % (
                "Invalid parameter value (or missing parameter)",
                str(self._screen_state),
                "for parameter",
                "STATE")
            raise AcsConfigException(
                AcsConfigException.INVALID_PARAMETER,
                message)

        # Check the MT call Test Scenario parameter
        '''
        Possible values for scenario defined:
        CALL_IDLE_SCREEN,
        DL_AFT_ANS,
        DL_BEF_ANS,
        DR_AFT_ANS,
        DR_BEF_ANS
        '''
        if not self._action:
            message = "%s '%s' %s '%s'" % (
                "Invalid parameter value (or missing parameter)",
                str(self._action),
                "for parameter",
                "ACTION")
            raise AcsConfigException(
                AcsConfigException.INVALID_PARAMETER,
                message)

        return Global.SUCCESS, "No errors"
#---------------------------------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """

        # Call the Base Class run_test method
        UseCaseBase.run_test(self)

        # Release any previous call (Robustness)
        self._voice_call_api.release()
        self._voice_call_api2.release()

        # screen in idle state
        if self._screen_state == "IDLE":
            self._phonesystem_api.set_phone_lock("1")
            self._phonesystem_api.display_off()

        # Answer and release Call when DUT is in idle state
        if self._action == "CALL_IDLE_SCREEN":
            self._logger.info("The call will be initiated by the Secondary DUT")
            self._caller_phone = self._voice_call_api2
            # Set the phone number used for dialing
            self._phone_number = self._phone_number1
            self._receiver_phone = self._voice_call_api
            # Attempt voice call
            self._dial_number()
            # Answer the call
            self._answer_call()
            # End the call
            self._releaser_phone = self._voice_call_api
            self._release_call()

        # Release Call from DUT after answering
        if self._action == "DL_AFT_ANS":
            self._logger.info("The call will be initiated by the Secondary DUT")
            self._caller_phone = self._voice_call_api2
            # Set the phone number used for dialing
            self._phone_number = self._phone_number1
            self._receiver_phone = self._voice_call_api
            # Attempt voice call
            self._dial_number()
            # Answer the call
            self._answer_call()
            # End the call
            self._releaser_phone = self._voice_call_api
            self._release_call()

        # Release Call from DUT before answering
        if self._action == "DL_BEF_ANS":
            self._logger.info("The call will be initiated by the Secondary DUT")
            self._caller_phone = self._voice_call_api2
            # Set the phone number used for dialing
            self._phone_number = self._phone_number1
            self._receiver_phone = self._voice_call_api
            # Attempt voice call
            self._dial_number()
            # End the call
            self._releaser_phone = self._voice_call_api
            self._release_call()
            # Disconnect MO
            time.sleep(self._wait_btwn_cmd)
            self._releaser_phone = self._voice_call_api2
            self._release_call()

        # Release Call from remote end after answering
        if self._action == "DR_AFT_ANS":
            self._logger.info("The call will be initiated by the Secondary DUT")
            self._caller_phone = self._voice_call_api2
            # Set the phone number used for dialing
            self._phone_number = self._phone_number1
            self._receiver_phone = self._voice_call_api
            # Attempt voice call
            self._dial_number()
            self._receiver_phone = self._voice_call_api
            # Answer the call
            self._answer_call()
            # End the call
            self._releaser_phone = self._voice_call_api2
            self._release_call()

        # Release Call from remote end before answering
        if self._action == "DR_BEF_ANS":
            self._logger.info("The call will be initiated by the Secondary DUT")
            self._caller_phone = self._voice_call_api2
            # Set the phone number used for dialing
            self._phone_number = self._phone_number1
            self._receiver_phone = self._voice_call_api
            # Attempt voice call
            self._dial_number()
            # End the call
            self._releaser_phone = self._voice_call_api2
            self._release_call()

        # MO call: Release Call from remote end before answering
        if self._action == "MO_DR_BEF_ANS":
            self._logger.info("The call will be initiated by the Main DUT")
            self._caller_phone = self._voice_call_api
            # Set the phone number used for dialing
            self._phone_number = self._phone_number2
            self._receiver_phone = self._voice_call_api2
            # Attempt voice call
            self._dial_number()
            # End the call
            self._releaser_phone = self._voice_call_api
            self._release_call()

        # MO call: Release Call locally before answering
        if self._action == "MO_DL_BEF_ANS":
            self._logger.info("The call will be initiated by the Main DUT")
            self._caller_phone = self._voice_call_api
            # Set the phone number used for dialing
            self._phone_number = self._phone_number2
            self._receiver_phone = self._voice_call_api2
            # Attempt voice call
            self._dial_number()
            # End the call
            self._releaser_phone = self._voice_call_api2
            self._release_call()

        # Missed call alert on DUT
        if self._action == "MISSED_CALL":
            self._logger.info("The call will be initiated by the Main DUT")
            self._caller_phone = self._voice_call_api
            # Set the phone number used for dialing
            self._phone_number = self._phone_number2
            self._receiver_phone = self._voice_call_api2
            # Attempt voice call
            self._dial_number()
            (verdict, message) = self._missed_call()
            if verdict == Global.FAILURE:
                return verdict, message

        # The DUT is busy
        if self._action == "BUSY_CALL":
            self._logger.info("The call will be initiated by the Main DUT")
            self._caller_phone = self._voice_call_api
            # Set the phone number used for dialing
            self._phone_number = self._phone_number2
            self._receiver_phone = self._voice_call_api2
            # Attempt voice call
            self._dial_number()
            # Initialize verdict variables
            verdict = Global.SUCCESS
            message = "No error."
            (verdict, message)= self._busy_call()
            if verdict == Global.FAILURE:
                return verdict, message

        # The DUT is not answering
        if self._action == "NO_ANS":
            self._logger.info("The call will be initiated by the Main DUT")
            self._caller_phone = self._voice_call_api
            # Set the phone number used for dialing
            self._phone_number = self._phone_number2
            self._receiver_phone = self._voice_call_api2
            # Attempt voice call
            self._dial_number()
            # Initialize verdict variables
            verdict = Global.SUCCESS
            message = "No error."
            (verdict, message)= self._no_answer()
            if verdict == Global.FAILURE:
                return verdict, message

        # Phone1 & 2 : Check call is idle
        self._voice_call_api.wait_for_state(
            self._uecmd_types.VOICE_CALL_STATE.NOCALL,
            self._call_setup_time)
        self._voice_call_api2.wait_for_state(
            self._uecmd_types.VOICE_CALL_STATE.NOCALL,
            self._call_setup_time)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        Disposes this test.
        """
        # Call the Base Class tear_down method
        UseCaseBase.tear_down(self)

        # "tear down" log message from the framework
        self._logger.info("Tear down: starting tear down operations.")
        # Release the call for both phones (Robustness only)
        self._voice_call_api.release()
        self._voice_call_api2.release()

        # End of tear_down
        return (Global.SUCCESS, "No errors")

    def _dial_number(self):
        """
        Dial the number and checks the expected call statuses.
        """
        # Initiating call
        self._caller_phone.dial(self._phone_number, False)

        # Waiting for incoming call before callSetupTimeout (in seconds)
        time.sleep(self._wait_btwn_cmd)
        self._receiver_phone.wait_for_state(
            self._uecmd_types.VOICE_CALL_STATE.INCOMING,
            self._call_setup_time)

    def _answer_call(self):
        """
        Answer the incoming call and checks the expected call statuses.
        """
        # Answer call
        self._receiver_phone.answer()

        # Check voice call is active
        self._receiver_phone.wait_for_state(
            self._uecmd_types.VOICE_CALL_STATE.ACTIVE,
            self._call_setup_time)

        # WAIT FOR CALL DURATION
        self._logger.info(
            "Wait for call duration: " + str(self._callduration) + "s...")
        time.sleep(self._callduration)

        # Check call is still active
        self._voice_call_api.check_state(
            self._uecmd_types.VOICE_CALL_STATE.ACTIVE)

    def _release_call(self):
        """
        Release any active call.
        """

        # RELEASE THE CALL
        self._releaser_phone.release()

    def _missed_call(self):

        """
        Check the scenarios related missed call event
        for the DUT.

        :type action: str
        :param action: action for the remote party
        :rtype: int, str
        :return: the verdict and the message
        """

        # In case the remote party is not answering
        self._logger.info("%s - %s %d %s" % (
        "The remote party will not answer the call",
        "wait",
        self._callduration,
        "seconds"))
        # Sleep for ringing time duration
        time.sleep(self._callduration)

        # Check call state for PHONE1 is still active
        # Check call state for PHONE2 is still INCOMING (ringing)
        # Phone1 & 2 : Check call is idle
        self._voice_call_api.wait_for_state(
            self._uecmd_types.VOICE_CALL_STATE.ACTIVE,
            self._call_setup_time)
        self._voice_call_api2.wait_for_state(
            self._uecmd_types.VOICE_CALL_STATE.INCOMING,
            self._call_setup_time)

        # From main DUT =  PHONE 1 -> Mobile Release
        self._logger.info("The call will be released as DUT didn't answer")
        self._voice_call_api.release()

        (number, call_type, sim) = self._voice_call_api2.get_last_call_details()
        # pylint: disable=E1101
        # Because pylint can not resolve enum.
        # Checking last call is an outgoing one.
        if str(call_type) != str(self._uecmd_types.VOICE_CALL_TYPE.MISSED):
        # pylint: enable=E1101
            return (Global.FAILURE,
                "The last call is not a MISSED call one. Call type is: %s"
                % call_type)
        else:
            return Global.SUCCESS, "No errors"

    def _busy_call(self):

        """
        Check the scenarios related DUT is BUSY.

        :type action: str
        :param action: BUSY
        :rtype: int, str
        :return: the verdict and the message
        """

        # Initialize the verdict as passed
        verdict = Global.SUCCESS
        message = "No Error"

        # In case the remote party is busy, it will reject the call
        self._logger.info("The remote party is busy - the call will be rejected")
        # Release the call from PHONE2
        self._voice_call_api2.release()
        try:
            # Check call state for PHONE1
            self._logger.debug("NOCALL state is checked")
            self._voice_call_api.wait_for_state(
                self._uecmd_types.VOICE_CALL_STATE.NOCALL,
                self._call_setup_time)

        except DeviceException as device_ex:
            # End the call from PHONE1
            self._voice_call_api.release()

            # Set the verdict as failure
            message = "Error after remote party has rejected the call, " \
                "exception caught: %s" % str(device_ex)
            self._logger.error(message)
            verdict = Global.FAILURE

        # Return verdict
        return (verdict, message)

    def _no_answer(self):

        """
        Check the scenarios when DUT is not answering.

        :type action: str
        :param action: action for DUT
        :rtype: int, str
        :return: the verdict and the message
        """

        # Initialize the verdict as passed
        verdict = Global.SUCCESS
        message = "No Error"

        # In case the DUT is not answering
        self._logger.info("%s - %s %d %s" % (
            "The DUT will not answer the call",
            "wait",
            self._callduration,
            "seconds"))

        # Sleep for ringing time duration
        time.sleep(self._callduration)

        try:
            # Check call state for PHONE1 is still active
            # Check call state for PHONE2 is still INCOMING (ringing)

            self.__check_call_state_both_phones(
                self._uecmd_types.VOICE_CALL_STATE.ACTIVE,
                self._uecmd_types.VOICE_CALL_STATE.INCOMING)

            # From main DUT =  PHONE 1 -> Mobile Release
            self._logger.info("The call will be released as DUT didn't answer")
            self._voice_call_api.release()

        except DeviceException as device_ex:
            # End the call from PHONE1
            self._voice_call_api.release()

            # Set the verdict as failure
            message = "Error after main DUT releases the call, remote party didn't answer, " \
                "exception caught: %s" % str(device_ex)
            self._logger.error(message)
            verdict = Global.FAILURE

        # Return verdict
        return (verdict, message)

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