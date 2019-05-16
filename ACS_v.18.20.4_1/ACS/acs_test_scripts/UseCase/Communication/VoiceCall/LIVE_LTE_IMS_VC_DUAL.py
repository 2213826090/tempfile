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
:summary:  Use case to validate LTE IMS MO/MT Voice Calls using two phones.
:since: 02/12/2014
:author: mariussx
"""

import time

from UseCase.Communication.VoiceCall.LIVE_LTE_IMS_VC_DUAL_BASE import LiveLteImsVcDualBase
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.Utilities.RegistrationUtilities import ImsRegistrationStatus
from acs_test_scripts.Utilities.FtpUtilities import perform_ftp_transfer
from ErrorHandling.DeviceException import DeviceException

class LiveLteImsVcDual(LiveLteImsVcDualBase):

    """
    Test of Live Voice Call on IMS over LTE with two devices.

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
        LiveLteImsVcDualBase.__init__(self, tc_name, global_config)
        # Retrieve ringing time parameter
        self._ringing_time = \
            int(self._tc_parameters.get_param_value("RINGING_TIME", 15))


#------------------------------------------------------------------------------

    def run_test(self):
        """
        Runs the test.
        """

        # Call inherited run_test method which will ensure IMS
        # IMS registration on device 1
        LiveLteImsVcDualBase.run_test(self)

        # Perform the IMS call procedure
        (verdict, message) = self._perform_ims_call(self._call_direction)

        # Because in case of failure the IMS stack may have crashed
        # we need to double-check the IMS registration status.
        in_service_status = ImsRegistrationStatus.in_service()
        registration_status = self._get_registration_status()
        self._logger.info("Registration status: %s (%d)" % (
            str(registration_status),
            registration_status.as_int()))
        # Compute the verdict
        if registration_status != in_service_status:
            message = "IMS stack has crashed or IMS registration is lost."
            self._logger.error(message)
            verdict = Global.FAILURE

        # Return the test result
        return (verdict, message)

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        Disposes this test.
        """
        # Write informative log message because some logs may appear
        # before the "tear down" log message from the framwork
        self._logger.info("Tear down: starting tear down operations.")
        # Release the call for both phones (Robustness only)
        self._voice_call_api.release()
        self._voice_call_api2.release()

        # Call the inherited tear_down method
        LiveLteImsVcDualBase.tear_down(self)

        # End of tear_down
        return (Global.SUCCESS, "No errors")

#------------------------------------------------------------------------------

    def _perform_ims_call(self, direction):
        """
        Performs the IMS call procedure depending on the 'direction' parameter
            MO -> the main DUT is initiating a call,
                  the secondary phone is answering
            MT -> the secondary phone is initiating a call and
                  the main DUT shall answer
        Call states are checked on both phones.
        The call will be released by the main DUT.

        :type direction: str
        :param direction: call direction: MO or MT
        :rtype: int, str
        :return: the verdict and the message
        """

        # Initialize verdict variables
        verdict = Global.SUCCESS
        message = "No error."

        if "MO" in direction:
            self._logger.info("An IMS call will be initiated by the main DUT")
            main_phone_system_api = self._phone_system_api
            main_voice_call_api = self._voice_call_api
            secondary_phone_system_api = self._phone_system_api2
            secondary_voice_call_api = self._voice_call_api2

            # Set the phone number used for dialing
            if self._phone_number is None:
                self._phone_number = self._phone_no_secondary_dut

        else:
            self._logger.info("An IMS call will be initiated by the secondary DUT")
            main_phone_system_api = self._phone_system_api2
            main_voice_call_api = self._voice_call_api2
            secondary_phone_system_api = self._phone_system_api
            secondary_voice_call_api = self._voice_call_api

            # Set the phone number used for dialing
            if self._phone_number is None:
                self._phone_number = self._phone_no_main_dut

        # Attempt an IMS voice call
        main_phone_system_api.wake_screen()
        secondary_phone_system_api.wake_screen()
        main_voice_call_api.dial(self._phone_number, check_state=False, single_dial=False, call_type=self._call_type)

        try:
            # Check voice call is in ACTIVE phase for the DUT which initiate the call
            self._logger.info("Waiting for ACTIVE state")

            main_voice_call_api.wait_for_state(
                self._uecmd_types.VOICE_CALL_STATE.ACTIVE,
                self._call_setup_time)

        except DeviceException as device_ex:
            # Compute the error message
            message = "Could not establish Voice Call, " \
                "exception caught: %s" % str(device_ex)
            self._logger.error(message)
            verdict = Global.FAILURE
            # End the call
            main_voice_call_api.release()

        # Proceed with the following test steps only if the
        # previous one succeeded.
        # Use a protected block here because we do not want
        # the Use Case to break in case of failure, as we
        # have some additional checks to perform.
        if verdict != Global.FAILURE:
            # Wait 5 seconds in ALERTING phase
            time.sleep(5)

            # Actions for the remote party
            if self._remote_party_action != "":
                (verdict, message) = \
                    self._perform_remote_party_operation(self._remote_party_action)
                # Return verdict
                return (verdict, message)

            # Answer the call
            if self._call_type == "IR92":
                secondary_voice_call_api.answer()
            else:
                # IR94 case
                secondary_voice_call_api.answer_ims_video_call()

            try:
                # Check voice call is active for both phones
                main_voice_call_api.wait_for_state(
                    self._uecmd_types.VOICE_CALL_STATE.ACTIVE,
                    self._call_setup_time)
                secondary_voice_call_api.wait_for_state(
                    self._uecmd_types.VOICE_CALL_STATE.ACTIVE,
                    self._call_setup_time)

                # Perform FTP data transfer if requested
                if self._ftp_transfer is True:
                    (status, msg) = self._perform_ftp_data_transfer()

                    # if failure during FTP data transfer
                    if status == Global.FAILURE:
                        # Release the call
                        main_voice_call_api.release()
                        secondary_voice_call_api.release()

                        # Set verdict as failed
                        message = "Error during FTP data transfer: " + msg
                        verdict = Global.FAILURE
                        return (verdict, message)

                # Wait for call duration
                self._logger.info(
                    "Wait for call duration: %s s..." % str(self._callduration))
                time.sleep(self._callduration)

                # Perform HOLD/RESUME procedure if requested
                if self._check_on_hold_resume_procedure is True:
                    try:
                        self._perform_hold_resume_procedure()

                    except DeviceException as device_ex:
                        message = "Error during call hold/resume procedures, " \
                            "exception caught: %s" % str(device_ex)
                        self._logger.error(message)
                        verdict = Global.FAILURE

                        # Return failure verdict
                        return (verdict, message)

                # Check call is still active for both phones
                self.__check_call_state_both_phones(self._uecmd_types.VOICE_CALL_STATE.ACTIVE,
                                                    self._uecmd_types.VOICE_CALL_STATE.ACTIVE)

            except DeviceException as device_ex:
                message = "Error during Voice Call, " \
                    "exception caught: %s" % str(device_ex)
                self._logger.error(message)
                verdict = Global.FAILURE


            try:
                self._perform_release_calls()

            except DeviceException as device_ex:
                message = "Error after the Voice Call was released, " \
                    "exception caught: %s" % str(device_ex)
                self._logger.error(message)
                verdict = Global.FAILURE

        return (verdict, message)

    def _perform_remote_party_operation(self, action):
        """
        Check the scenarios related with the actions for the
        remote party.

        :type action: str
        :param action: action for the remote party
        :rtype: int, str
        :return: the verdict and the message
        """

        # Initialize the verdict as passed
        verdict = Global.SUCCESS
        message = "No Error"

        # In case the remote party is busy, it will reject the call
        if "BUSY" in action:
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

        # In case the remote party is not answering
        if "NO_ANSWER" in action:
            self._logger.info("%s - %s %d %s" % (
                "The remote party will not answer the call",
                "wait",
                self._ringing_time,
                "seconds"))
            # Sleep for ringing time duration
            time.sleep(self._ringing_time)
            try:
                # Check call state for PHONE1 is still active
                # Check call state for PHONE2 is still INCOMING (ringing)
                self.__check_call_state_both_phones(
                    self._uecmd_types.VOICE_CALL_STATE.ACTIVE,
                    self._uecmd_types.VOICE_CALL_STATE.INCOMING)

                # From main DUT =  PHONE 1 -> Mobile Release
                self._logger.info("The call will be released as remote party didn't answer")
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

    def _perform_release_calls(self):
        """
        Releases the calls and checks the expected call statuses.
        :raise DeviceException: in case of failure
        """

        # Release the IMS call
        if "MR" in self._call_release_direction:
            # From main DUT =  PHONE 1 -> Mobile Release
            self._logger.info("Mobile Release")
            self._voice_call_api.release()
        else:
            # Release the call from PHONE 2
            # For PHONE 1 will be a Network Release
            self._logger.info("Network Release")
            self._voice_call_api2.release()

        # Wait 2 seconds
        time.sleep(2)

        self._voice_call_api.wait_for_state(
            self._uecmd_types.VOICE_CALL_STATE.NOCALL,
            self._call_setup_time)

        self._voice_call_api2.wait_for_state(
            self._uecmd_types.VOICE_CALL_STATE.NOCALL,
            self._call_setup_time)

    def _perform_hold_resume_procedure(self):
        """
        Performs the IMS call on hold and resume procedure for main DUT
        Call state is checked

        :rtype: none
        :return: none
        """

        # Set on HOLD state
        self._logger.info("Put the active call on hold and wait 10 seconds")
        self._voice_call_api.switch_holding_and_active()
        # Sleep 10 seconds
        time.sleep(10)

        # Check the call status
        self._logger.info("Check the call status for both phones")
        self.__check_call_state_both_phones(self._uecmd_types.VOICE_CALL_STATE.ON_HOLD,
                                            self._uecmd_types.VOICE_CALL_STATE.ON_HOLD)

        # Resume the call and wait 10 seconds
        self._logger.info("Resume the call and wait 5 seconds")
        self._voice_call_api.switch_holding_and_active()
        # Sleep 5 seconds
        time.sleep(5)

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

    def _perform_ftp_data_transfer(self):
        """
        Performs FTP data transfer
        """
        # Set the throughput targets to 1 kbps - not important
        target_tput = 1
        target_dl_tput = 1

        # Wait 10 seconds
        time.sleep(10)
        # Perform FTP data transfer
        (status, msg) = perform_ftp_transfer(self._ftp_direction,
                                                self._ip_address,
                                                self._username,
                                                self._password,
                                                self._ftp_filename,
                                                self._xfer_timeout,
                                                self._device.multimedia_path,
                                                None,
                                                self._ftp_api,
                                                target_tput,
                                                self._logger,
                                                self._dl_ftp_filename,
                                                target_dl_tput,
                                                self._device.binaries_path)

        return (status, msg)
