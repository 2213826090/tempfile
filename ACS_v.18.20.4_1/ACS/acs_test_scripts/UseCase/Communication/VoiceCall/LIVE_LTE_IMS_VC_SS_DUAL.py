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
:summary:  Use case to validate LTE IMS Voice Calls with Supplementary Services using two phones.
:since: 23/04/2015
:author: mariussx
"""

import time

from UseCase.Communication.VoiceCall.LIVE_LTE_IMS_VC_DUAL_BASE import LiveLteImsVcDualBase
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.Utilities.RegistrationUtilities import ImsRegistrationStatus
from acs_test_scripts.Utilities.LogcatParserUtilities import LogcatParserUtils, AtCmdParser

from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException

class LiveLteImsVcSSDual(LiveLteImsVcDualBase):

    """
    Test of Supplementary Services on IMS over LTE with two devices.

    This test case requires a Com4Innov cell to work.
    Because of this constraint some values that should
    have been given as parameters are actually hard-coded.
    That is to be changed if this test ever has to be
    executed in a different environment.
    """

    SS_FEATURES = {
        "CALL_FORWARDING": ["UNCONDITIONAL", "BUSY", "NO_ANSWER", "NOT_REACHABLE"],
        "CALL_BARRING": ["BAOC", "BOIC", "BAIC"],
        "CALL_WAITING": None}
    """
    Modes for each Supplementary Service
    """

    STATE_ENABLED = "enabled"
    """
    Supplementary Service status in case is activated
    """

    STATE_DISABLED = "disabled"
    """
    Supplementary Service status in case is deactivated
    """

    CALL_BARRING_AT_COMMAND = "AT+CLCK"
    """
    AT command which modem will receive for activation/de-activation of call barring
    """

    def __init__(self, tc_name, global_config):
        """
        Initializes this instance.
        """
        # Call inherited initializer
        LiveLteImsVcDualBase.__init__(self, tc_name, global_config)

        # Read the Supplementary Service parameter
        self._ims_ss_feature = self._tc_parameters.get_param_value("SS_FEATURE")

        # For call barring, retrieve the call barring activation code
        if "CALL_BARRING" in self._ims_ss_feature:
            self._cb_activation_code = self._tc_parameters.get_param_value("CB_ACTIVATION_CODE", "")
        else:
            self._cb_activation_code = None
            self._logcat_extract = None

        # Read the Supplementary Service Mode parameter
        self._ims_ss_mode = self._tc_parameters.get_param_value("SS_MODE")

        # Read the Phone Number for Supplementary Service
        self._ims_ss_phone_number = self._tc_parameters.get_param_value("SS_PHONE_NUMBER")

        # Delay introduced for the Supplementary Service activation or de-activation
        self._wait_ss_response = 30

        # Modify the call setup time in case the Supplementary Service is active
        self._call_setup_time = 30

        # Initialize the instances used for logcat parsing procedure
        self._at_cmd_parser = None
        self.logcat_extras = None
        if self._ims_ss_feature == "CALL_BARRING":
            self._parse_utils = LogcatParserUtils(self._device)
        else:
            self._parse_utils = None

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initializes the test.
        """
        LiveLteImsVcDualBase.set_up(self)

        # Check the parameter with the Supplementary Services feature
        if self._ims_ss_feature is None or \
            self._ims_ss_feature not in LiveLteImsVcSSDual.SS_FEATURES:

            message = "%s '%s' %s '%s'" % (
                "Invalid parameter value (or missing parameter)",
                str(self._ims_ss_feature),
                "for parameter",
                "SS_FEATURE")
            raise AcsConfigException(
                AcsConfigException.INVALID_PARAMETER,
                message)

        # Check the digest password in all cases too because it is used
        # for the XCAP requests
        if not self._digest_password:
            message = "%s %s" % (
                "Invalid parameter value: '%s'." % str(self._digest_password),
                "Parameter IMS_DIGEST_PASSWORD is mandatory for XCAP tests.")
            raise AcsConfigException(
                AcsConfigException.INVALID_PARAMETER,
                message)

        # Check the parameter related with Supplementary Services Mode
        if self._ims_ss_mode is None or \
            str(self._ims_ss_mode) not in LiveLteImsVcSSDual.SS_FEATURES[self._ims_ss_feature]:

            message = "%s '%s' %s '%s'" % (
                "Invalid parameter value (or missing parameter)",
                str(self._ims_ss_mode),
                "for parameter",
                "SS_MODE")
            raise AcsConfigException(
                AcsConfigException.INVALID_PARAMETER,
                message)

        # Use case limitation - only call forwarding and call barring supported for the moment
        if self._ims_ss_feature not in ("CALL_FORWARDING", "CALL_BARRING"):
            message = "For the moment, only '%s' and '%s' is supported by this use case" %("CALL_FORWARDING",
                                                                                           "CALL_BARRING")
            raise AcsConfigException(
                AcsConfigException.FEATURE_NOT_IMPLEMENTED,
                message)

        # Check the phone Number parameter in case of SS check/activation
        if self._ims_ss_phone_number is None:
            message = "%s '%s' %s '%s'" % (
                "Invalid parameter value (or missing parameter)",
                str(self._ims_ss_phone_number),
                "for parameter",
                "SS_PHONE_NUMBER")
            raise AcsConfigException(
                AcsConfigException.INVALID_PARAMETER,
                message)

        # For call barring check that activation code is present in the TC file
        if ("CALL_BARRING" in self._ims_ss_feature) and \
            (self._cb_activation_code is None):

            message = "%s '%s' %s '%s'" % (
                "Invalid parameter value (or missing parameter)",
                str(self._cb_activation_code),
                "for parameter",
                "CB_ACTIVATION_CODE")

        # Specific set up for XCAP
        self._logger.info("Applying configuration for XCAP")
        if self._ims_configure_modem_procedure == "IMS_ANDROID":
            self._apply_configurations("xcap")
        else:
            self._apply_configurations("xcap-at")

        # Force the reading of XCAP parameters from the modem NVM.
        self._logger.info("Flight mode ON/OFF cycle in order to take XCAP parameters into account.")
        self._networking_api.set_flight_mode("on")
        # Wait arbitrary time
        time.sleep(30)
        self._networking_api.set_flight_mode("off")
        # Wait arbitrary time
        time.sleep(10)
        # Set up is done correctly
        return (Global.SUCCESS, "No errors")
#------------------------------------------------------------------------------

    def run_test(self):
        """
        Runs the test.
        """
        # Call inherited run_test method which will ensure IMS
        # IMS registration on device 1
        self._logger.info("")
        self._logger.info("Calling inherited run_test step.")
        self._logger.info("")
        LiveLteImsVcDualBase.run_test(self)

        # Initialize verdict variables
        verdict = Global.SUCCESS
        message = "No error."

        # Proceed with the scenario based on the Supplementary service
        self._logger.info("")
        self._logger.info("Executing requested tests.")
        self._logger.info("")
        if self._ims_ss_feature == "CALL_FORWARDING":
            (verdict, message) = self._check_call_forwarding()
        elif self._ims_ss_feature == "CALL_BARRING":
            (verdict, message) = self._perform_call_barring()

        if verdict == Global.FAILURE:
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
        # Initialize some local variables
        message = ""
        verdict = Global.SUCCESS

        # Log a message indicating that we are enterint the tear down step
        self._logger.info("Starting tear down operations")
        # Release the call for secondary DUT (Robustness only)
        try:
            self._voice_call_api2.release()
        except DeviceException as device_ex:
            self._logger.warning("Error in tear down step: %s" % str(device_ex))

        # Supplementary Service de-activation
        try:
            (verdict, message) = self._deactivate_ss_feature()
        except DeviceException as device_ex:
            self._logger.warning("Error in tear down step: %s" % str(device_ex))

        # Delete the logcat file
        if self._ims_ss_feature == "CALL_BARRING":
            try:
                self._parse_utils.delete_logcat_file()
            except AcsConfigException as excp:
                self._logger.debug(excp)

        # Call the inherited tear_down method
        LiveLteImsVcDualBase.tear_down(self)

        # End of tear_down
        return (verdict, message)

#------------------------------------------------------------------------------

    def _check_call_forwarding(self): # pylint: disable=too-many-return-statements
        """
        Performs the IMS call forwarding procedure.

        :rtype: int, str
        :return: the verdict and the message
        """

        # Initialize verdict variables
        verdict = Global.SUCCESS
        message = "No error."

        # Check initial call forwarding state
        self._voice_call_api.check_call_forwarding_state(LiveLteImsVcSSDual.STATE_DISABLED)

        # Activate call forwarding on main DUT
        self._phone_system_api.wake_screen()
        self._voice_call_api.enable_call_forwarding(self._ims_ss_mode.lower(),
                                                    self._ims_ss_phone_number)
        # Sleep few seconds for activation of Supplementary Service
        self._logger.info("Sleep for %s seconds" % self._wait_ss_response)
        time.sleep(self._wait_ss_response)

        try:
            # Check the call forwarding state
            self._logger.info("Checking Call Fowarding state after activation.")
            self._voice_call_api.check_call_forwarding_state(LiveLteImsVcSSDual.STATE_ENABLED)
        except DeviceException as device_ex:
            message = "Exception caught: %s" % (str(device_ex))
            self._logger.error(message)
            verdict = Global.FAILURE
            return (verdict, message)

        # Case for 'NOT_REACHABLE' call forwarding
        if self._ims_ss_mode == "NOT_REACHABLE":
            self._logger.info("For 'NOT_REACHABLE' scenario, the main DUT shall be in out of service")
            # Flight mode ON for main DUT
            self._networking_api.set_flight_mode("on")
            # Sleep 30 seconds
            time.sleep(30)
            # Check that main DUT is not registered
            ims_service_status = ImsRegistrationStatus.out_of_service()
            # Log the latest IMS registration status
            registration_status = self._get_registration_status()
            if registration_status != ims_service_status:
                verdict = Global.FAILURE
                message = "IMS is still registered after airplane mode ON."
                self._logger.error(message)
                return(verdict, message)

        # Dial from the secondary DUT
        self._phone_system_api2.wake_screen()
        self._logger.info("Call initiated from secondary DUT")
        self._voice_call_api2.dial(self._phone_number, check_state=False)

        # Depending on the mode, perform action for main DUT
        # after the call was initiated by the secondary DUT
        (verdict, message) = self._main_dut_action_call_fw()
        if verdict != Global.SUCCESS:
            return (verdict, message)

        try:
            # Wait until the call is established for the secondary DUT with the other party
            self._logger.info("Wait until the call is established for the secondary DUT")
            self._voice_call_api2.wait_for_state(
                self._uecmd_types.VOICE_CALL_STATE.ACTIVE,
                self._call_setup_time)

            if self._ims_ss_mode != "NOT_REACHABLE":
                # Check the call state for main DUT - no active call
                self._voice_call_api.check_state(self._uecmd_types.VOICE_CALL_STATE.NOCALL)

        except DeviceException as device_ex:
            # End the call
            self._voice_call_api2.release()
            # Compute the error message
            message = "Could not establish the voice call from secondary DUT within %d" \
                "exception caught: %s" % (self._call_setup_time, str(device_ex))
            self._logger.error(message)
            verdict = Global.FAILURE
            return (verdict, message)

        # Sleep 30 seconds
        self._logger.info("Wait 30 seconds with call in active state")
        time.sleep(30)

        try:
            if self._ims_ss_mode != "NOT_REACHABLE":
                # Check call state for PHONE1 is still NOCALL
                # Check call state for PHONE2 is still ACTIVE
                self.__check_call_state_both_phones(self._uecmd_types.VOICE_CALL_STATE.NOCALL,
                                                    self._uecmd_types.VOICE_CALL_STATE.ACTIVE)
            else:
                # For "NOT_REACHABLE" scenario, check the call state only for secondary DUT
                self._voice_call_api.check_state(self._uecmd_types.VOICE_CALL_STATE.ACTIVE)

        except DeviceException as device_ex:
            # Compute the error message
            message = "Problem encountered after the call was established for the secondary DUT" \
                "exception caught: %s" % str(device_ex)
            self._logger.error(message)
            verdict = Global.FAILURE
            return (verdict, message)

        # Release the call for the secondary DUT
        self._voice_call_api2.release()

        if self._ims_ss_mode != "NOT_REACHABLE":
            # Flight mode off for main DUT
            self._networking_api.set_flight_mode("off")
            # Sleep 30 seconds
            time.sleep(30)
            # Check the registration status for IMS, should be "IN_SERVICE"
            ims_service_status = ImsRegistrationStatus.in_service()
            self._networking_api.check_ims_registration_before_timeout(
                self._ims_registration_timeout)
            # Log the latest IMS registration status
            registration_status = self._get_registration_status()
            if registration_status != ims_service_status:
                verdict = Global.FAILURE
                message = "IMS is not registered after registration ON for the main DUT."
                self._logger.error(message)
                return (verdict, message)

        # De-activate the call forwarding feature
        self._logger.info("Call forwarding de-activation")
        self._voice_call_api.disable_call_forwarding(self._ims_ss_mode.lower())

        # Final return
        return (verdict, message)

#------------------------------------------------------------------------------

    def _deactivate_ss_feature(self):
        """
        Supplementary Service de-activation

        :rtype: int, str
        :return: the verdict and the message
        """

        # Initialize the verdict
        verdict = Global.SUCCESS
        message = "No error."

        # For 'CALL_FORWARDING' scenario
        if self._ims_ss_feature == "CALL_FORWARDING":
            # For 'NOT_REACHABLE' case perform the re-registration scenario upon flight mode off
            if self._ims_ss_mode != "NOT_REACHABLE":
                # Flight mode off for main DUT
                self._networking_api.set_flight_mode("off")
                # Sleep 30 seconds
                time.sleep(30)
                # Check the registration status for IMS, should be "IN_SERVICE"
                ims_service_status = ImsRegistrationStatus.in_service()
                self._networking_api.check_ims_registration_before_timeout(
                    self._ims_registration_timeout)
                # Check the IMS registration status
                registration_status = self._get_registration_status()
                if registration_status != ims_service_status:
                    verdict = Global.FAILURE
                    message = "IMS is not registered after registration ON for main DUT"
                    self._logger.error(message)
                    return (verdict, message)

            # De-activate the call forwarding on main DUT
            self._phone_system_api.wake_screen()
            self._voice_call_api.disable_call_forwarding(self._ims_ss_mode.lower())
            # Sleep few seconds for activation of Supplementary Service
            self._logger.info("Sleep for %s seconds" % self._wait_ss_response)
            time.sleep(self._wait_ss_response)

            try:
                # Check the call forwarding state
                self._voice_call_api.check_call_forwarding_state(LiveLteImsVcSSDual.STATE_DISABLED)
            except DeviceException as device_ex:
                message = "Exception caught: %s" % (str(device_ex))
                self._logger.error(message)
                verdict = Global.FAILURE

        # For 'CALL_BARRING' scenario
        elif self._ims_ss_feature == "CALL_BARRING":
            # Disable the call barring
            self._voice_call_api.disable_call_barring(
                self._ims_ss_mode.lower(),
                self._cb_activation_code)
            # Sleep few seconds for activation of Supplementary Service
            self._logger.info("Sleep for %s seconds" % self._wait_ss_response)
            time.sleep(self._wait_ss_response)

        return (verdict, message)

#------------------------------------------------------------------------------

    def _main_dut_action_call_fw(self):
        """
        Depending on the call forwarding mode, perform the necessary actions for the main DUT

        :rtype: int, str
        :return: the verdict and the message
        """

        # Initialize the verdict
        verdict = Global.SUCCESS
        message = "No error"

        # Case for 'UNCONDITIONAL' call forwarding
        if self._ims_ss_mode == "UNCONDITIONAL":
            self._logger.info(
                "Unconditional forwarding, secondary DUT will be redirected directly to %s" % \
                    self._ims_ss_phone_number)

        # Case for 'BUSY' call forwarding
        if self._ims_ss_mode == "BUSY":
            self._logger.info("For 'BUSY' scenario, the main DUT will reject the incoming call")
            try:
                # Wait for incoming call from the main DUT
                self._voice_call_api.wait_for_state(
                    self._uecmd_types.VOICE_CALL_STATE.ACTIVE,
                    self._call_setup_time)
                self._logger.info(
                    "The main DUT rejects the call, DUT will be redirected to %s." % \
                        self._ims_ss_phone_number)
                # Release the call from PHONE1
                self._voice_call_api.release()

            except DeviceException as device_ex:
                # Compute the error message
                message = "No incoming call for the main DUT," \
                    "exception caught: %s" % str(device_ex)
                self._logger.error(message)
                verdict = Global.FAILURE
                return (verdict, message)

        # Case for 'NO ANSWER' call forwarding
        if self._ims_ss_mode == "NO_ANSWER":
            self._logger.info("For 'NO_ANSWER' scenario, the main DUT will not answer to the incoming call")
            try:
                # Wait for incoming call from the main DUT
                self._voice_call_api.wait_for_state(
                    self._uecmd_types.VOICE_CALL_STATE.ACTIVE,
                    self._call_setup_time)
            except DeviceException as device_ex:
                # Compute the error message
                message = "No incoming call for the main DUT," \
                    "exception caught: %s" % str(device_ex)
                self._logger.error(message)
                verdict = Global.FAILURE
                return (verdict, message)

            # Sleep 30 seconds
            self._logger.info(
                "Incoming call - wait for 30s in order to be redirected to %s." % \
                    self._ims_ss_phone_number)
            time.sleep(30)

        return (verdict, message)

#------------------------------------------------------------------------------

    def __check_call_state_both_phones(self,
                                       call_state_main_dut,
                                       call_state_secondary_dut):
        """
        Checks the call state for both DUTs

        :type call_state_main_dut: str
        :param call_state_main_dut: State for main DUT
        :type call_state_secondary_dut: str
        :param call_state_secondary_dut: State for secondary DUT
        """

        # Check call state for PHONE1
        self._voice_call_api.check_state(call_state_main_dut)
        # Check call state for PHONE2
        self._voice_call_api2.check_state(call_state_secondary_dut)

#------------------------------------------------------------------------------

    def _perform_call_barring(self):
        """
        Performs the IMS call barring procedure.

        :rtype: int, str
        :return: the verdict and the message
        """

        # Clear the logcat
        # (used for checking if barring procedure was successfully activated/deactivated)
        self._parse_utils.clear_logcat("radio")

        # Activate call barring on main DUT
        self._phone_system_api.wake_screen()
        self._voice_call_api.enable_call_barring(self._ims_ss_mode.lower(),
                                                 self._cb_activation_code)
        # Sleep few seconds for activation of Supplementary Service
        self._logger.info("Sleep for %s seconds" % self._wait_ss_response)
        time.sleep(self._wait_ss_response)

        # Check call barring for enabled case
        (verdict, message) = self._check_call_barring(LiveLteImsVcSSDual.STATE_ENABLED)
        # In case of failure
        if verdict == Global.FAILURE:
            return (verdict, message)

        # Clear the logcat
        # (used for checking if barring procedure was successfully activated/deactivated)
        self._parse_utils.clear_logcat("radio")

        # Disable the call barring
        self._voice_call_api.disable_call_barring(
            self._ims_ss_mode.lower(),
            self._cb_activation_code)
        # Sleep few seconds for activation of Supplementary Service
        self._logger.info("Sleep for %s seconds" % self._wait_ss_response)
        time.sleep(self._wait_ss_response)

        # Check call barring for disabled case
        (verdict, message) = self._check_call_barring(LiveLteImsVcSSDual.STATE_DISABLED)

        # Return verdict
        return (verdict, message)

#------------------------------------------------------------------------------

    def _check_call_barring(self,
                            call_barring_state):
        """
        Checks call barring depending on its state.

        :type call_barring_state: str
        :param call_barring_state: State for call barring

        :rtype: int, str
        :return: the verdict and the message
        """

        # Initialize verdict variables
        verdict = Global.SUCCESS
        message = "No error."

        if self._ims_configure_modem_procedure == "IMS_ANDROID":
            at_command_expected_result = "OK"
            # Store the logcat in a file
            self.logcat_extras = self._parse_utils.retrive_logcat("radio")
            self._at_cmd_parser = AtCmdParser(self.logcat_extras)

            # Check that call barring was successfully activated/deactivated based on AT command response
            (verdict, output) = \
                self._at_cmd_parser.check_at_command_response(
                    LiveLteImsVcSSDual.CALL_BARRING_AT_COMMAND,
                    at_command_expected_result)
            if verdict == Global.FAILURE:
                return (verdict, output)

        # Case for barring all outgoing calls
        if self._ims_ss_mode == "BAOC":
            self._logger.info("For 'barring all outgoing calls' scenario, \
                                the main DUT will initiate a call which shall be rejected")
            # Dial from the main DUT
            self._phone_system_api.wake_screen()
            self._logger.info("Calling the secondary DUT...")
            self._voice_call_api.dial(self._phone_no_secondary_dut)

        # Case for barring all outgoing international calls
        if self._ims_ss_mode == "BOIC":
            self._logger.info("For 'barring all outgoing international calls' scenario, \
                                the main DUT will initiate a call to an international number which shall be rejected")
            # Dial from the main DUT
            self._phone_system_api.wake_screen()
            self._logger.info("Call to an international phone number...")
            self._voice_call_api.dial(self._ims_ss_phone_number)

        # Case for barring all incoming calls
        if self._ims_ss_mode == "BAIC":
            self._logger.info("For 'barring all incoming calls' scenario, \
                                the secondary DUT will initiate a call to main DUT")

            # Dial from the main DUT
            self._phone_system_api2.wake_screen()
            self._logger.info("Calling the main DUT...")
            self._voice_call_api2.dial(self._phone_no_main_dut)

        try:
            # For call barring enabled
            if call_barring_state is LiveLteImsVcSSDual.STATE_ENABLED:
                # Check that call was not established for main DUT
                self._voice_call_api.wait_for_state(
                    self._uecmd_types.VOICE_CALL_STATE.NOCALL,
                    self._call_setup_time)

            # For call barring disabled
            else:
                # Check that call is successfully established for main DUT
                self._logger.info("Waiting for call to be establish")
                self._voice_call_api.wait_for_state(
                    self._uecmd_types.VOICE_CALL_STATE.ACTIVE,
                    self._call_setup_time)

                # Wait 15 seconds
                self._logger.info("Waiting 15 seconds")
                time.sleep(15)

                # Check the call state
                if self._ims_ss_mode != "BOIC":
                    self.__check_call_state_both_phones(self._uecmd_types.VOICE_CALL_STATE.ACTIVE,
                                                        self._uecmd_types.VOICE_CALL_STATE.ACTIVE)
                else:
                    self._voice_call_api.check_state(self._uecmd_types.VOICE_CALL_STATE.ACTIVE)

                # Release the call from main DUT
                self._voice_call_api.release()

        except DeviceException as device_ex:
            # For call barring enabled
            if call_barring_state is LiveLteImsVcSSDual.STATE_ENABLED:
                # Compute the error message
                message = "The initiated call was not barred" \
                    "exception caught: %s" % str(device_ex)
                self._logger.error(message)
                verdict = Global.FAILURE
                # Release the call from PHONE1
                self._voice_call_api.release()

                if self._ims_ss_mode == "BAIC":
                    # Release the call from PHONE2
                    self._voice_call_api2.release()

            # For call barring disabled
            else:
                # Compute the error message
                message = "The initiated call was not established" \
                    "exception caught: %s" % str(device_ex)
                self._logger.error(message)
                verdict = Global.FAILURE
                # Robustness call release
                self._voice_call_api.release()
                self._voice_call_api2.release()

        # Return verdict
        return (verdict, message)

