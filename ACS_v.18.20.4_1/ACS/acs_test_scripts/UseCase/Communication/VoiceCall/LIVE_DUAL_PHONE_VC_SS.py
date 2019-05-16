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
:summary: Use case to validate Voice Calls with Supplementary Services using two phones
:since: 29/04/2016
:author: sahoosax/nowelchx
"""
from LIVE_DUAL_PHONE_VC_BASE import LiveDualPhoneVcBase
import time
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException
from uiautomator import Device
from Device.DeviceManager import DeviceManager

class LiveDualPhoneVcSs(LiveDualPhoneVcBase):

    """
    Test of Supplementary Services with two devices.
    This test case requires a Two Phone setup.
    This test case tests the Call Forwarding and the Call waiting feature.
    """

    SS_FEATURES = {
        "CALL_FORWARDING": ["UNCONDITIONAL", "BUSY", "NO_ANSWER", "NOT_REACHABLE"],
        "CALL_WAITING":None,"CLIR": None,"CLIR_ANSWER":None}
    """
    Modes for each Supplementary Service
    """

    STATE_ENABLED = "1"
    """
    Supplementary Service status in case is activated
    """

    STATE_DISABLED = "0"
    """
    Supplementary Service status in case is deactivated
    """

    def __init__(self, tc_name, global_config):

        """
        Constructor
        """
        LiveDualPhoneVcBase.__init__(self, tc_name, global_config)

        # Get phone number of the DUT
        self._phone_number = str(self._device.get_phone_number())

        self._phone_serial = DeviceManager().get_device("PHONE1")
        self._phone_serial_number = str(self._phone_serial.get_serial_number())
        self.d = Device(self._phone_serial_number)
        self._phone2_serial = DeviceManager().get_device("PHONE2")
        self._phone2_serial_number = str(self._phone2_serial.get_serial_number())
        self.d2 = Device(self._phone2_serial_number)

        # Get phone number of the reference phone
        if self._phone2 is not None:
            self._phone_number2 = str(self._phone2.get_phone_number())
        # Read the Supplementary Service parameter
        self._ss_feature = self._tc_parameters.get_param_value("SS_FEATURE")
        # Read the Supplementary Service Mode parameter
        self._ss_mode = self._tc_parameters.get_param_value("SS_MODE")

        # Read the Phone Number for Supplementary Service
        self._ss_phone_number = self._tc_parameters.get_param_value("SS_PHONE_NUMBER")

        # Delay introduced for the Supplementary Service activation or de-activation
        self._wait_ss_response = 30
        # Modify the call setup time in case the Supplementary Service is active
        self._call_setup_time = 15

        self._test_timeout=10

        # Retrieve phone system APIs
        self._phone_system_api = self._device.get_uecmd("PhoneSystem")
        self._phone_system_api2 = self._phone2.get_uecmd("PhoneSystem")
#---------------------------------------------------------------------------------------------------------
    def set_up(self):
        """
        Validating the test parameters.
        """
        # Check the parameter with the Supplementary Services feature
        if self._ss_feature is None or \
            self._ss_feature not in LiveDualPhoneVcSs.SS_FEATURES:
            message = "%s '%s' %s '%s'" % (
                "Invalid parameter value (or missing parameter)",
                str(self._ss_feature),
                "for parameter",
                "SS_FEATURE")
            raise AcsConfigException(
                AcsConfigException.INVALID_PARAMETER,
                message)

        # Check the SS_MODE parameter
        if not self._ss_mode:
            message = "%s '%s' %s '%s'" % (
                "Invalid parameter value (or missing parameter)",
                str(self._ss_mode),
                "for parameter",
                "SS_MODE")
            raise AcsConfigException(
                AcsConfigException.INVALID_PARAMETER,
                message)

        # Calling Base Class SetUp method
        LiveDualPhoneVcBase.set_up(self)

        # Check the SS_PHONE_NUMBER parameter
        if not self._ss_phone_number:
            message = "%s '%s' %s '%s'" % (
                "Invalid parameter value (or missing parameter)",
                str(self._ss_phone_number),
                "for parameter",
                "SS_PHONE_NUMBER")
            raise AcsConfigException(
                AcsConfigException.INVALID_PARAMETER,
                message)

        # Check the Call Duration parameter
        if not self._callduration:
            message = "%s '%s' %s '%s'" % (
                "Invalid parameter value (or missing parameter)",
                str(self._callduration),
                "for parameter",
                "CALL_DURATION")
            raise AcsConfigException(
                AcsConfigException.INVALID_PARAMETER,
                message)

        return Global.SUCCESS, "No errors"
#-----------------------------------------------------------------------------------------------

    def run_test(self):
        """
        Runs the test.
        """
        # Call inherited run_test method which will ensure
        UseCaseBase.run_test(self)
        #LiveDualPhoneVcBase.run_test(self)

        # Initialize verdict variables
        verdict = Global.SUCCESS
        message = "No error."

        # Proceed with the scenario based on the Supplementary service
        self._logger.info("Executing requested tests.")

        if self._ss_feature == "CALL_FORWARDING":
            (verdict, message) = self._check_call_forwarding()

        if self._ss_feature == "CALL_WAITING":
            (verdict, message) = self._check_call_waiting()

        if self._ss_feature == "CLIR":
            (verdict, message) = self._check_CLIR()

        if self._ss_feature == "CLIR_ANSWER":
            (verdict,message) = self._ans_CLIR()


        if verdict == Global.FAILURE:
            self._logger.error(message)
            verdict = Global.FAILURE

        # Return the test result
        return (verdict, message)
#------------------------------------------------------------------------------------------------

    def tear_down(self):
        """
        Disposes this test.
        """
        # Call the inherited tear_down method
        UseCaseBase.tear_down(self)

        # Initialize some local variables
        message = ""
        verdict = Global.SUCCESS
        #to reset the watcher list
        self.d.watchers.reset()
        # Log a message indicating that LIRwe are enter int the tear down step
        self._logger.info("Starting tear down operations")
        # Release the call for secondary DUT (Robustness only)
        try:
            self._voice_call_api2.release()
            self._voice_call_api.release()
        except DeviceException as device_ex:
            self._logger.warning("Error in tear down step: %s" % str(device_ex))

        # Supplementary Service de-activation
        try:
            (verdict, message) = self._deactivate_ss_feature()
        except DeviceException as device_ex:
            self._logger.warning("Error in tear down step: %s" % str(device_ex))

        # Return the test result
        return (verdict, message)

#-----------------------------------------------------------------------------------------------------

    def _check_call_forwarding(self):
        """
        Performs the Call forwarding procedure.
        :rtype: int, str
        :return: the verdict and the message
        """
        # Initialize verdict variables
        verdict = Global.SUCCESS
        message = "No error."

        # Check initial call forwarding state
        self._voice_call_api.check_call_forwarding_state(LiveDualPhoneVcSs.STATE_DISABLED)

        # Activate call forwarding on main DUT
        self._phone_system_api.wake_screen()
        self._voice_call_api.enable_call_forwarding(self._ss_mode.lower(),
                                                    self._ss_phone_number)
        # Sleep few seconds for activation of Supplementary Service
        self._logger.info("Sleep for %s seconds" % self._wait_ss_response)
        time.sleep(self._wait_ss_response)

        # Checking the call forwarding state
        if self._ss_mode == "UNCONDITIONAL":
            try:
            # Check the call forwarding state
                self._logger.info("Checking Call Forwarding state after activation.")
                self._voice_call_api.check_call_forwarding_state(LiveDualPhoneVcSs.STATE_ENABLED)

            except DeviceException as device_ex:
                message = "Exception caught: %s" % (str(device_ex))
                self._logger.error(message)
                verdict = Global.FAILURE
                return (verdict, message)

        # Case for 'NOT_REACHABLE' call forwarding
        if self._ss_mode == "NOT_REACHABLE":
            self._logger.info("For 'NOT_REACHABLE' scenario, the main DUT shall be in out of service")
            # Flight mode ON for main DUT
            self._networking_api.set_flight_mode("on")
            # Sleep 30 seconds
            time.sleep(self._wait_ss_response)

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

            if self._ss_mode != "NOT_REACHABLE":
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
            if self._ss_mode != "NOT_REACHABLE":
                # Check call state for PHONE1 is still NOCALL
                # Check call state for PHONE2 is still ACTIVE
                self.__check_call_state_both_phones(self._uecmd_types.VOICE_CALL_STATE.NOCALL,
                                                    self._uecmd_types.VOICE_CALL_STATE.ACTIVE)
            else:
                # For "NOT_REACHABLE" scenario, check the call state only for secondary DUT
                self._voice_call_api2.check_state(self._uecmd_types.VOICE_CALL_STATE.ACTIVE)

        except DeviceException as device_ex:
            # Compute the error message
            message = "Problem encountered after the call was established for the secondary DUT" \
                "exception caught: %s" % str(device_ex)
            self._logger.error(message)
            verdict = Global.FAILURE
            return (verdict, message)


        # Release the call for the secondary DUT
        self._voice_call_api2.release()

        if self._ss_mode == "NOT_REACHABLE":
            # Flight mode off for main DUT
            self._networking_api.set_flight_mode("off")
            # Sleep 30 seconds
            time.sleep(self._wait_ss_response)

        # De-activate the call forwarding feature
        self._logger.info("Call forwarding de-activation")
        self._voice_call_api.disable_call_forwarding(self._ss_mode.lower())
        time.sleep(10)

        # Dial from the secondary DUT
        self._phone_system_api2.wake_screen()
        self._logger.info("Call initiated from secondary DUT")
        self._voice_call_api2.dial(self._phone_number, check_state=False)

        # Answer call
        time.sleep(self._call_setup_time)
        self._voice_call_api.answer()

        # Check secondary DUT call state
        self._voice_call_api2.check_state(
            self._uecmd_types.VOICE_CALL_STATE.ACTIVE)

        # Check main DUT call state
        self._voice_call_api.check_state(
            self._uecmd_types.VOICE_CALL_STATE.ACTIVE)

        # Release the call for the secondary DUT
        self._voice_call_api2.release()

        # Final return
        return (verdict, message)

#--------------------------------------------------------------------------------------------------------
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
        if self._ss_mode == "UNCONDITIONAL":
            self._logger.info(
                "Unconditional forwarding, secondary DUT will be redirected directly to %s" % \
                    self._ss_phone_number)

        # Case for 'BUSY' call forwarding
        if self._ss_mode == "BUSY":
            self._logger.info("For 'BUSY' scenario, the main DUT will reject the incoming call")
            try:
                # Wait for incoming call from the main DUT
                self._voice_call_api.wait_for_state(
                    self._uecmd_types.VOICE_CALL_STATE.INCOMING,
                    self._call_setup_time)
                self._logger.info(
                    "The main DUT rejects the call, DUT will be redirected to %s." % \
                        self._ss_phone_number)
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
        if self._ss_mode == "NO_ANSWER":
            self._logger.info("For 'NO_ANSWER' scenario, the main DUT will not answer to the incoming call")
            try:
                # Wait for incoming call from the main DUT
                self._voice_call_api.wait_for_state(
                    self._uecmd_types.VOICE_CALL_STATE.INCOMING,
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
                    self._ss_phone_number)
            time.sleep(30)

        return (verdict, message)

#--------------------------------------------------------------------------------------------------------
    def _check_call_waiting(self):
        """
        Performs the Call waiting procedure.
        :rtype: int, str
        :return: the verdict and the message
        """
        # Initialize verdict variables
        verdict = Global.SUCCESS
        message = "No error."

        # Activate call waiting on main DUT
        self._phone_system_api.wake_screen()
        self._voice_call_api.enable_call_waiting();

        # Sleep few seconds for activation of Supplementary Service
        self._logger.info("Sleep for %s seconds" % self._wait_ss_response)
        time.sleep(self._wait_ss_response)

        # Keep DUT busy by calling VoiceMail number, after enabling call waiting feature
        self._phone_system_api.wake_screen()
        self._voice_call_api.dial(self._ss_phone_number, check_state=False)

        # Sleep time to make the DUT call state "ACTIVE"
        time.sleep(7)

        # Check main DUT is active
        self._voice_call_api.check_state(
            self._uecmd_types.VOICE_CALL_STATE.ACTIVE)

        # Check secondary DUT is Idle
        self._voice_call_api2.check_state(
            self._uecmd_types.VOICE_CALL_STATE.NOCALL)

        # Dial from the secondary DUT
        self._logger.info("Call initiated from Secondary DUT")
        self._voice_call_api2.dial(self._phone_number, check_state=False)

        # Sleep time to make the DUT call state "INCOMING"
        time.sleep(10)

        # Check secondary DUT is ACTIVE
        self._voice_call_api2.check_state(
            self._uecmd_types.VOICE_CALL_STATE.ACTIVE)

        # Check main DUT is "INCOMING" as call-waiting is Enabled
        self._voice_call_api.check_state(
            self._uecmd_types.VOICE_CALL_STATE.INCOMING)

        # Release the call
        self._voice_call_api2.release()
        self._voice_call_api.release()

        # disable the call-waiting feature
        self._logger.info("Disabling Call waiting")
        self._voice_call_api.disable_call_waiting()
        time.sleep(15)

        # Check main DUT is Idle
        self._voice_call_api.check_state(
            self._uecmd_types.VOICE_CALL_STATE.NOCALL)

        # Check secondary DUT is Idle
        self._voice_call_api2.check_state(
            self._uecmd_types.VOICE_CALL_STATE.NOCALL)

        # Keep DUT busy by calling VoiceMail number, after disabling call-waiting feature
        self._phone_system_api.wake_screen()
        self._voice_call_api.dial(self._ss_phone_number, check_state=False)

        # Sleep time to make the DUT call state "ACTIVE"
        time.sleep(7)

        # main DUT state should be ACTIVE
        self._voice_call_api.check_state(
            self._uecmd_types.VOICE_CALL_STATE.ACTIVE)

        # secondary DUT should be IDLE
        self._voice_call_api2.check_state(
            self._uecmd_types.VOICE_CALL_STATE.NOCALL)

        # secondary DUT is dialing main DUT
        self._logger.info("Call initiated from Secondary DUT")
        self._voice_call_api2.dial(self._phone_number, check_state=False)
        time.sleep(15)

        # Call state for main DUT is "ACTIVE" as Call-Waiting is Disabled
        self._voice_call_api.check_state(
            self._uecmd_types.VOICE_CALL_STATE.ACTIVE)

        # Call state for secondary DUT is ACTIVE
        self._voice_call_api2.check_state(
            self._uecmd_types.VOICE_CALL_STATE.ACTIVE)

        # Final return
        return (verdict, message)
#--------------------------------------------------------------------------------------------------------
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
        if self._ss_feature == "CALL_FORWARDING":
            # For 'NOT_REACHABLE' case perform the re-registration scenario upon flight mode off
            if self._ss_mode != "NOT_REACHABLE":
                # Flight mode off for main DUT
                self._networking_api.set_flight_mode("off")
                # Sleep 30 seconds
                time.sleep(30)
            # De-activate the call forwarding on main DUT
            self._phone_system_api.wake_screen()
            self._voice_call_api.disable_call_forwarding(self._ss_mode.lower())
            # Sleep few seconds for activation of Supplementary Service
            self._logger.info("Sleep for %s seconds" % self._wait_ss_response)
            time.sleep(self._wait_ss_response)
            try:
                # Check the call forwarding state
                self._voice_call_api.check_call_forwarding_state(LiveDualPhoneVcSs.STATE_DISABLED)
            except DeviceException as device_ex:
                message = "Exception caught: %s" % (str(device_ex))
                self._logger.error(message)
                verdict = Global.FAILURE

        # For 'CALL_WAITING' scenario
        if self._ss_feature == "CALL_WAITING":
            self._logger.info("Disabling Call waiting")
            self._voice_call_api.disable_call_waiting()
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
#-------------------------------------------------------------------------------
    def _check_CLIR(self):

        """
        Performs the Checking for CLIR without answering.
        :rtype: int, str
        :return: the verdict and the message

        """
        # Initialize the verdict
        verdict1=Global.FAILURE
        verdict2 = Global.FAILURE
        msg2="CLIR is not ENABLED"
        #Waking up the screens
        self._phone_system_api.wake_screen()
        self._phone_system_api2.wake_screen()
        #setting the screen timeout
        self._phone_system_api.set_screen_timeout(300)
        self._phone_system_api2.set_screen_timeout(300)
        #Clearing the Call logs
        self._clear_call_history()
        self._device.run_cmd("adb shell input keyevent 82",self._test_timeout,
                                                  force_execution=True)
        time.sleep(3)
        #to check the status for clir *#31#
        self._phone_clir_status = "%2A" + "%23" + "31" + "%23"
        #dialing the *#31# to check the status for clir in phone 1
        self._voice_call_api.dial( self._phone_clir_status, check_state=False)
        #to check the dialog box pop out or not
        if self.d(resourceId='android:id/scrollView').wait.exists(
                    timeout=5000):
            #creating whether list and checking the text is present in the pop dialog box
            if "Clirstatus1" not in self.d.watchers:
                self.d.watcher("Clirstatus1").when(textContains="Caller ID defaults to not restricted.").click(text="OK")
            self.d.watchers.run()
            #return true or false if watcher perform the task
            status1=(self.d.watchers.triggered)
            self._logger.info(status1)
        else:
            err_msg="Dialog is not opening"
            raise DeviceException(DeviceException.OPERATION_SET_ERROR, err_msg)
        time.sleep(8)
        #IF CLIR enabled then status is true else false
        if status1 == True:
            self._voice_call_api.dial(self._phone_number2, check_state=False)
            self._logger.info("Call initiated from DUT to Reference Phone Without UUID")
            time.sleep(10)
            # Wait for incoming call from the main DUT
            self._voice_call_api2.wait_for_state(self._uecmd_types.
                                            VOICE_CALL_STATE.INCOMING,
                                            self._call_setup_time)
            # Release the call
            self._voice_call_api.release()
            #To check the last call details
            (number, call_type, _sim) = self._voice_call_api2.get_last_call_details()
            # Checking the last call type is missed.
            self._logger.info("Checking last call type is MISSED.")
            if call_type != str(self._uecmd_types.VOICE_CALL_TYPE.MISSED):
                return (Global.FAILURE, "Last call should be MISSED, is: %s"
                                    % call_type)
            # Checking the last call number not displayed on the DUT.
            self._logger.info("Checking last call number is private")
            # "" is the phone number returned when the number is unknown
            self._logger.info("the number is %s"%number)

            #to remove the + from the number if number comes
            phone_string=number.replace("+","")
            self._logger.info("THE phone number without + is %s"%phone_string)
            #to check whether the number we got from call details is digits or not
            if phone_string.isdigit():
                return (Global.FAILURE, "The CLIR is enabled but the number %s is shown"
                                    % number)
            else:
                verdict2,msg2= Global.SUCCESS,"The CLIR is enabled and number is restricted"
                self._logger.info(msg2)
            #Call logs are cleared
            self._clear_call_history()
            #Creating the phone number with UUID code*31#
            self._phone_clir_number = "%2A" + "31" + "%23"+self._phone_number2
            #Dialing the number from DUT
            self._voice_call_api.dial(self._phone_clir_number, check_state=False)
            self._logger.info("Call initiated from DUT to the reference phone with UUID code ")
            time.sleep(10)
            # Wait for incoming call from the main DUT
            self._voice_call_api2.wait_for_state(self._uecmd_types.
                                            VOICE_CALL_STATE.INCOMING,
                                            self._call_setup_time)
            # Release the call
            self._voice_call_api.release()
            #To check the last call details
            (number, call_type, _sim) = self._voice_call_api2.get_last_call_details()
            # Checking the last call type is missed.
            self._logger.info("Checking last call type is MISSED.")
            if call_type != str(self._uecmd_types.VOICE_CALL_TYPE.MISSED):
                return (Global.FAILURE, "Last call should be MISSED, is: %s"
                                    % call_type)
            # Checking the last call number not displayed on the DUT.
            self._logger.info("Checking last call number should not be private")
            #the phone number is returned
            self._logger.info("the number is %s"%number)

            #to remove the + from the number if number comes
            phone_string=number.replace("+","")
            self._logger.info("THE phone numerber without + is %s"%phone_string)
            if phone_string.isdigit():
                verdict1,msg1=(Global.SUCCESS, "The CLIR is temporarily deactivated so the number %s is shown"
                                    % number)
                self._logger.info(msg1)
            else:
                return(Global.FAILURE,"The CLIR is temporarily deactivated so number should not be restricted")
                self._logger.info(msg2)
            if verdict1==Global.SUCCESS and verdict2==Global.SUCCESS:
                return Global.SUCCESS,"THE CLIR FUNCTIONALITY IS CHECKED"
        else:
            return(Global.FAILURE,"CLIR is not ENABLED")
#------------------------------------------------------------------------------
    def _ans_CLIR(self):
        """
        Performs the Checking for CLIR wih answering.
        :rtype: int, str
        :return: the verdict and the message

        """
        # Initialize the verdict
        verdict1=Global.FAILURE
        verdict2 = Global.FAILURE
        msg2="CLIR is not ENABLED"
        #Waking up the screens
        self._phone_system_api.wake_screen()
        self._phone_system_api2.wake_screen()
        #setting the screen timeout
        self._phone_system_api.set_screen_timeout(300)
        self._phone_system_api2.set_screen_timeout(300)
        #Clearing the Call logs
        self._clear_call_history()
        self._device.run_cmd("adb shell input keyevent 82",self._test_timeout,
                                                  force_execution=True)
        time.sleep(3)
        self._phone_clir_status = "%2A" + "%23" + "31" + "%23"
        #dialing the *#31# to check the status for clir in phone 1
        self._voice_call_api.dial( self._phone_clir_status, check_state=False)
        #to check the dialog box pop out or not
        if self.d(resourceId='android:id/scrollView').wait.exists(
                    timeout=5000):
            #creating whether list and checking the text is present in the pop dialog box
            if "Clirstatus1" not in self.d.watchers:
                self.d.watcher("Clirstatus1").when(textContains="Caller ID defaults to not restricted.").click(text="OK")
            self.d.watchers.run()
            #return true or false if watcher perform the task
            status1=(self.d.watchers.triggered)
            self._logger.info(status1)
        else:
            err_msg="Dialog is not opening"
            raise DeviceException(DeviceException.OPERATION_SET_ERROR, err_msg)
        time.sleep(8)
        if status1==True:
            #Dialing the number from DUT
            self._voice_call_api.dial(self._phone_number2, check_state=False)
            self._logger.info("Call initiated from DUT to the reference phone without UUID")
            time.sleep(10)
            # Wait for incoming call from the main DUT
            self._voice_call_api2.wait_for_state(self._uecmd_types.
                                             VOICE_CALL_STATE.INCOMING,
                                             self._call_setup_time)
            #Call is received from reference phone
            self._voice_call_api2.answer()
            # Check voice call is active
            self._voice_call_api2.wait_for_state(
                                                 self._uecmd_types.VOICE_CALL_STATE.ACTIVE,
                                                 self._call_setup_time)
            # WAIT FOR CALL DURATION
            self._logger.info(
                              "Wait for call duration: " + str(self._callduration) + "s...")
            time.sleep(self._callduration)
            # Check call is still active
            self._voice_call_api.check_state(
                                             self._uecmd_types.VOICE_CALL_STATE.ACTIVE)
            # Release the call from DUT
            self._voice_call_api.release()
            #To check the last call details
            (number, call_type, _sim) = self._voice_call_api2.get_last_call_details()
            # Checking the last call type is incoming.
            self._logger.info("Checking last call type is INCOMING.")
            if call_type != str(self._uecmd_types.VOICE_CALL_TYPE.INCOMING):
                return (Global.FAILURE, "Last call should be INCOMING, is: %s"
                                    % call_type)
            # Checking the last call number not displayed.
            self._logger.info("Checking last call number is private")
            # "" is the phone number returned when the number is unknown
            self._logger.info("the number is %s"%number)
            #To remove the + from the number if number comes
            phone_string=number.replace("+","")
            self._logger.info("THE phone numerber without + is %s"%phone_string)
            #To check whether the number we got from call details is digits or not
            if phone_string.isdigit():
                return (Global.FAILURE, "The CLIR is enabled but the number %s is shown"
                                    % number)
            else:
                verdict2,msg2= Global.SUCCESS,"The CLIR is enabled and number is restricted "
                self._logger.info(msg2)
            #Call logs are cleared
            self._clear_call_history()
            #Creating the phone number with UUID code*31#
            self._phone_clir_number = "%2A" + "31" + "%23"+self._phone_number2
            #Dialing the number from DUT
            self._voice_call_api.dial(self._phone_clir_number, check_state=False)
            self._logger.info("Call initiated from DUT to the reference phone with UUID code")
            time.sleep(10)
            # Wait for incoming call from the main DUT
            self._voice_call_api2.wait_for_state(self._uecmd_types.
                                             VOICE_CALL_STATE.INCOMING,
                                            self._call_setup_time)
            #Call is received from reference phone
            self._voice_call_api2.answer()
            # Check voice call is active
            self._voice_call_api2.wait_for_state(
                                                 self._uecmd_types.VOICE_CALL_STATE.ACTIVE,
                                                 self._call_setup_time)
            # WAIT FOR CALL DURATION
            self._logger.info(
                              "Wait for call duration: " + str(self._callduration) + "s...")
            time.sleep(self._callduration)
            # Check call is still active
            self._voice_call_api.check_state(
                                             self._uecmd_types.VOICE_CALL_STATE.ACTIVE)
            # Release the call
            self._voice_call_api.release()
            #To check the last call details
            (number, call_type, _sim) = self._voice_call_api2.get_last_call_details()
            # Checking the last call type is missed.
            self._logger.info("Checking last call type is INCOMING.")
            if call_type != str(self._uecmd_types.VOICE_CALL_TYPE.INCOMING):
                return (Global.FAILURE, "Last call should be INCOMING, is: %s"
                                    % call_type)
            # Checking the last call number not displayed on the DUT.
            self._logger.info("Checking last call number should not be private")
            #The phone number is returned
            self._logger.info("the number is %s"%number)
            #to remove the + from the number if number comes
            remove1=number.replace("+","")
            if remove1.isdigit():
                verdict1,msg1=(Global.SUCCESS, "The CLIR is temporarily deactivated so the number %s is shown"
                                    % number)
                self._logger.info(msg1)
            else:
                return(Global.FAILURE,"The CLIR is temporarily deactivated so number should not be restricted")
                self._logger.info(msg2)
            if verdict1==Global.SUCCESS and verdict2==Global.SUCCESS:
                return Global.SUCCESS,"THE CLIR FUNCTIONALITY IS CHECKED"
        else:
            return(Global.FAILURE,"CLIR is not ENABLED")
#---------------------------------------------------------------------------------
    def _clear_call_history(self):
        """
        CLEAR THE CALL LOGS
        """
        self._phone2.run_cmd("adb shell input keyevent 82",self._test_timeout,
                                                  force_execution=True)
        #to launch the dailer activity
        self._phone2.run_cmd("adb shell am start -n com.android.dialer/.DialtactsActivity",self._test_timeout,
                                                  force_execution=True)
        if self.d2(resourceId='com.android.dialer:id/search_box_collapsed').wait.exists(
                    timeout=5000):
            self.d2(resourceId='com.android.dialer:id/dialtacts_options_menu_button').click()
            self.d2(text='Call History',resourceId='android:id/title').click()

        else:
            err_msg="Call history Dialog is not opening"
            raise DeviceException(DeviceException.OPERATION_SET_ERROR, err_msg)
            return Global.FAILURE,err_msg
        if self.d2(resourceId='android:id/action_bar',className='android.view.ViewGroup').wait.exists(timeout=5000):
            self._phone2.run_cmd("adb shell input keyevent 82",self._test_timeout,
                                                  force_execution=True)
            #To check clear history dailog box is there or not
            if self.d2(text='Clear call history',resourceId='android:id/title').wait.exists(timeout=5000):
                self.d2(text='Clear call history',resourceId='android:id/title').click()
            else:
                self._logger.info("CALL HISTORY IS EMPTY")
                return Global.SUCCESS,"CALL HISTORY IS EMPTY"
        else:
            err_msg="Call history Activity not opening"
            raise DeviceException(DeviceException.OPERATION_SET_ERROR, err_msg)
            return Global.FAILURE,err_msg
        if self.d2(resourceId='android:id/title_template',className='android.widget.LinearLayout').wait.exists(
                    timeout=5000):

            self.d2(index=1,resourceId='android:id/button1').click()
            self._logger.info("CALL HISTORY IS DELETED")
            return Global.SUCCESS,"CALL HISTORY IS DELETED"

        else:
            err_msg="Dialog is not opening"
            raise DeviceException(DeviceException.OPERATION_SET_ERROR, err_msg)
            return Global.FAILURE,err_msg