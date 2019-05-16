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
:summary:  Use case to validate LTE IMS MO Voice Calls.
:since: 28/06/2014
:author: asebbanx
"""

import time

from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from UseCase.Networking.LIVE_LTE_IMS_REG import LiveLteImsReg
from Utilities.RegistrationUtilities import ImsRegistrationStatus
from UtilitiesFWK.Utilities import Global
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsConfigException import AcsConfigException


class LiveLteImsMoVc(LiveLteImsReg):

    """
    Test of Live MO Voice Call on IMS over LTE with two devices.

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

        # Retrieve call duration parameter
        self._callduration = \
            int(self._tc_parameters.get_param_value("CALL_DURATION", 0))

        # Initialize the attribute to use as phone number
        self._phone_number = self._tc_parameters.get_param_value("PHONE_NUMBER")

        if self._ims_reg_operation == "CHECK_ONLY":
            self._perform_ims_registration = False
        else:
            self._perform_ims_registration = True

        # Read the call type parameter
        self._call_type = self._tc_parameters.get_param_value("CALL_TYPE", "")

        # Read callSetupTimeout from Phone_Catalog.xml
        self._call_setup_time = \
            int(self._dut_config.get("callSetupTimeout"))

        # Instantiate UE categories for first device
        self._voice_call_api = self._device.get_uecmd("VoiceCall")
        self._networking_api = self._device.get_uecmd("Networking")
        self._file_system_api = self._device.get_uecmd("File")
        self._phone_system_api = self._device.get_uecmd("PhoneSystem")

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
        else:
            # Call the base setup method in order to be
            # compliant with ACS framework
            self._logger.info("Skipping IMS registration step (assumed to be done).")
            UseCaseBase.set_up(self)
            if self._perform_ims_registration is None:
                # But raise an exception
                message = "Invalid parameter value for IMS_REGISTRATION_OPERATION"
                raise AcsConfigException(
                    AcsConfigException.INVALID_PARAMETER,
                    message)
            # Simply perform a IMS registration check
            self._networking_api.check_ims_registration_before_timeout(1)

        # Check the call duration parameter
        if not self._callduration:
            message = "%s '%s' %s '%s'" % (
                "Invalid parameter value (or missing parameter)",
                str(self._callduration),
                "for parameter",
                "CALL_DURATION")
            raise AcsConfigException(
                AcsConfigException.INVALID_PARAMETER,
                message)
        # We arbitrarily set a minimum allowed value for call duration
        if self._callduration < 15:
            message = "%s '%s' %s '%s'. %s." % (
                "Invalid parameter value ",
                str(self._callduration),
                "for parameter",
                "CALL_DURATION",
                "The value should be set to 15 at the minimum")
            raise AcsConfigException(
                AcsConfigException.INVALID_PARAMETER,
                message)

        # Check the call type parameter
        if self._call_type not in ("IR92", "IR94_AUDIO", "IR94_RX", "IR94_TX", "IR94_BIDIRECTIONAL"):
            message = "%s '%s' %s '%s'" % (
                "Invalid parameter value (or missing parameter)",
                str(self._call_type),
                "for parameter",
                "CALL_TYPE")
            raise AcsConfigException(
                AcsConfigException.INVALID_PARAMETER,
                message)

        # Disable Flight Mode on both devices
        self._networking_api.set_flight_mode("off")

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

        # Initialize verdict variables
        verdict = Global.SUCCESS
        message = "No error."

        # Attempt an IMS VoiceCall
        self._phone_system_api.wake_screen()
        self._voice_call_api.dial(self._phone_number,
                                  False,
                                  False,
                                  self._call_type)

        # Check voice call is active
        # Use a protected block here because we do not want
        # the Use Case to break in case of failure, as we
        # have some additional checks to perform.
        try:
            self._voice_call_api.wait_for_state(
                self._uecmd_types.VOICE_CALL_STATE.ACTIVE,
                self._call_setup_time)

        except DeviceException as device_ex:
            # Log the error and update the verdict accordingly
            message = "Could not establish Voice Call, " \
                "exception caught: %s" % str(device_ex)
            self._logger.error(message)
            verdict = Global.FAILURE
            # Robustness: release the call
            self._voice_call_api.release()

        # Proceed with the following test steps only if the
        # previous one succeeded.
        # Use a protected block here because we do not want
        # the Use Case to break in case of failure, as we
        # have some additional checks to perform.
        if verdict != Global.FAILURE:
            # Wait for call duration
            self._logger.info(
                "Wait for call duration: %s s..." % str(self._callduration))
            time.sleep(self._callduration)

            # Check call is still active
            try:
                self._voice_call_api.check_state(
                    self._uecmd_types.VOICE_CALL_STATE.ACTIVE)

                # Release the call
                self._voice_call_api.release()

                # Check call is idle
                self._voice_call_api.wait_for_state(
                    self._uecmd_types.VOICE_CALL_STATE.NOCALL,
                    self._call_setup_time)
            except DeviceException as device_ex:
                message = "Error during Voice Call, " \
                    "exception caught: %s" % str(device_ex)
                self._logger.error(message)
                verdict = Global.FAILURE

        # Because in case of failure the IMS stack may have crashed
        # we need to double-check the IMS registration status.
        in_service_status = ImsRegistrationStatus.in_service()
        registration_status = self._get_registration_status()
        self._logger.info("Registation status: %s (%d)" % (
            str(registration_status),
            registration_status.as_int()))
        # Compute the verdict
        if str(registration_status) != str(in_service_status):
            message = "IMS stack has crashed or IMS registration is lost."
            self._logger.error(message)
            verdict = Global.FAILURE

        # Return the test result
        return (verdict, message)

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

        # Release the call
        self._voice_call_api.release()

        # Check call is idle
        self._voice_call_api.wait_for_state(
            self._uecmd_types.VOICE_CALL_STATE.NOCALL,
            self._call_setup_time)

        # End of tear_down
        return (Global.SUCCESS, "No errors")

