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
:summary: This file implements the LIVE VC MOMR UC
:since: 17/01/2013
:author: asebbanx
"""

import sys
import time

from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from UtilitiesFWK.Utilities import Global

from acs_test_scripts.Utilities.SmsUtilities import compute_sms_equals, SmsMessage
from ErrorHandling.AcsBaseException import AcsBaseException
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsToolException import AcsToolException


class LiveSmsVcFlightMode(UseCaseBase):

    """
    Live SMS + VC while DUT in Flight Mode.
    We do not inherit from existing LIVE VC or LIVE SMS
    Use Case because we want to handle parameters differently.
    """

    KEYCODE_HOME = "3"
    """
    The keycode corresponding the "HOME" keycode.
    """

    def __init__(self, tc_name, global_config):
        """
        Initializes this instance.
        """
        UseCaseBase.__init__(self, tc_name, global_config)

        # Read callSetupTimeout from Phone_Catalog.xml
        self._call_setup_time = \
            int(self._dut_config.get("callSetupTimeout"))

        # Store the settle down duration for later use
        self._settledown_duration = \
            self._device.get_soft_shutdown_settle_down_duration()
        # Read the optional parameter for settle down duration overriding
        override_settledown = \
            self._tc_parameters.get_param_value("OVERRIDE_SETTLEDOWN_DURATION")
        # Override the settle down duration if needed
        if override_settledown not in (None, ""):
            self._settledown_duration = int(override_settledown)

        # Get Test Cases Parameters
        self._destination_number = \
            self._tc_parameters.get_param_value("DESTINATION_NUMBER")

        # Read SMS_TRANSFER_TIMEOUT from xml UseCase parameter file
        self._sms_transfer_timeout = \
            int(self._tc_parameters.get_param_value("SMS_TRANSFER_TIMEOUT"))

        # The SMS message content
        self._message = \
            str(self._tc_parameters.get_param_value("MESSAGE_CONTENT"))

        self._callduration = \
            int(self._tc_parameters.get_param_value("CALL_DURATION"))

        # Get UECmdLayer
        self._voicecall_api = self._device.get_uecmd("VoiceCall")
        self._messaging_api = self._device.get_uecmd("SmsMessaging")
        self._networking_api = self._device.get_uecmd("Networking")
        self._phone_system_api = self._device.get_uecmd("PhoneSystem")

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initilizes this tests.
        """
        UseCaseBase.set_up(self)

        # If Flight Mode was set
        if self._networking_api.get_flight_mode():
            # Disable flight mode and wait
            # for the phone to settle down
            self._logger.info("Disabling Flight Mode.")
            self._networking_api.set_flight_mode("off")
            time.sleep(self._settledown_duration)
        # Other wise we may have some pending calls
        else:
            # Release any previous call (Robustness)
            self._logger.info("Releasing any previous active Voice Call.")
            self._voicecall_api.release()

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Executes the test
        """
        UseCaseBase.run_test(self)

        # Perform a first Voice Call to check that
        # the operation succeeds in nominal conditions
        self.__perform_voice_call()

        # Enable flight mode and wait
        # for the phone to settle down
        self._logger.info("Enabling Flight Mode.")  # pylint: disable-msg=E1101
        self._networking_api.set_flight_mode("on")
        time.sleep(self._settledown_duration)

        # Perform a new Voice Call which is expected to fail
        try:
            self.__perform_voice_call()
        except AcsBaseException as the_exception:
            # Handle expected failure
            self.__check_failure(the_exception, DeviceException, DeviceException.TIMEOUT_REACHED)
        except:
            # Handle non-exception failures
            exception_message = "Unexpected error: %s" % (str(sys.exc_info()[0]))
            raise DeviceException(DeviceException.INTERNAL_EXEC_ERROR, exception_message)

        # Remove annoying possible popups
        self.__remove_popups()

        # Perform a SMS send which is also expected to fail
        try:
            self.__send_sms()
        except BaseException as the_exception:
            # We may have to go back on the home screen
            # to remove pop up message
            # We give the UE Command 5 seconds to do its job.
            self._device.run_cmd(
                "adb shell input keyevent %s" %
                LiveSmsVcFlightMode.KEYCODE_HOME,
                5)
            # Handle expected failure
            self.__check_failure(the_exception, DeviceException, DeviceException.PHONE_OUTPUT_ERROR)
        except:
            # Handle non-exception failures
            exception_message = "Unexpected error: %s" % \
                (str(sys.exc_info()[0]))
            raise DeviceException(DeviceException.INTERNAL_EXEC_ERROR, exception_message)

        # Remove annoying possible popups
        self.__remove_popups()

        # Disable flight mode and wait
        # for the phone to settle down
        self._logger.info("Disabling Flight Mode.")  # pylint: disable-msg=E1101
        self._networking_api.set_flight_mode("off")
        time.sleep(self._settledown_duration)

        # Remove annoying possible popups
        self.__remove_popups()

        # We wait a little longer for the DUT to camp
        time.sleep(self._settledown_duration)

        # Perform another Voice Call to check that
        # the Flight Mode has actually been disabled
        self.__perform_voice_call()

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        Disposes this test.
        """
        UseCaseBase.tear_down(self)

        # If Flight Mode is still set
        if self._networking_api.get_flight_mode():
            # Disable flight mode and wait
            # for the phone to settle down
            self._logger.info("Disabling Flight Mode.")
            self._networking_api.set_flight_mode("off")
            time.sleep(self._settledown_duration)
        # Other wise we may have some pending calls
        else:
            # Release any previous call (Robustness)
            self._logger.info("Releasing any previous active Voice Call.")
            self._voicecall_api.release()

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def __remove_popups(self):
        """
        Removes annoying popups that may appear
        when performing some operation while the
        Flight Mode is activated.
        This methods allows to avoid UI freezes that
        may slow down or even completely block the
        I{camp} phase of the phone.
        """
        # Try and clear possible popups
        # First wake the screen
        self._phone_system_api.wake_screen()
        # We may have to go back on the home screen
        # to remove pop up message
        # We give the UE Command 5 seconds to do its job.
        self._device.run_cmd(
            "adb shell input keyevent %s" % LiveSmsVcFlightMode.KEYCODE_HOME,
            5)

#------------------------------------------------------------------------------

    def __perform_voice_call(self):
        """
        Performs a Voice Call with this test's parameters.

        :raise DeviceException: if the Voice Call was not
            successful.
        """
        self._voicecall_api.dial(self._destination_number)

        self._logger.info("Wait for call duration: %ss..." % str(self._callduration))
        time.sleep(self._callduration)

        self._voicecall_api.check_state(
            self._uecmd_types.VOICE_CALL_STATE.ACTIVE)

        self._voicecall_api.release()

#------------------------------------------------------------------------------

    def __send_sms(self):
        """
        Sends a SMS with this test's parameters.

        :raise DeviceException: if the SMS sending was not
            successful.
        """

        # Clear old SMS (Non blocking for this test if function isn't
        # implemented on CDK)
        time.sleep(self._wait_btwn_cmd)
        self._messaging_api.delete_all_sms()

        # Send SMS to equipment using SMS parameters :
        # - SMS_TEXT
        # - DESTINATION_NUMBER
        time.sleep(self._wait_btwn_cmd)
        sms_sent = SmsMessage(self._message, self._destination_number)

        # register on intent to receive incoming sms
        self._messaging_api.register_for_sms_reception()

        self._messaging_api.send_sms(
            self._destination_number,
            self._message)

        # Get received sms
        sms_received = self._messaging_api.wait_for_incoming_sms(self._sms_transfer_timeout)

        # Compare sent and received SMS (Text,
        # Destination number)
        (_result_verdict, result_message) = compute_sms_equals(
            sms_sent,
            sms_received)

        self._logger.info(result_message)

#------------------------------------------------------------------------------

    def __check_failure(
            self,
            the_exception,
            expected_exception_type,
            expected_error_code):
        """
        Checks whether the given C{BaseException} instance matches
        the expected exception type and error code.

        :param the_exception: the exception to check
        :type the_exception: BaseException

        :param expected_exception_type: the class of the expected
            exception
        :type: type

        :param expected_error_code: the error code (or message) that the
            tested exception should match
        :type expected_error_code: str

        :raise AcsToolException: if the given C{BaseException} instance
            does not match the expected exception and error code.
        """
        # Check the exception class
        if isinstance(the_exception, expected_exception_type):
            # If the class matches, check the error code
            error_message = the_exception.get_generic_error_message()
            if error_message != expected_error_code:
                exception_message = \
                    "Unexpected exception message [%s], expected: %s." % (
                        str(error_message),
                        str(expected_error_code))
                raise AcsToolException(AcsToolException.PROHIBITIVE_BEHAVIOR, exception_message)

        # Otherwise handle ACS exceptions properly
        elif isinstance(the_exception, AcsBaseException):
            # Retrieve useful information about the exception
            error_message = the_exception.get_error_message()
            class_name = the_exception.__class__.__name__
            exception_message = \
                "Unexpected exception [%s]: %s (expected [%s]: %s)." % (
                    class_name,
                    error_message,
                    str(expected_exception_type),
                    str(expected_error_code))
            raise AcsToolException(AcsToolException.PROHIBITIVE_BEHAVIOR, exception_message)
        # Otherwise handle other Python exceptions
        elif isinstance(the_exception, BaseException):
            # Retrieve useful information about the exception
            exception_message = "Unexpected exception: %s "\
                "(expected [%s]: %s)." % (
                    str(the_exception),
                    str(expected_exception_type),
                    str(expected_error_code))
            raise AcsToolException(AcsToolException.PROHIBITIVE_BEHAVIOR,
                exception_message)
        # Otherwise simply raise an exception with generic information
        else:
            exception_message = "Unknown error: %s (expected [%s]: %s)." % (
                str(the_exception),
                str(expected_exception_type),
                str(expected_error_code))
            raise AcsToolException(AcsToolException.PROHIBITIVE_BEHAVIOR, exception_message)
