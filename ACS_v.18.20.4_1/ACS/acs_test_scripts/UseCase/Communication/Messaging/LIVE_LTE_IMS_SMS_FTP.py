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
:summary:  Use case to validate SMS sending/receiving during data transfer
:since: 18/03/2015
:author: amitrofx
"""

import time
import os

from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from UseCase.Networking.LIVE_LTE_IMS_REG import LiveLteImsReg
from Utilities.RegistrationUtilities import ImsRegistrationStatus
from UtilitiesFWK.Utilities import Global, internal_shell_exec
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsConfigException import AcsConfigException
from acs_test_scripts.Utilities.SmsUtilities import compute_sms_equals, SmsMessage
import acs_test_scripts.Device.UECmd.UECmdTypes as UECmdTypes
import traceback

class LiveLteImsSmsFtp(LiveLteImsReg):

    """
    Test of SMS sending/receiving over LTE during data transfer

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

        # Get FTP server parameters from bench config file
        self._server = \
            global_config.benchConfig.get_parameters("LAB_LTE_IMS_SERVER")
        self._ip_address = self._server.get_param_value("IP")
        self._username = self._server.get_param_value("username")
        self._password = self._server.get_param_value("password")

        if self._server.has_parameter("ftp_path"):
            self._ftp_path = self._server.get_param_value("ftp_path")
        else:
            self._ftp_path = ""

        # The get_param_value output is casted to a str
        # None will be "None"
        if self._username in ("None", ""):
            self._logger.info("FTP user not indicated, using 'anonymous'")
            self._username = str("anonymous")

        # The get_param_value, output is casted to a str
        # None will be "None"
        if self._password in ("None", ""):
            self._logger.info("FTP user's password not indicated, using 'none'")
            self._password = str("none")

        # Read the ftp DL file name from TC's xml
        self._ftp_filename = os.path.join(self._ftp_path, self._tc_parameters.get_param_value("DL_FILENAME", ""))
        self._ftp_filename = self._ftp_filename.replace('\\', '/')
        self._dl_ftp_filename = None

        # The base name of the file we download
        self._ftp_file_basename = self._tc_parameters.get_param_value("BASENAME")

        # Some attributes needed to add robustness to FTP operations
        # The absolute file path on the DUT
        self._absolute_path_on_dut = None

        # Get UECmdLayer for Data Use Cases
        self._networking_api = self._device.get_uecmd("Networking")
        self._phone_system_api = self._device.get_uecmd("PhoneSystem")
        self._ftp_api = self._device.get_uecmd("Ftp")
        self._sms_api = self._device.get_uecmd("SmsMessaging")

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
            # Simply perform a IMS registration check
            self._networking_api.check_ims_registration_before_timeout(1)

        # Compute the file name on DUT
        self._absolute_path_on_dut = "".join([
            self._device.multimedia_path,
            self._ftp_file_basename])

        # Delete the FTP downloaded file on the DUT
        self._delete_file_if_exists(self._absolute_path_on_dut)

        # Disable Flight Mode on the device
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

        # Clear old SMS (Non blocking for this test if function isn't
        # implemented on CDK)
        time.sleep(self._wait_btwn_cmd)
        self._sms_api.delete_all_sms()

        time.sleep(self._wait_btwn_cmd)

        sms_sent = SmsMessage(self._message, self._destination_number)

        # Start the FTP data transfer
        self._dl_id = self._launch_ftp_data_transfer()

        # Check the status of the FTP connection
        status = self._ftp_api.get_ftp_status(self._dl_id)

        # If the transfer is not working some exceptions
        if str(status) <> "transferring":
            # If the transfer is not established raised an exception
            if str(status) in ("notrunning", "transfer failed"):
                message = "FTP transfer failed / not started: %s" % str(status)
                raise DeviceException(DeviceException.CONNECTION_LOST, message)

            # If the transfer status is 'connecting'
            # additionally wait for the connection to be established
            elif str(status) == "connecting":
                self._logger.info("FTP transfer not started yet. STATUS is CONNECTING")
                time.sleep(self._wait_btwn_cmd)
                if str(status) == "connecting":
                    message = "FTP transfer status is CONNECTING for a too long time"
                    raise DeviceException(DeviceException.CONNECTION_LOST, message)

            # If the FTP data transfer is finished already
            # the transferred file length chosen is too short
            else :
                message = "FTP transfer finished before SMS has been sent"
                self._logger.error(message)
                raise AcsConfigException(AcsConfigException.INVALID_TEST_CASE_FILE, message)

        # If the FTP data transfer is on going start the SMS sending procedures
        # If the FTP data transfer status is "transferring"
        else:

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

            final_transfer_status = str(self._ftp_api.get_ftp_status(self._dl_id))
            while final_transfer_status <> "transfer successful" :
                if str(final_transfer_status) in ("transfer successful", "transferring") :
                    time.sleep(self._wait_btwn_cmd)
                else:
                    message = "FTP transfer failed after SMS sent: %s" % str(final_transfer_status)
                    raise DeviceException(DeviceException.CONNECTION_LOST, message)
                final_transfer_status = str(self._ftp_api.get_ftp_status(self._dl_id))

            if final_transfer_status == "transfer successful" :
                message = "FTP transfer finished successfully"
                self._logger.info(message)
                result_message = result_message + ". " + message

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

        # Delete remaining FTP transfered files from DUT
        self._delete_file_if_exists(self._absolute_path_on_dut)

        # Stop the ftp transfer if was not stopped succesfully
        self._ftp_api.stop_ftp(self._dl_id)

        # End of tear_down
        return (Global.SUCCESS, "No errors")

    #------------------------------------------------------------------------------

    def _launch_ftp_data_transfer(self):
        """
        Launch FTP data transfer
        Returns the ID of the FTP transfer

        :rtype: long
        :return: id of the ftp transfer
        """

        # Wait 10 seconds
        time.sleep(10)
        # Launch FTP data transfer

        task_id = self._ftp_api.\
                start_ftp(UECmdTypes.XFER_DIRECTIONS.DL,
                           self._ip_address,
                           self._username,
                           self._password,
                           self._ftp_filename,
                           self._device.multimedia_path,
                           None)

        return task_id

    #------------------------------------------------------------------------------

    def _delete_file_if_exists(self, file_path, fail_on_error=True):
        """
        Deletes the given file on DUT if it exists.

        :type file_path: str
        :param file_path: the absolute path of the file to delete.

        :type fail_on_error: bool
        :param fail_on_error: [optional] a boolean indicating whether
            we want to raise an exception on error or not.
            Defaults to C{True}.
        """
        serial_number = self._device.get_serial_number()
        # Force file path to a str value
        file_path = str(file_path)
        rm_cmd = "adb -s %s shell rm %s" % (
            str(serial_number),
            file_path)
        # We go on only if the file exists on the DUT
        if not self._phone_system_api.check_file_exist_from_shell(file_path):
            self._logger.debug("No such file to delete.")
            return
        # We try to delete the file
        try:
            # Run the command to remove the file, we give it 10 seconds to run
            self._logger.debug("Deleting file %s." % file_path)
            (exit_status, _output) = internal_shell_exec(rm_cmd, 10)
        except (KeyboardInterrupt, SystemExit):
            raise
        # We want to trap any kind of error/exception so
        # we use an empty exception clause and disable the
        # corresponding Pylint warning.
        # pylint: disable=W0702
        except:
            exit_status = Global.FAILURE
            traceback.print_exc()

        # Check the status of the command execution
        if exit_status != Global.SUCCESS:
            # Build an error message
            error_message = "Command execution failed (command: '%s')." % rm_cmd
            # Handle the error case as expected
            if fail_on_error:
                # If we have to fail, raise an exception
                raise DeviceException(DeviceException.OPERATION_FAILED, error_message)
            else:
                # Otherwise simply log a warning
                self._logger.warning(error_message)

        # We double-check that the file has been deleted
        if self._phone_system_api.check_file_exist_from_shell(file_path):
            # Build an error message
            error_message = "File deletion failed (file: '%s')." % file_path
            # Handle the error case as expected
            if fail_on_error:
                # If we have to fail, raise an exception
                raise DeviceException(DeviceException.OPERATION_FAILED, error_message)
            else:
                # Otherwise simply log a warning
                self._logger.warning(error_message)
        else:
            self._logger.info("File %s deleted successfully." % file_path)
