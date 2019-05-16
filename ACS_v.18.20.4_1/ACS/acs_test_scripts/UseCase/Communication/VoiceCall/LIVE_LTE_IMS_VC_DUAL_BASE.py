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
:summary:  Base use case for LTE IMS MO/MT Voice Calls using two phones
:since: 02/12/2014
:author: mariussx
"""

import os
import time

from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from UseCase.Networking.LIVE_LTE_IMS_REG import LiveLteImsReg
from UtilitiesFWK.Utilities import Global, str_to_bool
from acs_test_scripts.Utilities.RegistrationUtilities import \
    ImsRegistrationStatus
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException
from Device.DeviceManager import DeviceManager

class LiveLteImsVcDualBase(LiveLteImsReg):

    """
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

        # Read the call direction parameter
        self._call_direction = str(self._tc_parameters.get_param_value("CALL_DIRECTION",
                                                                       "MO"))

        # Read the call release parameter
        self._call_release_direction = str(
            self._tc_parameters.get_param_value("CALL_RELEASE",
                                                "MR"))

        # Read the call type parameter
        self._call_type = self._tc_parameters.get_param_value("CALL_TYPE", "")

        # Retrieve call duration parameter
        self._callduration = \
            int(self._tc_parameters.get_param_value("CALL_DURATION", 15))

        # Read the parameter for call on hold/resume procedure
        self._check_on_hold_resume_procedure = str_to_bool(
            self._tc_parameters.get_param_value(
                "CHECK_CALL_ON_HOLD_RESUME",
                "False"))

        # Read the call option parameter for remote party
        self._remote_party_action = \
            self._tc_parameters.get_param_value("REMOTE_PARTY_ACTION", "")

        # Initialize some attributes to store various phone numbers
        self._phone_no_main_dut = None
        self._phone_no_secondary_dut = None

        # Read the phone number parameter from TC's XML
        self._phone_number = self._tc_parameters.get_param_value("PHONE_NUMBER")

        # If Phone number is not specified in TC XML, try to retrieve it from bench config
        if self._phone_number is None:
            # Retrieve the numbers for each phone used, from bench config file
            self._phone_no_main_dut = \
                self._retrive_parameter_bench_config(global_config, "PhoneNumberDut1")
            self._phone_no_secondary_dut = \
                self._retrive_parameter_bench_config(global_config, "PhoneNumberDut2")

        if self._ims_reg_operation == "CHECK_ONLY":
            self._perform_ims_registration = False
        else:
            self._perform_ims_registration = True

        # Read the ftp transfer parameter
        self._ftp_transfer = str_to_bool(
            self._tc_parameters.get_param_value("FTP_TRANSFER",
                                                "False"))

        if self._ftp_transfer is True:
            # Retrieve the parameters needed for FTP data transfer
            self._retrieve_ftp_parameters(global_config)
            # Instantiate the FTP UE commands for main device
            self._ftp_api = self._device.get_uecmd("Ftp")
        else:
            # Initialize the FTP parameters
            self._ftp_api = None
            self._server = None
            self._server_ip_v4_address = None
            self._server_ip_v6_address = None
            self._username = None
            self._password = None
            self._ftp_path = None
            self._ftp_direction = None
            self._ftp_filename = None
            self._dl_ftp_filename = None
            self._ftp_ip_version = None
            self._ip_address = None
            self._xfer_timeout = None

        # Read callSetupTimeout from Phone_Catalog.xml
        self._call_setup_time = \
            int(self._dut_config.get("callSetupTimeout"))

        # Instantiate UE categories for first device
        self._voice_call_api = self._device.get_uecmd("VoiceCall")
        self._networking_api = self._device.get_uecmd("Networking")
        self._file_system_api = self._device.get_uecmd("File")
        self._phone_system_api = self._device.get_uecmd("PhoneSystem")

        # Load instance of the PHONE2
        self._remote_phone = DeviceManager().get_device("PHONE2")
        self._remote_dut_config = DeviceManager().get_device_config("PHONE2")

        # Instantiate UE categories for second device
        if self._remote_phone is not None:
            self._voice_call_api2 = self._remote_phone.get_uecmd("VoiceCall")
            self._networking_api2 = self._remote_phone.get_uecmd("Networking")
            self._phone_system_api2 = self._remote_phone.get_uecmd("PhoneSystem")
        else:
            self._voice_call_api2 = None
            self._networking_api2 = None
            self._phone_system_api2 = None

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
            self._networking_api.check_ims_registration_before_timeout(10)

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

        # Check the call type parameter
        if self._call_direction not in ("MO", "MT"):
            message = "%s '%s' %s '%s'" % (
                "Invalid parameter value (or missing parameter)",
                str(self._call_direction),
                "for parameter",
                "CALL_DIRECTION")
            raise AcsConfigException(
                AcsConfigException.INVALID_PARAMETER,
                message)

        # Check the call type parameter
        if self._call_release_direction not in ("MR", "NR"):
            message = "%s '%s' %s '%s'" % (
                "Invalid parameter value",
                str(self._call_release_direction),
                "for parameter",
                "CALL_RELEASE")
            raise AcsConfigException(
                AcsConfigException.INVALID_PARAMETER,
                message)

        # Check if we have the second phone available
        if self._remote_phone is None:
            # We are using this multi UC with only one phone
            message = \
                "This use case requires two phones to be executed !"
            self._logger.error(message)
            raise AcsConfigException(AcsConfigException.INVALID_BENCH_CONFIG,
                                     message)

        # Boot the other phone (the DUT is already booted)
        if not self._remote_phone.is_available():
            DeviceManager().boot_device("PHONE2")

        # Disable Flight Mode for the secondary DUT (robustness)
        self._networking_api2.set_flight_mode("off")

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

        # Check registration for secondary DUT
        self._check_registration_second_dut()

        # Release any previous call (Robustness)
        self._logger.info("Releasing any ongoing calls")
        self._voice_call_api.release()
        self._voice_call_api2.release()

        # Return the test result
        return (Global.SUCCESS, "No error.")

#------------------------------------------------------------------------------

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

        # End of tear_down
        return (Global.SUCCESS, "No errors")

    def _retrieve_ftp_parameters(self, global_config):
        """
        Read and set the FTP parameters from bench config and TC's .XML files.
        """

        # Get FTP server parameters from bench config file
        self._server = \
            global_config.benchConfig.get_parameters("LAB_LTE_IMS_SERVER")
        self._server_ip_v4_address = self._server.get_param_value("IP4")
        self._server_ip_v6_address = self._server.get_param_value("IP6")
        self._username = self._server.get_param_value("username")
        self._password = self._server.get_param_value("password")
        if self._server.has_parameter("ftp_path"):
            self._ftp_path = self._server.get_param_value("ftp_path")
        else:
            self._ftp_path = ""

        # Read the the direction parameter name from TC's xml file
        self._ftp_direction = self._tc_parameters.get_param_value("FTP_DIRECTION")
        # Read the ftp file name from TC's xml
        if self._ftp_direction == "DL":
            self._ftp_filename = os.path.join(self._ftp_path, self._tc_parameters.get_param_value("DL_FILENAME", ""))
            self._ftp_filename = self._ftp_filename.replace('\\', '/')
            self._dl_ftp_filename = None
        elif self._ftp_direction == "UL":
            # Read the UL_FILENAME value from TC's xml
            self._ftp_filename = os.path.join(self._ftp_path, self._tc_parameters.get_param_value("UL_FILENAME", ""))
            self._ftp_filename = self._ftp_filename.replace('\\', '/')
            self._dl_ftp_filename = None
        elif self._ftp_direction == "BOTH":
            self._dl_ftp_filename = os.path.join(self._ftp_path, self._tc_parameters.get_param_value("DL_FILENAME", ""))
            self._dl_ftp_filename = self._dl_ftp_filename.replace('\\', '/')
            # Read the UL_FILE value from TC's xml
            self._ftp_filename = os.path.join(self._ftp_path, self._tc_parameters.get_param_value("UL_FILENAME", ""))
            self._ftp_filename = self._ftp_filename.replace('\\', '/')

        # Read the the ip_version from TC's xml
        self._ftp_ip_version = self._tc_parameters.get_param_value("FTP_IP_VERSION", "IPV4")

        # Selecting the IPV6 address of the FTP server, according to
        # the TC parameter value.
        if self._ftp_ip_version == "IPV6":
            if self._server_ip_v6_address is not None:
                # If Protocol is IPV6 use IPV6 address.
                log_msg = "Using IPV6 address to connect to the FTP server."
                self._logger.info(log_msg)
                self._ip_address = self._server_ip_v6_address
            else:
                # If IPV6 address is not present in the BenchConfig.
                msg = "The IPV6 parameter is missing from the Bench Config!"
                raise AcsConfigException(AcsConfigException.INVALID_BENCH_CONFIG, msg)
        else:
            self._ip_address = self._server_ip_v4_address

        # Read the XFER_TIMEOUT from UseCase xml Parameter
        self._xfer_timeout = self._tc_parameters.get_param_value("XFER_TIMEOUT")
        if self._xfer_timeout is not None and str(self._xfer_timeout).isdigit():
            self._xfer_timeout = int(self._xfer_timeout)
        else:
            self._xfer_timeout = None

    def _check_ftp_parameters(self):
        """
        Check the parameters related with FTP data transfer

        :raise AcsConfigException: in case any parameter is incorrect
        """

        # Check the call type parameter
        if self._ftp_direction not in ("DL", "UL", "BOTH"):
            message = "%s '%s' %s '%s'" % (
                "Invalid parameter value (or missing parameter)",
                str(self._ftp_direction),
                "for parameter",
                "FTP_DIRECTION")
            raise AcsConfigException(
                AcsConfigException.INVALID_PARAMETER,
                message)

        if self._xfer_timeout is None:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "XFER_TIMEOUT should be int")

    def _check_registration_second_dut(self):
        """
        Check if secondary DUT is registered or not to NW

        :raise DeviceException: in case 2nd DUT not registered
        """
        in_service_status = ImsRegistrationStatus.in_service()
        reg_status = ImsRegistrationStatus(
                            self._networking_api2.get_ims_registration_status())

        # If not registered
        if reg_status != in_service_status:
            self._logger.warning("The secondary DUT is NOT registered! Airplane on/off will be performed!")
            self._networking_api2.set_flight_mode("on")
            # Wait 10 seconds
            time.sleep(10)
            self._networking_api2.set_flight_mode("off")
            # Wait 10 seconds
            time.sleep(10)
            try:
                self._networking_api2.check_ims_registration_before_timeout(60)
            except DeviceException:
                message = "Secondary DUT still not registered after airplane on/off !"
                raise DeviceException(DeviceException.TIMEOUT_REACHED, message)
