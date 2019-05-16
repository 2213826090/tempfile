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
:summary:  Use case to validate LTE IMS registration on LIVE network
:since: 20/06/2014
:author: asebbanx
"""

import time

from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from UtilitiesFWK.Utilities import Global, internal_shell_exec
from acs_test_scripts.Utilities.RegistrationUtilities import \
    ImsRegistrationStatus, ImsConfigGenerator
from ErrorHandling.AcsToolException import AcsToolException
from ErrorHandling.AcsConfigException import AcsConfigException
from acs_test_scripts.Device.UECmd.UECmdTypes import PreferredNetwork


class LiveLteImsReg(UseCaseBase):

    """
    Test of Live IMS registration over LTE.

    This test case requires a Com4Innov cell to work.
    Because of this constraint some values that should
    have been given as parameters are actually hard-coded.
    That is to be changed if this test ever has to be
    executed in a different environment.
    """

    IMS_CONFIGURATION_FILE = "/sdcard/intel/IMS_Config.ini"
    """
    The path on the device where the IMS configuration file has to be pushed.
    """

    IMS_REGISTRATION_SUCCESS = 0
    """
    IMS registration status in a case of success.
    """

    IMS_REGISTRATION_FAILED = 1
    """
    IMS registration status in a case of failure.
    """

    IMS_REGISTRATION_NOT_ATTEMPTED = 2
    """
    IMS registration status when registration has not been attempted.
    """

    C4I_DEFAULT_IPV4_APN_CONFIG = {
        "name": "C4I internet",
        "apn": "internet.v4.pft",
        "protocol": "IPV4"
    }
    """
    The parameters to use for IMS default APN configuration - IPV4.
    """

    APN_VALUES = {
        "IPV4": "internet.v4.pft",
        "IPV6": "internet.v6.pft",
        "IPV4V6": "internet.v4v6.pft"
    }
    """
    The parameters for IMS APN values.
    """

    XICFG_IP_VERSION = {
        "IPV4": 0,
        "IPV6": 1,
        "IPV4V6": 2
    }
    """
    The parameters related with IP version for the +XICFG AT command.
    """

    def __init__(self, tc_name, global_config):
        """
        Initializes this instance.
        """
        # Call inherited initializer
        UseCaseBase.__init__(self, tc_name, global_config)

        # Read registrationTimeout from Device_Catalog.xml
        self._registration_timeout = \
            int(self._dut_config.get("registrationTimeout"))

        # Read IMS registration parameter
        self._ims_registration_timeout = self._tc_parameters.get_param_value(
            "IMS_REGISTRATION_TIMEOUT",
            default_cast_type=int)

        # Read the IMS password parameter
        self._digest_password = \
            self._tc_parameters.get_param_value("IMS_DIGEST_PASSWORD", "")
        # Device phone number
        self._dut_phone_number = str(self._device.get_phone_number())
        # If not valid, retrieve it from bench configuration file
        if self._dut_phone_number is None or self._dut_phone_number =="":
            self._dut_phone_number = \
                self._retrive_parameter_bench_config(global_config, "PhoneNumberDut1")

        # Read the IMS configuration file path parameter
        self._ims_parameter_file = \
            self._tc_parameters.get_param_value("IMS_PARAMETER_FILE")

        # Read the parameter for deregistration method
        self._deregistration_method = \
            self._tc_parameters.get_param_value("DEREGISTRATION_METHOD", None)

        # Retrieve the IMS modem configuration procedure from bench configuration file
        # if defined
        try:
            self._ims_configure_modem_procedure = \
                str(self._retrive_parameter_bench_config(global_config, "ImsModemConfiguration"))
        except AcsConfigException:
            self._ims_configure_modem_procedure = "AT-COMMANDS"

        # Read the IMS IP version parameter
        self._ip_version = \
            self._tc_parameters.get_param_value("IP_VERSION",
                                                "IPV4")
        self.__xicfg_ip_version = None

        # Initialize an IMS Configurator instance
        self._generator = None

        # Read the authentication mode
        self.__authentication_mode = None
        authentication_mode = \
            self._tc_parameters.get_param_value("IMS_AUTHENTICATION")
        if authentication_mode:
            authentication_mode = authentication_mode.lower()
        if authentication_mode in ("aka", "digest"):
            self.__authentication_mode = authentication_mode

        # Read the precondition parameter
        self.__preconditions = None
        preconditions = \
            self._tc_parameters.get_param_value("IMS_PRECONDITIONS")
        if preconditions:
            preconditions = preconditions.lower()
        if preconditions in ("ietf", "3gpp"):
            self.__preconditions = preconditions

        # Create an attribute that keeps the list of configurations
        # to be applied.
        if self._ims_configure_modem_procedure == "IMS_ANDROID":
            self.__configurations = \
                    (self._ip_version,
                     self.__preconditions,
                     self.__authentication_mode)
        else:
            self.__configurations = \
                    (self.__authentication_mode + "-at_cmd",
                     self.__preconditions + "-at_cmd",
                     "flush" + "-at_cmd")

        # Read the IMS registration operation
        try:
            self._ims_reg_operation = \
                self._retrive_parameter_bench_config(global_config, "ImsRegistrationOperation")
        except AcsConfigException:
            self._ims_reg_operation = \
                str(self._tc_parameters.get_param_value("IMS_REGISTRATION_OPERATION",
                                                        "REGISTER"))
        if "CHECK_ONLY" == self._ims_reg_operation:
            self._logger.info("IMS registration will only be checked")
        elif "SKIP_DEREGISTER" == self._ims_reg_operation:
            self._logger.info("IMS registration will be performed but de-registration will be skipped.")

        # Retrieve appropriate UE Cmd categories
        self._phone_system_api = self._device.get_uecmd("PhoneSystem")
        self._networking_api = self._device.get_uecmd("Networking")
        self._file_system_api = self._device.get_uecmd("File")
        self._modem_flashing_api = self._device.get_uecmd("ModemFlashing")
        self._modem_api = self._device.get_uecmd("Modem")

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        # Call inherited set_up method
        UseCaseBase.set_up(self)

        # Check the value for the IMS operation parameter
        if self._ims_reg_operation not in ("REGISTER", "CHECK_ONLY", "SKIP_DEREGISTER"):
            raise AcsConfigException(
                AcsConfigException.INVALID_PARAMETER,
                "Invalid parameter value for '%s' (%s)" % (
                    "IMS_REGISTRATION_OPERATION",
                    str(self._ims_reg_operation)))

        # Perform operation according the the IMS operation parameter
        if self._ims_reg_operation in ("REGISTER", "SKIP_DEREGISTER"):
            # De-register for IMS services
            (verdict, message) = self._perform_ims_deregistration()
            if verdict == Global.FAILURE:
                raise AcsToolException(
                    AcsToolException.OPERATION_FAILED,
                    message)

            # Generate the configuration and flush the parameters
            # to the NVM.
            # We force the 'flush' configurations.
            self._generator = ImsConfigGenerator(
                self._ims_parameter_file,
                None,
                None,
                self._logger)

        # Check the value for the IP_VERSION parameter
        if self._ip_version not in ("IPV4", "IPV6", "IPV4V6"):
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "Invalid parameter value for '%s' (%s)" % ("IP_VERSION", str(self._ip_version)))

        if self._ims_reg_operation in ("REGISTER", "SKIP_DEREGISTER"):
            # Set the APN for IPV6 or IPV4V6
            if self._ip_version in ("IPV6", "IPV4V6"):
                # Modify the default values for APN configuration depending on the IP version
                ipv6_apn_values = self.C4I_DEFAULT_IPV4_APN_CONFIG.copy()
                ipv6_apn_values["apn"] = self.APN_VALUES[self._ip_version]
                ipv6_apn_values["protocol"] = self._ip_version

                # Update the APN configuration
                self._logger.info("Setting the APN configuration for %s protocol."
                                  % self._ip_version)
                self._networking_api.update_apn(ipv6_apn_values)
            # Default case - IPV4
            else:
                self._logger.info("APN configuration is the default one, for %s protocol."
                                  % self._ip_version)

            # Configure value for +XICFG command depending on the IP version
            self.__xicfg_ip_version = str(self.XICFG_IP_VERSION[self._ip_version])

            # Prior to anything let us check we are registered to LTE first
            self._modem_api.check_rat_with_pref_network(
                PreferredNetwork.LTE_ONLY,
                self._registration_timeout)

            # Apply the IMS configuration
            self._generate_configuration()

        # Set up is done correctly
        return (Global.SUCCESS, "No errors")

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        # Call inherited run_test method
        UseCaseBase.run_test(self)

        # Initialize some local variables
        registration_message = "IMS registration failure."
        verdict = Global.FAILURE
        in_service_status = ImsRegistrationStatus.in_service()

        # Proceed with the proper execution of the test.
        if "REGISTER" == self._ims_reg_operation:
            # Sleep during registration timeout + IMS registration timeout
            self._logger.info("Waiting %d seconds for IMS registration..." % \
                self._ims_registration_timeout)

            # Check the registration status from DUT as returned by the API
            self._networking_api.check_ims_registration_before_timeout(
                self._ims_registration_timeout)

        # Log the latest IMS registration status
        registration_status = self._get_registration_status()
        self._logger.info("Registration status: %s (%d)" % (
            str(registration_status),
            registration_status.as_int()))
        # Compute the verdict
        if registration_status == in_service_status:
            verdict = Global.SUCCESS
            registration_message = "IMS registration success."

        # Check IMS registration after registration OFF/ON procedures
        if self._deregistration_method:
            self._logger.info("Check IMS registration during registration OFF/ON procedures")
            (verdict, registration_message) = \
                self._check_registration_off_on()

        # Return the test result
        return (verdict, registration_message)

    def tear_down(self):
        """
        Disposes this test.
        """
        # Call the inherited tear_down method
        UseCaseBase.tear_down(self)

        if "REGISTER" == self._ims_reg_operation and self._ims_configure_modem_procedure != "IMS_ANDROID":
            # Restore the original IMS configuration file
            try:
                self._restore_configuration()
            except AcsToolException as acs_ex:
                self._logger.warning("Error in tear down step: %s" % str(acs_ex))

            # Restore the IPV4 APN configuration
            if self._ip_version in ("IPV6", "IPV4V6"):
                self._logger.info("Restore the default APN configuration")
                self._networking_api.update_apn(self.C4I_DEFAULT_IPV4_APN_CONFIG)

        # End of tear_down
        return (Global.SUCCESS, "No errors")

    def _get_registration_status(self):
        """
        Returns an ImsRegistrationStatus object describing the current
        IMS registration status of the device.
        :return: the current registration status
        :rtype: ImsRegistrationStatus
        """
        return ImsRegistrationStatus(
            self._networking_api.get_ims_registration_status())

    def _check_registration_off_on(self):
        """
        Performs the registration OFF->ON and checks
        the IMS registration status for both cases.

        :return: the verdict and its corresponding message
        :rtype: int, str
        """

        verdict = Global.SUCCESS

        # Registration OFF depending on the method (AIRPLANE/AT Command)
        if self._deregistration_method == "AIRPLANE_ON_OFF":
            self._networking_api.set_flight_mode("on")
        else:
            # IMS Config
            if self._ims_configure_modem_procedure == "IMS_ANDROID":
                self._modem_api.turn_ims("OFF")
            # AT commands
            else:
                (status, message) = self._execute_at_command("at+xireg=0")
                if status != Global.SUCCESS:
                    message = "%s (%s)." % (
                        "An error occured during IMS de-registration",
                        message)
                    raise AcsToolException(
                        AcsToolException.OPERATION_FAILED,
                        message)
        # Wait 5 seconds
        time.sleep(5)

        # Check the registration status for IMS, should be "OUT_OF_SERVICE"
        ims_service_status = ImsRegistrationStatus.out_of_service()
        # Log the latest IMS registration status
        registration_status = self._get_registration_status()
        self._logger.info("Registration status: %s (%d)" % (
            str(registration_status),
            registration_status.as_int()))

        if registration_status != ims_service_status:
            verdict = Global.FAILURE
            message = "IMS is still registered after registration OFF."

        # Checks that LTE network registration
        if self._deregistration_method == "AT_COMMAND":
            self._modem_api.check_rat_with_pref_network(PreferredNetwork.LTE_ONLY, self._registration_timeout)
            # Wait 5 seconds
            time.sleep(5)

        # Registration ON depending on the method (AIRPLANE/AT Command)
        if self._deregistration_method == "AIRPLANE_ON_OFF":
            self._networking_api.set_flight_mode("off")
        else:
            # IMS Config
            if self._ims_configure_modem_procedure == "IMS_ANDROID":
                self._modem_api.turn_ims("ON")
            # AT commands
            else:
                (status, message) = self._execute_at_command("at+xireg=1")
                if status != Global.SUCCESS:
                    message = "%s (%s)." % (
                        "An error occured during IMS re-registration",
                        message)
                    raise AcsToolException(
                        AcsToolException.OPERATION_FAILED,
                        message)
        # Wait 5 seconds
        time.sleep(5)

        # Check the registration status for IMS, should be "IN_SERVICE"
        ims_service_status = ImsRegistrationStatus.in_service()
        self._networking_api.check_ims_registration_before_timeout(
            self._ims_registration_timeout)
        # Log the latest IMS registration status
        registration_status = self._get_registration_status()
        self._logger.info("Registration status: %s (%d)" % (
            str(registration_status),
            registration_status.as_int()))

        if registration_status != ims_service_status:
            verdict = Global.FAILURE
            message = "IMS is not registered after registration ON."

        if verdict == Global.SUCCESS:
            message = "IMS registration successful after registration OFF/ON procedures"

        # Return the verdict
        return (verdict, message)

    def _generate_configuration(self):
        """
        Generates the set of AT commands to send to the modem and
        executes them.
        :raise AcsToolException: if an error occurred during the
            generation.
        """
        # Apply the configuration related to IMS registration
        self._apply_configurations(self.__configurations)

        # Trigger IMS registration (we get better chances of success
        # in IMS registration when performing this step separately).
        if self._ims_configure_modem_procedure == "IMS_ANDROID":
            self._modem_api.turn_ims("ON")
        else:
            self._apply_configurations("register-at_cmd")

    def _restore_configuration(self):
        """
        Restores the initial modem configuration.
        :raise AcsToolException: if an error occurred during the
            restore process.
        """
        # Apply the configuration for the IMS configuration reset
        self._apply_configurations(("reset-at_cmd", "flush-at_cmd"))

    def _apply_configurations(self, configuration_names):
        """
        Applies the modem configuration with the given name.
        The configuration_names parameter can be either tuple or list or even
        str (for a single configuration name).
        :param configuration_names: the name of the configuration to apply
        :type configuration_names: tuple
        :raise AcsToolException: if an error occurred during the
            configuration process.
        """
        # Pre processing on phone number format
        phone_number = self._dut_phone_number
        if phone_number.startswith("+"):
            # Remove leading "+" if any
            phone_number = phone_number[1:]
        # Generate the configuration
        template_values = {
            "PASS": self._digest_password,
            "PHONE_NUMBER": phone_number,
            "IP_VERSION": self.__xicfg_ip_version}
        # Update the configuration for the generator
        self._generator.configurations = configuration_names

        if self._ims_configure_modem_procedure == "IMS_ANDROID":
            # Get the parameters which needs to be configured
            (ims_params, param_values) = self._generator.get_ims_config_parameters()
            # Replace the necessary tokens
            param_values = self._generator.replace_tokens(param_values, template_values)
            # Set the IMS configuration
            self._modem_api.\
                set_ims_modem_configuration(ims_params, param_values)

        else:
            # Build AT command list for the configuration
            at_commands = self._generator.get_at_commands()
            at_commands = self._generator.replace_tokens(at_commands, template_values)
            # Execute the AT commands
            (status, message) = self._execute_at_command_set(at_commands)
            if status != Global.SUCCESS:
                message = "%s (%s)." % (
                    "An error occured during IMS configuration",
                    message)
                raise AcsToolException(
                    AcsToolException.OPERATION_FAILED,
                    message)


    def _execute_at_command_set(self, at_commands):
        """
        Executes the given at_commands.

        :type at_commands: tuple
        :param at_commands: the set of AT commands to execute

        :rtype: tuple
        :return: the verdict and the message in a tuple
        """
        # Initialize some local variables
        configuration_status = False
        message = "No error."
        # Iterate on the set of AT commands
        for command in at_commands:
            # Execute the current AT command
            (status, message) = self._execute_at_command(command)
            if status == Global.SUCCESS:
                configuration_status = True
            else:
                self._logger.error("Error on AT command execution: %s " % str(message))
                configuration_status = False
                break
        # Compute the operation verdict
        execution_status = Global.FAILURE
        if configuration_status is True:
            execution_status = Global.SUCCESS
        #Return the result
        return (execution_status, message)

    def _execute_at_command(self, at_command):
        """
        Executes the given at_command.

        :type at_command: str
        :param at_command: the AT command to execute

        :rtype: tuple
        :return: the verdict and the message in a tuple
        """
        # Check parameter
        if not at_command:
            return (Global.FAILURE, "Invalid input parameter")
        self._logger.debug("Sending AT command: %s " % str(at_command))
        # Escape quotes in AT command
        at_command = at_command.replace('\"', '\\"')
        # Build the adb command
        adb_command = "adb -s %s shell %s %s %s \"%s\"" % (
            self._device.get_serial_number(),
            "teltbox",
            "mdm_acm",
            "/dev/gsmtty20",
            at_command)
        # Execute the AT command
        (status, output) = internal_shell_exec(adb_command, 5)  # pylint: disable=W0212
        # Check the command output
        if "-ERROR-" in output:
            message = "Error during AT command execution: %s" % output
            raise AcsToolException(
                AcsToolException.OPERATION_FAILED,
                message)
        #Sleep arbitrary time
        time.sleep(1)
        # Return a status and a message
        return (status, output)

    def _perform_ims_deregistration(self):
        """
        Perform IMS de-registration using AT command / IMS config

        :return: the verdict and its corresponding message
        :rtype: int, str

        :raise AcsToolException OPERATION_FAILED: if an error occurred during the
            AT command execution.
        """
        # Initialize local variables
        verdict = Global.SUCCESS
        message = "No error"

        # AT Command for un register fails if the DUT is not registered to IMS
        registration_status = self._get_registration_status()
        self._logger.info("Registration status: %s (expected %s)." % (
            str(registration_status),
            str(ImsRegistrationStatus.in_service())))
        state = (registration_status != ImsRegistrationStatus.in_service())
        self._logger.debug("Comparison status: %s" % str(state))
        if registration_status != ImsRegistrationStatus.in_service():
            self._logger.info("DUT is not registered to IMS, nothing to do.")
            return (Global.SUCCESS, "Nothing to do.")

        if self._ims_configure_modem_procedure == "IMS_ANDROID":
            # Turn off IMS
            self._modem_api.turn_ims("OFF")
            # Wait 5 seconds
            time.sleep(5)
        else:
            # Execute at+xireg=0 command
            (status, message) = self._execute_at_command("at+xireg=0")
            if status != Global.SUCCESS:
                message = "%s (%s)." % (
                    "An error occured during IMS de-registration",
                    message)
                raise AcsToolException(
                    AcsToolException.OPERATION_FAILED,
                    message)
            # Wait 15 seconds
            time.sleep(15)

        # Check the registration status for IMS, should be "OUT_OF_SERVICE"
        ims_service_status = ImsRegistrationStatus.out_of_service()
        # Log the latest IMS registration status
        registration_status = self._get_registration_status()
        self._logger.info("Registration status: %s (%d)" % (
            str(registration_status),
            registration_status.as_int()))

        if registration_status != ims_service_status:
            verdict = Global.FAILURE
            message = "IMS is still registered after registration OFF."

        return (verdict, message)


    def _retrive_parameter_bench_config(self, global_config, parameter):
        """
        Retrieve the value for a parameter defined in bench configuration file

        :type global_config: object
        :param global_config: global configuration

        :type parameter: str
        :param parameter: the name of the parameter in the bench configuration file

        :return: The value for the parameter
        :rtype: str

        :raise AcsToolException INVALID_PARAMETER: if the parameter is not
            defined in the bench configuration file
        """

        details_duts = global_config.benchConfig.get_parameters("DUT_DETAILS")
        try:
            config_value = details_duts.get_param_value(parameter)
        except AcsConfigException:
            message = "Phone number parameter is not set neither in TC's XML\
             nor in BENCH CONFIG file as \"%s\"" %(parameter)
            raise AcsConfigException(
                AcsConfigException.INVALID_PARAMETER,
                message)
        return config_value

