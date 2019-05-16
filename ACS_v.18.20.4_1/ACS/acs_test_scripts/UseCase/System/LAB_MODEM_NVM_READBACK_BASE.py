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
:summary: This file implements the basic methods for Modem NVM READBACK Use Cases
:since: 17/09/2012
:author: asebbanx
"""

from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase


class LabModemNvmReadbackBase(UseCaseBase):

    """
    Lab Modem NVM READBACK Use Case base Class.
    """

    def __init__(self, tc_name, global_config):
        """
        Initializes this instance.
        """

        # Call UseCase base Init function
        UseCaseBase.__init__(self, tc_name, global_config)

        # Initialize flash attributes
        self._nvm_config_file_path = None

        # Get the flash file path value either from parameter file
        nvm_config_file_path = self._tc_parameters.get_param_value("NVM_CONFIGURATION_FILE_PATH")
        if nvm_config_file_path is not None and nvm_config_file_path != "":
            self._nvm_config_file_path = nvm_config_file_path

        # Get the prefix to be checked.
        # All ines from NVM configuration file that start with
        # the given prefix will be checked.
        self._prefixes = None
        prefixes = self._tc_parameters.get_param_value("PREFIXES")
        if prefixes is not None and prefixes != "":
            prefixes = prefixes.split(";")
            self._prefixes = tuple(prefixes)

        # Initialize the attribute that will hold the TTY value
        self._at_proxy_tty = None

        # Initialize the attribute that will hold the
        # handle to the serial port
        self._at_proxy_serial = None

        # The baud rate to talk with the modem is a fixed
        # value as describe it AT proxy tool documentation.
        self._at_proxy_baud_rate = 115200

        # Instantiate the modem flashing UE Commands
        self._modem_flashing_api = self._device.get_uecmd("ModemFlashing")

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test.
        """
        # Run UC base set_up
        UseCaseBase.set_up(self)

        # Consolidate the NVN configuration file path
        result, new_path = self._get_file_path(self._nvm_config_file_path)
        if result != Global.SUCCESS:
            message = "Invalid parameter value for parameter '%s' (%s)." % ("NVM_CONFIGURATION_FILE_PATH", self._nvm_config_file_path)
            self._logger.debug("Could not find resource at %s" % str(new_path))
            self._logger.error(message)
            return Global.FAILURE, message
        else:
            self._nvm_config_file_path = new_path

        return Global.SUCCESS, "No error."

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """

        # Run UC base run_test
        UseCaseBase.run_test(self)

        # Open the configuration file in read-only mode
        config_file = open(self._nvm_config_file_path, 'r')

        # Initialize global verdict and return message
        return_code = None
        return_message = None

        # Read NVM configuration file line by line
        for line in config_file:
            # Check whether this line shall be tested or not
            if line.startswith(self._prefixes) and line.find("=") != -1:
                self._logger.debug("Checking line: %s." % line)
                # Retrieve both the property and its value
                (config_element, element_value) = line.split("=")
                # Store the expected result for later use
                element_value = element_value.translate(None, "\r\n")
                # Build the corresponding AT command to send to the modem
                at_command = "at@%s?" % config_element
                self._logger.debug("Preparing AT command: %s" % at_command)
                self._logger.debug("Value to check: '%s'" % element_value)

                # Issue an AT command to the modem
                self._at_proxy_serial.write("%s\r\n" % at_command)

                # Initialize loop interation values
                answer_complete = False
                answer_line = None
                full_answer = ""
                # Read maximum tries
                maximum_tries = 10
                # Read current try
                current_try = 0

                while not answer_complete and current_try <= maximum_tries:
                    # Update current try
                    current_try += 1
                    # Read a line
                    answer_line = self._at_proxy_serial.readline()
                    # Remove unwanted characters
                    answer_line = answer_line.translate(None, "\r\n")
                    # Ignore the line if empty or None
                    if answer_line is None or answer_line == "":
                        continue
                    else:
                        if answer_line in ("ERROR", "OK"):
                            answer_complete = True
                        else:
                            self._logger.debug("Got modem response: '%s'" % answer_line)
                            full_answer += answer_line

                # Compare with the expected result
                if element_value not in full_answer:
                    # If the values do not match
                    message = "Modem value for '%s' is: '%s' (expected '%s')" % \
                        (config_element, str(full_answer), element_value)
                    # Log an error
                    self._logger.error(message)
                    # Update the global verdict if required
                    return_code = Global.FAILURE

            else:
                self._logger.debug("Ignoring line: %s" % line)

        # Update the verdict if no error occured
        if return_code is None:
            return_code = Global.SUCCESS
            return_message = "No error."
        else:
            # If return code is not None
            # then it means that an error occured.
            # We will return the last error message
            return_message = "Last error was: %s" % message

        # In any case close the file opened in read-only mode
        config_file.close()

        # Return the verdict of the flash
        return return_code, return_message
