"""
:copyright: (c)Copyright 2012, Intel Corporation All Rights Reserved.
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
:summary: Utilities to run GTester commands
:author: asebbanx
:since: 08/11/2012
"""

from acs_test_scripts.Utilities.GTester.Parser import GCampaignParser, GTesterResult
from UtilitiesFWK.Utilities import Global


class GTesterRunner(object):

    """
    A class responsible for I{Component Testing} tasks automation.

    This class uses a C{DUT} instance (at instantiation time) in
    order to perform some operations.

    Instances of this class are able to run I{GTester} and I{AT} commands
    as well as perform some operations on the phone that may be required
    before running some tests (e.g: disable some deamons such as C{rild}).
    """

    DEFAULT_COMMAND_TIMEOUT = 240
    """
    The default timeout (in seconds) to wait when
    running a command on the I{DUT}.
    """

    def __init__(self, device):
        """
        Initializes this instance.

        :param device: the C{AndroidDeviceBase} instance that will be tested.
        :type device: AndroidDeviceBase
        """
        self.__gtester_result = None
        self.__device = device
        self.__timeout = None

    def get_timeout(self):
        """
        Returns the timeout used when running
        shell commands on the board.

        :rtype: int
        :return: the timeout in seconds
        """
        if self.__timeout is None:
            self.__timeout = GTesterRunner.DEFAULT_COMMAND_TIMEOUT
        return self.__timeout

    def set_timeout(self, timeout):
        """
        Sets the timeout to use when running
        shell commands to the given value.

        :param timeout: the new timeout value (in seconds):
        :type timeout: int
        """
        self.__timeout = timeout

    def run_gtester_command(self, command):
        """
        Runs the given I{gtester} command.

        :param command: the I{gtester} command to run.
        :type command: str

        :rtype: tuple
        :return: a C{tuple} containing the exit status of the
            command and its ouput (<exit_status>, <output>).
        """
        # First create a GTesterResult
        result = GTesterResult.create()
        # Keep a track of the result
        self._set_last_result(result)
        # Pre-process the command
        command = command.replace(
            GCampaignParser.DEFAULT_GTESTER_RESULT_FILE,
            result.remote_file_path)
        # Run the command
        (exit_status, message) = self.device.run_cmd(
            command,
            self.timeout,
            force_execution=True)
        # Return the shell command exit status and message
        return exit_status, message

    def run_command(self, command):
        """
        Runs the given command.

        :type command: str
        :param command: the command to run.

        :rtype: tuple
        :return: a C{tuple} containing the exit status of the
            command and its ouput (<exit_status>, <output>).
        """
        (exit_status, message) = (Global.FAILURE, "No command executed.")
        # We check whether the command is an AT command or not
        if self.is_at_command(command):
            # If the command is an AT command, run it as such
            (exit_status, message) = self.run_at_command(command)
        else:
            # Otherwise we consider it a GTester command
            (exit_status, message) = self.run_gtester_command(command)
        # Return the command execution status
        return exit_status, message

    def run_at_command(self, command):
        """
        Runs the given I{AT} command.

        Note that this method requires the application
        C{modem_test} to be installed on the DUT.

        :param command: the I{AT} command to run.
        :type command: str
        """
        # Temporary pass-through method.
        # pylint: disable=W0613
        return Global.SUCCESS, "No error."

    def get_last_result(self):
        """
        Returns the result of the last command execution.

        :rtype: GTesterResult
        :return: the result as C{GTesterResult}
        """
        return self.__gtester_result

    def get_device(self):
        """
        Returns this object's attached I{DUT} instance.

        :rtype: AndroidDeviceBase
        :return: the I{DUT} instance
        """
        return self.__device

    def is_at_command(self, command):
        """
        Returns a boolean indicating whether the given C{command}
        is an I{AT} command or not.
        :rtype: bool
        :return: whether the C{command} is an I{AT} command or not:
            - C{True} if this is an I{AT} command
            - C{False} otherwise
        """
        # Initialize the return value
        is_at_command = False
        # We do something only if the command is not None
        if command is not None:
            # Convert command to str just to be sure
            # and make it lower case
            command = str(command).lower()
            # Check whether we were given an AT command or not.
            # AT commands start with "at" (basic AT commands) or
            # with "at+" (extended AT commands)
            is_at_command = command.startswith(("at", "at+"))
        # Return the value
        return is_at_command

    def _set_last_result(self, result):
        """
        Sets the C{gtester_result} property to the given value.

        :type result: GTesterResult
        :param result: the new result
        """
        self.__gtester_result = result

    # Declare properties
    last_result = property(
        get_last_result,
        None,
        None,
        "The result of the last executed command.")
    device = property(
        get_device,
        None,
        None,
        "Returns this object's attached DUT instance.")

    timeout = property(
        get_timeout,
        set_timeout,
        None,
        "The timeout value (in seconds) used for shell commands.")
