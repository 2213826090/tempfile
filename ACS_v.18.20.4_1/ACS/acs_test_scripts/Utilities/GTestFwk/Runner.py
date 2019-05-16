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
:summary: Utilities to run GTestFwk commands
:author: jreynaux
:since: 07/04/2013
"""

from acs_test_scripts.Utilities.GTestFwk.Parser import GTestFwkResult
from acs_test_scripts.Utilities.GTester.Parser import GCampaignParser
import os.path
import re


class GTestFwkRunner(object):

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
        self.__gtest_fwk_result = None
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
            self.__timeout = GTestFwkRunner.DEFAULT_COMMAND_TIMEOUT
        return self.__timeout

    def set_timeout(self, timeout):
        """
        Sets the timeout to use when running
        shell commands to the given value.

        :param timeout: the new timeout value (in seconds):
        :type timeout: int
        """
        self.__timeout = timeout

    def run_gtest_fwk_command(self, command):
        """
        Runs the given I{gtest fwk} command.

        :param command: the I{gtest fwk} command to run.
        :type command: str

        :rtype: tuple
        :return: a C{tuple} containing the exit status of the
            command and its ouput (<exit_status>, <output>).
        """
        file_name = self._extract_file_name(command)
        dir_name = self._extract_dir_name(command)
        if file_name not in [None, ""]:
            if GCampaignParser.DEFAULT_GTESTER_RESULT_FILE in file_name:
                result = GTestFwkResult.create(remote_dir_path=dir_name)
            else:
                result = GTestFwkResult.create(file_name=file_name, remote_dir_path=dir_name)
        else:
            result = GTestFwkResult.create(remote_dir_path=dir_name)

        # Keep a track of the result
        self._set_last_result(result)
        # Pre-process the command
        command = command.replace(
            GCampaignParser.DEFAULT_GTESTER_RESULT_FILE,
            result.file_name)
        # Run the command
        (exit_status, message) = self.device.run_cmd(
            command,
            self.timeout,
            force_execution=True)
        # Return the shell command exit status and message
        return exit_status, message

    def get_last_result(self):
        """
        Returns the result of the last command execution.

        :rtype: GTesterResult
        :return: the result as C{GTesterResult}
        """
        return self.__gtest_fwk_result

    def get_device(self):
        """
        Returns this object's attached I{DUT} instance.

        :rtype: AndroidDeviceBase
        :return: the I{DUT} instance
        """
        return self.__device

    def _set_last_result(self, result):
        """
        Sets the C{gtester_result} property to the given value.

        :type result: GTestFwkResult
        :param result: the new result
        """
        self.__gtest_fwk_result = result

    def _extract_file_name(self, command):
        """
        Extract the file name from the path given in the command

        :param command: The (GTestFwk) command where to find the directory name
        :type command: str

        :return: The file name
        :rtype: str
        """
        output = None
        e = "--gtest_output=\"xml:(.*)$"
        m = re.compile(e).search(command)

        if m is not None:
            output = m.group(1)

        if output not in [None, ""]:
            output = os.path.basename(output)

        return output

    def _extract_dir_name(self, command):
        """
        Extract the directory name from the path given in the command

        :param command: The (GTestFwk) command where to find the directory name
        :type command: str

        :return: The directory name
        :rtype: str
        """
        output = None
        e = "--gtest_output=\"xml:(.*)$"
        m = re.compile(e).search(command)

        if m is not None:
            output = m.group(1)

        if output not in [None, ""]:
            # [1:] to remove the double-quote
            output = os.path.dirname(output[1:])

        return output

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
