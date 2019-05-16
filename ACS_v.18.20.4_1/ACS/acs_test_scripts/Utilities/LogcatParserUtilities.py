"""
:copyright: (c)Copyright 2015, Intel Corporation All Rights Reserved.
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
:summary: Implements methods for parsing the logcat related with AT commands
:since: 18/06/2015
:author: mariussx
"""

import os
import re

from ErrorHandling.AcsConfigException import AcsConfigException
from UtilitiesFWK.Utilities import Global
from Core.PathManager import Paths

class LogcatParserUtils(object):
    """
    A class that allows to manipulate the logcat buffers
    """
    def __init__(self, device):
        """
        Initializes this instance.

        :param device: the instance of the device used
        :type device: instance
        """
        self._device = device
        self.__logcat_file = None

    def retrive_logcat(self, log):
        """
        This function will retrieve the logcat buffer from the DUT and store it
        under a temporary file

        :type log: str
        :param log: type of buffer to be retrieved

        :raise AcsConfigException: in case the ADB command fails
        """

        # Create a file which stores the locgcat from DUT
        logcat_extract = os.path.join(Paths.EXECUTION_CONFIG,
                                      "parserlogcat.log")

        # Dump the 'radio' log into a file on the local computer
        adb_command = "adb logcat -b %s -d > %s" %(log,
                                                   logcat_extract)

        (result, msg) = self._device.run_cmd(adb_command,
                                             timeout=10)
        if (result == Global.SUCCESS) and ("Read-only file system" in msg):
            # Try again with "os.system" command
            os.system(adb_command)
        elif result == Global.FAILURE:
            return_msg = "Command %s has failed, output: %s " %(adb_command,
                                                                msg)
            raise AcsConfigException(AcsConfigException.OPERATION_FAILED,
                                     return_msg)

        # Set the logcat file
        self.set_logcat_file(logcat_extract)

        return logcat_extract

    def clear_logcat(self, log):
        """
        This function will clear the logcat buffer from the DUT
        :type log: str
        :param log: type of buffer to be cleared

        :raise AcsConfigException: in case the adb command fails
        """

        adb_clear_comand = "adb logcat -b %s -c" %(log)
        (result, msg) = self._device.run_cmd(adb_clear_comand, timeout=5)

        if result == Global.FAILURE:
            return_msg = "Command %s has failed, output: %s " %(adb_clear_comand, msg)
            raise AcsConfigException(AcsConfigException.OPERATION_FAILED,
                                     return_msg)

    def set_logcat_file(self, logcat_file):
        """
        This function initialize the context with the logcat extract captured
        :type logcat_file: str
        :param logcat_file: logcat

        :raise AcsConfigException: in case the adb command fails
        """
        self.__logcat_file = logcat_file

    def get_logcat_file(self):
        """
        This function returns the logcat extras
        :rtype: str
        :return: the logcat extras

        :raise AcsConfigException.OPERATION_FAILED: in case the adb command fails
        :raise AcsConfigException.FILE_NOT_FOUND: in case the logcat was not initialized
        """
        if self.__logcat_file is None:
            raise AcsConfigException(AcsConfigException.FILE_NOT_FOUND,
                                     "No logcat file captured")
        else:
            return self.__logcat_file

    def delete_logcat_file(self):
        """
        This function deletes the logcat file stored
        """
        if self.__logcat_file:
            try:
                # Delete the log
                os.remove(self.__logcat_file)
                self.__logcat_file = None
            except IOError as excp:
                msg = "Cannot delete the logcat file, exception: %s" % excp
                raise AcsConfigException(AcsConfigException.OPERATION_FAILED,
                                         msg)
        else:
            msg = "There is no logcat file!"
            raise AcsConfigException(AcsConfigException.FILE_NOT_FOUND,
                                     msg)

class AtCmdParser(object):
    """
    A class that allows to parse the AT commands from the logcat
    """

    def __init__(self, logcat_data, logger=None):
        """
        Initializes this instance.
        """
        self.__logcat_file = logcat_data
        self.__logger = logger

    @property
    def logger(self):
        """
        The logger instance to use as a property.
        :rtype: logging.logger
        """
        if not self.__logger:
            import logging
            self.__logger = logging.getLogger(__name__)
            self.__logger.setLevel(logging.DEBUG)
            formatter = logging.Formatter('%(name)s - %(levelname)s - %(message)s')
            handler = logging.StreamHandler()
            handler.setFormatter(formatter)
            self.__logger.addHandler(handler)
        return self.__logger

    def get_at_command_response(self, at_command, logcat_file):
        """
        Procedure to retrieve, from the logcat, the response for an AT command

        :type at_command: str
        :param at_command: The AT command
        :type self.__logcat_file: str
        :param self.__logcat_file: The context of the logcat file

        :rtype: int, str
        :return: the verdict and AT comamnd response

        :raise AcsConfigException: Exception raised if logcat file cannot be opened
        """
        response = None
        verdict = Global.FAILURE

        # Open the file
        try:
            with open(logcat_file, 'r') as logcat:
                file_data = logcat.readlines()
        except IOError as excp:
            message = "Error while opening the file: %s - Error: %s " %(logcat_file,
                                                                        excp)
            self.logger.error(message)
            raise AcsConfigException(
                AcsConfigException.OPERATION_FAILED,
                message)

        content = "".join(file_data)
        logcat.close()

        if not content.strip():
            self.logger.info("logcat was not saved")
            logcat.close()
            return verdict, response

        # Get the channel number of the sent AT command
        (channel_number, line_number_ofsset) = \
                self.get_channel_for_sent_at_command(at_command,
                                                     file_data)
        # Get the response on the specific channel number
        if channel_number is not None:
            response = self.get_response_on_channel_number(self.__logcat_file,
                                                           channel_number,
                                                           line_number_ofsset)
            if response is not None:
                verdict = Global.SUCCESS
            else:
                verdict = Global.FAILURE
                response = "No response found"
                self.logger.info(response)
        else:
            verdict = Global.FAILURE
            response = "The AT command was sent or an error occurs during parsing the logcat"
            self.logger.info(response)

        # Return verdict
        return (verdict, response)

    def check_at_command_response(self,
                                  at_command,
                                  expected_response):
        """
        Compare the received and the expected responses for the AT command searched

        :type at_command: str
        :param at_command: the AT command for which response will be compared

        :type expected_response: str
        :param expected_response: the expected AT command response

        :rtype: int, str
        :return: verdict and the output message
        """

        # Check that call barring AT command was successfully executed
        self.logger.info("Getting the response for the specific AT command")
        (verdict, at_cmd_response) = \
            self.get_at_command_response(at_command,
                                         self.__logcat_file)
        if verdict is Global.FAILURE:
            return (verdict, at_cmd_response)

        if at_cmd_response not in expected_response:
            output = "The received response '%s' is different from the expected one: '%s'." \
                        %(at_cmd_response,
                          expected_response)
            self.logger.info(output)
            return (Global.FAILURE, output)
        else:
            output = "The received response '%s' is the expected one." %(at_cmd_response)
            self.logger.info(output)
            return (Global.SUCCESS, output)

    def get_channel_for_sent_at_command(self,
                                        at_command,
                                        logcat_data):
        """
        Parse the logcat and extract the channel number on which the AT command was sent to modem

        :type expected_response: str
        :param expected_response: the expected AT command response

        :rtype: str, int
        :return: The channel number and the number of the first line after the one with the AT command
        """
        channel_number = None

        line_number = 0
        # compare with every line
        for line in logcat_data:
            if at_command in line:
                channel_number = self.extract_channel_number(line)
                if channel_number is not None:
                    break
            line_number += 1

        return (channel_number, line_number + 1)

    def extract_channel_number(self, line):
        """
        Extract the channel number on which AT command was sent from the line provided

        :type line: str
        :param line: the line from logcat which contains the AT command

        :rtype: str
        :return: the channel number
        """

        pattern = "chnl=\[\d+\]"

        try:
            channel_extras = re.search(pattern, line)
            channel_number = re.findall(r'\d+', str(channel_extras.group()))
            return channel_number[0]
        except Exception as excp:
            message = "Exception while extracting the channel number: " + str(excp)
            self.logger.error(message)
            return None

    def get_response_on_channel_number(self,
                                       logcat_file,
                                       channel_number,
                                       line_number_offset):
        """
        Retrieve the AT command response

        :type self.__logcat_file: str
        :param self.__logcat_file: The logcat extras
        :type channel_number: str
        :param channel_number: The channel number used for AT command
        :type line_number_offset: int
        :param line_number_offset: The line number from which the response is searched

        :rtype: str
        :return: the channel number
        """

        # Initialize some variables
        response = None
        channel_pattern = "chnl=[%s]" %str(channel_number)
        at_cmd_response_start_pattern = "[<cr><lf>"
        at_cmd_response_end_pattern = "<cr><lf>]"

        # Open the file and read the context
        with open(logcat_file, 'r') as logcat:
            file_data = logcat.readlines()
        content = "".join(file_data)
        logcat.close()
        if not content.strip():
            message = "Error - logcat was not saved."
            self.logger.error(message)
            logcat.close()
            raise AcsConfigException(
                AcsConfigException.OPERATION_FAILED,
                message)

        line_number = 0
        # Compare with every line
        for line in file_data:
            # Go to the first line after the AT command was sent
            if line_number <= line_number_offset:
                line_number += 1
                continue
            else:
                # Check for any response on the given channel number
                if (channel_pattern in line) and ("RX" in line):
                    response = re.findall(re.escape(at_cmd_response_start_pattern)+"(.*)" +\
                                          re.escape(at_cmd_response_end_pattern), line)[0]
                    if response:
                        # Response has been found, break
                        break

                if (channel_pattern in line) and ("TX" in line):
                    # Another AT command was executed on this channel number
                    break

        # Return the response
        return response
