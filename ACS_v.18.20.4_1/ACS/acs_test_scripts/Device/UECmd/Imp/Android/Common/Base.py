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
:summary: UECmd based file for all Android Application Framework Base
:since: 21/03/2011
:author: sfusilie
"""
from UtilitiesFWK.Utilities import AcsConstants
from acs_test_scripts.Device.UECmd.Imp.UECmdBase import UECmdBase
import random
import time
import re
import os
from lxml import etree
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.AcsToolException import AcsToolException
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsBaseException import AcsBaseException


class AcsDict(dict):

    """
    This class definition allow overriding of __getitem__
    method from standard dict definition.

    In order to add robustness in case of key not found in dict.
    """

    def __getitem__(self, key):
        """
        Retrieve the content of the dict according to the key.

        :raise AcsConfigException: In case of key not found.
        :rtype: object
        :return: value associated to key
        """
        # Much faster than ``keys()`` which is not a generator
        if key not in self.iterkeys():
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "Key \"" + str(key) + "\" does not exist !")
        else:
            return dict.__getitem__(self, key)


class Base(UECmdBase):

    """
    Class that handle all generic operations
    """

    PARAM_SEPARATOR = "::"
    """
    This class attribute represent the string value
    to be used when extracting result from I{LogCat}.
    It is used to separate a parameter name from
    its value:
        - param1<PARAM_SEPARATOR>value1
    """

    FIELDS_SEPARATOR = " - "
    """
    This class attribute represent the string value
    to be used when extracting result from I{LogCat}.
    It is used to separate all (key,value) couples:
        - param1<PARAM_SEPARATOR>value1<FIELDS_SEPARATOR>param2...
    """

    OPCODE_MARKER = "opcode"
    """
    This class attribute represent the string value
    to be used when extracting UeCmd opcode from I{LogCat}.
    It is used to retrieve the result of the uecmd execution:
        - opcode<PARAM_SEPARATOR>opcode value<FIELDS_SEPARATOR>...
    """

    STATUS_MARKER = "status"
    """
    This class attribute represent the string value
    to be used when extracting UeCmd status from I{LogCat}.
    It is used to define the verdict of the uecmd execution:
        - status<PARAM_SEPARATOR>verdict<FIELDS_SEPARATOR>...
    """

    OUTPUT_MARKER = "output"
    """
    This class attribute represent the string value
    to be used when sending generic text info on I{LogCat}.
    It is used to define the text info of the uecmd execution:
        - output<PARAM_SEPARATOR>text here<FIELDS_SEPARATOR>...
    """

    EXCEPTION_MSG_MARKER = "exception_message"
    """
    This class attribute represent the string value
    to be used when sending generic text info on I{LogCat}.
    It is used to define the text info of the uecmd execution:
        - exception_message<PARAM_SEPARATOR>text here<FIELDS_SEPARATOR>...
    """

    MULTIPLE_OUTPUT_MARKER = "multiple_output"
    """
    This class attribute represent the int value
    to be used when sending multiple feedback from uecmd on I{LogCat}.
    It is used to define the number of nexts I{rows} that will be send on  I{LogCat}
        - multiple_output<PARAM_SEPARATOR>X
    """

    CARRIAGE_RETURN_MARKER = "[-CR-]"
    """
    This class attribute represent a carriage return value to be replaced by \n
    this is for used with file parsing through daemon api
    """

    ALTERNATIVE_PARAM_SEPARATOR = "[-PARAM-]"
    """
    alternative value for param separator if it is present on text
    """

    ALTERNATIVE_FIELDS_SEPARATOR = "[-FLD-]"
    """
    alternative value for field separator if it is present on text
    """

    TASK_STATE = "task_state"
    """
    state of a daemonized AcsTask.
    """

    TASK_ID = "task_id"
    """
    id of daemonized AcsTask.
    """

    SUCCESS = "SUCCESS"
    """
    This class attribute represent the string value
    to be used when sending a success verdict from uecmd on I{LogCat}.
    It is a standard status to be used for logging a I{success}
        - <STATUS_MARKER><PARAM_SEPARATOR><SUCCESS>
    """

    FAILURE = "FAILURE"
    """
    This class attribute represent the string value
    to be used when sending a failure verdict from uecmd on I{LogCat}.
    It is a standard status to be used for logging a I{failure}
        - <STATUS_MARKER><PARAM_SEPARATOR><FAILURE>
    """

    RECEIVED = "RECEIVED"
    """
    This class attribute represent the string value
    to be used when retrieving an intent well received from uecmd on I{LogCat}.
    It is a standard output to be used for logging an I{intent reception}
        - <OUTPUT_MARKER><PARAM_SEPARATOR><RECEIVED>
    """

    UECMD_TIMEOUT_INTENT_MSG = "The board did not get the intent in time"
    """
    This class attribute represent the string value
    to be used when raising an exception if the intent is not received
    after retries.
    It is a standard message that can be used to be used for other purpose
    """

    UECMD_TIMEOUT_RESULT_MSG = "Did not get ue command result from the board in time."
    """
    This class attribute represent the string value
    to be used when raising an exception if the result of an uecmd
    is not received after waiting maximum of the timeout.
    It is a standard message that can be used to be used for other purpose
    """

    AUTO_LOG_NEW_ENTRY_SEPARATOR = ":[***NEW-ENTRY***]:"
    """
    this class attribute represent autolog separator between two logs"
    """

    SHELL_FILE_PARSING_ERROR = ["no such file", "read-only file system",
                                "no such file or directory", "Permission denied", "error: device not found"]
    """
    this class attribute represent part of the recurring error messages you can meet
    when executing direct shell cmd
    """

    def __init__(self, device):
        """
        Constructor
        """
        super(Base, self).__init__()
        self._cmd = "adb shell am"
        self._device = device
        self._uecmd_default_timeout = int(self._device.get_uecmd_timeout())
        self._device_logger = self._device.get_device_logger()
        self._logger = self._device.get_logger()
        self._last_tag_op_code = ""
        self.icategory = None
        self.component = None
        self._daemon_component = "com.intel.acs.agent/.daemonizer.Daemonizer"
        self._autolog_component = "com.intel.acs.agent/.daemonizer.AutoLog"
        self._autolog_folder = ""
        self._task_folder = ""
        if "GENERAL" in device.get_em_parameters().keys():
            self._autolog_folder = "%sautolog" % device.get_em_parameters()["GENERAL"]["GENERATED_FILE_PATH"]
            self._task_folder = "%s" % device.get_em_parameters()["GENERAL"]["GENERATED_FILE_PATH"]

        self._autotog_category = "intel.intents.category.AUTOLOG"
        self._daemon_category = "intel.intents.category.DAEMONIZER"
        self._parse_reg = re.compile("([^ ]*::.*)")

    def _build_list_from_dict(self, input_dict, input_string):
        """
        Returns a list built from the given dict and given str key.
        The input str's expected format shall be a python-like
        list description.

        :type input_dict: dictionary list
        :param input_dict: the dictionnary list to parse

        :type input_string: str
        :param input_string: the str key to extract

        :raise AcsToolException: if the provided parameter is not valid
        :rtype: list
        :return: a list of str built from the provided dict and str parameter
        """
        try:
            output_list = []
            for input_dict_elt in input_dict:
                if input_string in input_dict_elt:
                    output_list.append(input_dict_elt[input_string])

        except (KeyboardInterrupt, SystemExit):
            raise
        except:
            raise AcsToolException(AcsToolException.INVALID_PARAMETER,
                                   "Invalid input dict retrieved: %s" % (str(input_dict)))

        return output_list

    def _remove_ack_message(self, messages):
        """
        Internal method that is used to remove the message indicating
        that the intent has been received

        :type  messages: list
        :param messages: List of str key for each result sent from Agent

        :rtype: list
        :return: a list containing str dictionaries but without acknowledgment message
        """
        index = -1
        for message in messages:
            if ((message.find(Base.STATUS_MARKER + Base.PARAM_SEPARATOR + Base.SUCCESS) != -1) and
                    (message.find(Base.OUTPUT_MARKER + Base.PARAM_SEPARATOR + Base.RECEIVED) != -1)):
                index = messages.index(message)
                self._logger.debug("Acknowledgment message found, index: " + str(index))
                if index != 0:
                    self._logger.warning("The acknowledgment message was not the First " +
                                         "Received, please check your build's known issues")
                break

        if index != -1:
            self._logger.debug("Remove acknowledgment message")
            messages.pop(index)
        else:
            self._logger.error("Unable to find Acknowledgment message on current results")

        return messages

    def _generate_key(self):
        """
        Generate a pseudo random key to be sent through intent.

        :rtype: String
        :return: The key
        """
        key = str(random.getrandbits(32))
        return key

    def _extract_results(self, tag_op_code, message):
        """
        This method is used to extract the result formatted by the ACS Agent
        into a readable dict.

        :type tag_op_code: str
        :param tag_op_code: The Operation Code reference

        :type message: str
        :param message: The input message to extract the I{ACS} results

        :rtype: dict
        :return: The dict builded from input message
        """

        result = AcsDict()

        payload = self._parse_reg.findall(message)
        if payload:
            for pair in re.split(Base.FIELDS_SEPARATOR, payload[0]):
                split_result = pair.split(Base.PARAM_SEPARATOR)
                if len(split_result) >= 2:
                    key = split_result[0].strip()
                    value = split_result[1].strip()
                    result[key] = value

        if not result:
            error_message = "(%s) " % (str(tag_op_code),)
            error_message += "Cannot parse incoming result:" + message
            raise AcsToolException(AcsToolException.PHONE_OUTPUT_ERROR, error_message)

        return result

    def _exec(self, cmd, timeout=None, force_execution=False, wait_for_response=True, raise_error=True):
        """
        Internal method that execute std command on device

        :type  cmd: str
        :param cmd: cmd to be executed

        :type  timeout: integer
        :param timeout: Script execution timeout in ms

        :type force_execution: Boolean
        :param force_execution: Force execution of command
                    without check phone connected (dangerous)

        :type wait_for_response: Boolean
        :param wait_for_response: Wait response from adb before
                                        statuing on command

        :type raise_error: Boolean
        :param raise_error: Set it to False if you want the erroneous output instead of raising automatically an error
                                        
        :return: output str
        :rtype: string
        """
        if timeout is None:
            timeout = self._uecmd_default_timeout

        (result, output) = self._device.run_cmd(cmd, timeout, force_execution, wait_for_response)
        if not raise_error:
            return output
        else :
            if result == 0:
                if output is not None:
                    return output
                else:
                    raise DeviceException(DeviceException.PHONE_OUTPUT_ERROR, "\"" + cmd + "\" returned null output !")
            else:
                raise DeviceException(DeviceException.PHONE_OUTPUT_ERROR, output)

    def __get_message_triggered_status(self, tag_opcode, timeout):
        """
        Get triggered message from device logger or grep command on device aplogs
        :return: list
        """
        if timeout is None:
            timeout = self._uecmd_default_timeout

        # Retrieve triggered messages from device logger first
        messages = self._device_logger.get_message_triggered_status(tag_opcode)

        if self._device.has_intel_os() and not messages:
            # Try to get opcode directly from logcat files stored on the device (only intel devices)
            result, aplog_grep_output = self._device.run_cmd(
                "adb shell grep {0} /logs/aplog?(.)*([0-9]) /data/logs/aplog?(.)*([0-9])".format(tag_opcode),
                timeout,
                force_execution=True,
                wait_for_response=True,
                silent_mode=True)

            if result == 0 and aplog_grep_output:
                aplog_lines = aplog_grep_output.splitlines()
                messages = [x for x in aplog_lines if "no such file" not in x.lower()]
        return messages

    def __run_command_before_timeout(self, cmd, timeout, tag_op_code, wait_for_response=True):
        """
        Runs the given I{command} before the specified C{timeout} using the
        given C{tag_op_code}.

        .. warning:: Verify if the I{start} intent have been successfully received, if not
        retry to send the command until B{max_retry}
        Returns the raw data messages (as list) without first one, representing intent reception.

        :raise DeviceException : if the command did not exit on C{SUCCESS}.

        :param cmd: the command to run
        :type cmd: str

        :param timeout: the time out before which the command should be completed.
        :type timeout: int

        :param tag_op_code: the tag to use (will be used for LOGCAT messages).
        :type tag_op_code: str

        :rtype: list
        :return: the raw data messages
        """
        # Trigger the answer of the intent
        self._device_logger.add_trigger_message(tag_op_code)

        # Set the time to wait for the ack intent reply
        ack_message_timeout = float(timeout) / 4
        ack_message = False

        # Init values
        messages_received = False
        attempt = 0
        messages = None
        messages_len = 0
        start_time = time.time()

        while not messages_received and (time.time() - start_time) < timeout:
            if not ack_message:
                # Send the intent
                if attempt > 0:
                    # Log if former Intent has not been received
                    debug_msg = "Intent not received after {0} s".format(ack_message_timeout)

                    acs_agent = self._device.get_acs_agent()
                    if acs_agent:
                        if acs_agent.is_running():
                            debug_msg += ", but acs agent is running"
                        else:
                            debug_msg += ", and acs agent is not running"
                    debug_msg += ", retrying sending command '{0}' ...".format(cmd)
                    self._logger.debug(debug_msg)
                attempt += 1
                self._exec(cmd, timeout, wait_for_response)

                # Wait for first message or timeout
                check_time = time.time()
                while (not ack_message and
                       time.time() - check_time < ack_message_timeout and
                       time.time() - start_time < timeout):
                    # Extract the first message
                    messages = self.__get_message_triggered_status(tag_op_code, timeout)

                    # Retrieve messages length if possible.
                    if isinstance(messages, list):
                        messages_len = len(messages)
                        if messages_len > 0:
                            # Log intent
                            self._logger.debug("Intent has been received, waiting for result now...")
                            ack_message = True

                    # Wait before reading triggered messages
                    time.sleep(0.1)
            else:
                messages = self.__get_message_triggered_status(tag_op_code, timeout)

                # Retrieve messages length if possible.
                if isinstance(messages, list):
                    messages_len = len(messages)
                    if messages_len > 1:
                        messages_received = True

                # Wait before reading triggered messages
                time.sleep(0.2)
        self._logger.debug("Nb triggered messages: %s " % str(messages_len))

        if not messages_received:
            # Remove the triggered message
            self._device_logger.remove_trigger_message(tag_op_code)

            # Add opcode in the front of error message in order to retrieve the
            # related command which cause the exception
            error_message = "(%s) " % (str(tag_op_code),)

            # Timeout error
            if messages_len == 0:
                # If no message, that mean intent has been never received
                error_message += self.UECMD_TIMEOUT_INTENT_MSG + " after %s attempts." % attempt

            elif messages_len == 1:
                # If one message only, that mean intent log has been received but not
                # the ue command result.
                error_message += self.UECMD_TIMEOUT_RESULT_MSG

            self._logger.debug(error_message)
            raise DeviceException(DeviceException.OPERATION_FAILED, error_message)

        # messages list contains one or more elements
        # Remove the received intent message
        messages = self._remove_ack_message(messages)
        return messages

    def _internal_exec(self,
                       cmd,
                       timeout=None,
                       remove_triggered_message=True,
                       op_code=None,
                       broadcast=False,
                       instrument=False):
        """
        Internal method that execute std command on device

        :type  cmd: str
        :param cmd: cmd to be executed

        :rtype: dict
        :return: the output parameters in a dictionary.
        """
        if op_code is None:
            op_code = self._generate_key()

        tag_op_code = Base.OPCODE_MARKER + Base.PARAM_SEPARATOR + str(op_code)

        # store the last tag in order to easly retrieve it on logcat
        self._last_tag_op_code = tag_op_code

        # Check if broadcasting/start/instrument intent
        if broadcast:
            # If YES, full command sould be: adb shell am broadcast
            _cmd = self._cmd + " broadcast"
        elif instrument:
            _cmd = self._cmd + " instrument"
        else:
            # If not, full command sould be: adb shell am start
            _cmd = self._cmd + " start"

        # For instrument, the command is different
        if instrument:
            full_cmd = "%s -e '%s' '%s' %s " % (_cmd, Base.OPCODE_MARKER, op_code, cmd)
        else:
            full_cmd = "%s %s --es %s %s" % (_cmd, cmd, Base.OPCODE_MARKER, op_code)

        # Set a default timeout if none was provided
        if timeout is None:
            timeout = self._uecmd_default_timeout

        if instrument:
            messages = self.__run_command_before_timeout(full_cmd, timeout, tag_op_code, False)
        else:
            messages = self.__run_command_before_timeout(full_cmd, timeout, tag_op_code)

        # Crop the list to str
        message = messages[0]
        # Extract data from the message
        results = self._extract_results(tag_op_code, message)

        # Remove the triggered message
        if remove_triggered_message:
            self._device_logger.remove_trigger_message(tag_op_code)

        # Check the result
        if results[Base.STATUS_MARKER] != Base.SUCCESS:
            # Add opcode in the front of error message in order to retrieve the related command which cause the FAIL
            exception_msg = results.get(Base.EXCEPTION_MSG_MARKER, "")
            error_message = "%s %s %s" % (str(tag_op_code), results[Base.OUTPUT_MARKER], exception_msg)
            raise DeviceException(DeviceException.PHONE_OUTPUT_ERROR, error_message)
        else:
            return results

    def _internal_exec_multiple(self,
                                cmd,
                                timeout=None,
                                remove_triggered_message=True,
                                op_code=None,
                                broadcast=False,
                                instrument=False):
        """
        Internal method that execute std command on device and returns a I{multiple}
        result.

        :type  cmd: str
        :param cmd: cmd to be executed

        :rtype: list
        :return: a list containing dictionaries.
        E.g: ({"p1":value2, "p2":value3}, {"p1":value1, "p3":value4}, ...)
        """
        if op_code is None:
            op_code = self._generate_key()

        tag_op_code = Base.OPCODE_MARKER + Base.PARAM_SEPARATOR + str(op_code)

        # store the last tag in order to easly retrieve it on logcat
        self._last_tag_op_code = tag_op_code

        # Note: No Check if broadcasting intent here, just pass the boolean
        # Note (bis): No opcode appended to cmd here, just forward cmd
        # Set a default timeout if none was provided
        if timeout is None:
            timeout = self._uecmd_default_timeout

        header_parameters = \
            self._internal_exec(cmd, timeout, False, op_code, broadcast, instrument)
        # Get information about the result (number of result, etc.).
        count = int(header_parameters[Base.MULTIPLE_OUTPUT_MARKER])
        # Because the first line (header) is included in the
        # retrieved message list we increment the count value
        count += 1
        # Initialize the return value.
        results = []

        # Wait for the other messages
        message_len = -1
        start_time = time.time()
        current_timeout = 0
        while message_len < count and current_timeout < timeout:
            # extract the results
            messages = self.__get_message_triggered_status(tag_op_code, timeout)
            # retrieve messages length (messages must be a list).
            if isinstance(messages, list):
                message_len = len(messages)

            current_timeout = time.time() - start_time

        if current_timeout > timeout:
            # Remove the triggered message
            self._device_logger.remove_trigger_message(tag_op_code)

            # Timeout error
            # Add opcode in the front of error message in order to retrieve the
            # related command which cause the timeout error
            error_message = "(%s) %s" % (str(tag_op_code), self.UECMD_TIMEOUT_RESULT_MSG)
            raise DeviceException(DeviceException.PHONE_OUTPUT_ERROR, error_message)

        # We do not have to check the status,
        # it is done in "_internal_exec" so we go on:
        # Remove the first line (header).
        # We already extract messages in the while loop
        messages.pop(0)
        # Extract all results
        for message in messages:
            # Extract data from the message
            message_results = self._extract_results(tag_op_code, message)
            results.append(message_results)

        # Remove the triggered message
        if remove_triggered_message:
            self._device_logger.remove_trigger_message(tag_op_code)

        # Return the value
        return results

    def _internal_exec_with_retry(self,
                                  cmd, timeout=None,
                                  remove_triggered_message=True,
                                  op_code=None,
                                  broadcast=False,
                                  instrument=False,
                                  max_retry=5,
                                  uecmd_error_type=None):
        """
        Internal method that execute std command on device

        :type  cmd: str
        :param cmd: cmd to be executed

        :type  max_retry: integer
        :param max_retry: maximum times when cmd should be executed

        :type  uecmd_error_type: str
        :param uecmd_error_type: Type of the error which the control and retry to be executed
        In case the value is [ALL|all] the mechanism will retry on any detected uecommand error types

        .. seealso:: _internal_exec for more details

        :rtype: dict
        :return: the output parameters in a dictionary.
        """
        # To ensure using a str, in case of enum or anything else
        # In case of default value None, uecmd_error_type_lower will be convert to "none"
        uecmd_error_type = str(uecmd_error_type)
        # Lowers to be case unsensitive
        uecmd_error_type_lower = uecmd_error_type.lower()

        uecmd_error_message = AcsConstants.NO_ERRORS
        last_error_message = ""

        try_count = 1
        while try_count <= max_retry:

            try:
                results = self._internal_exec(cmd, timeout, remove_triggered_message, op_code, broadcast, instrument)
                # If no Exception occurred, reset uecmd_error_message to NO ERRORS and ends loop
                uecmd_error_message = AcsConstants.NO_ERRORS
                break

            # FIXME handle properly the different exception types inheriting from AcsBaseException
            except AcsBaseException as error:
                # Continue loop until execution success
                returned_uecmd_error = error.get_specific_message()

                if (returned_uecmd_error.find(self.UECMD_TIMEOUT_INTENT_MSG) != -1 or
                        returned_uecmd_error.find(self.UECMD_TIMEOUT_RESULT_MSG) != -1):
                    # Raise directly the error in case of uecommand intent or result timeout
                    # In those cases retry mechanism is already managed
                    # in the method _internal_exec
                    raise error

                if uecmd_error_type_lower == "none" or returned_uecmd_error.lower().find(uecmd_error_type_lower) != -1:
                    # Fail error due to uecmd_error_type passed in parameter
                    warning_msg = "TRY %d: Fail, following error raised (%s), retrying (max %d)" \
                        % (try_count, returned_uecmd_error, max_retry)
                    self._logger.warning(warning_msg)

                    # Add returned uecmd error message in the final error message
                    # if different from last error message
                    current_error_message = re.sub("\(%s%s.*\) " % (Base.OPCODE_MARKER, Base.PARAM_SEPARATOR),
                                                   "", returned_uecmd_error)
                    if last_error_message != current_error_message:
                        if uecmd_error_message == AcsConstants.NO_ERRORS:
                            # Override uecmd_error_message to store first detected error
                            uecmd_error_message = current_error_message
                        else:
                            # Concatenate the error messages
                            uecmd_error_message += "; %s" % current_error_message

                    try_count += 1
                    time.sleep(1)

                    # Store the last error message without opcode info
                    last_error_message = current_error_message

                else:
                    # Other critical error to be raised
                    error_msg = "TRY %d: Fail, error (%s) not detected, " \
                                "but following error raised (%s)" % (try_count,
                                                                     uecmd_error_type,
                                                                     returned_uecmd_error)
                    self._logger.error(error_msg)
                    raise error

        if uecmd_error_message != AcsConstants.NO_ERRORS:
            # That mean still fail until max_retry
            error_message = "Unable to perform command '%s' after %d try(ies), " \
                            "for the following reason(s): %s" % (str(cmd), max_retry, str(uecmd_error_message))
            self._logger.error(error_message)

            # FIXME raise exception as caught during retry loop
            raise DeviceException(DeviceException.OPERATION_FAILED, uecmd_error_message)

        else:
            # uecmd_error_type not encountered, return the results as self._internal_exec
            self._logger.debug("Command OKAY after %d try(ies), returning results..." % try_count)
            return results

    def _exec_with_retry(self, cmd, timeout=None, max_retry=None, uecmd_error_type=None):
        """
        Internal method that execute std command on device

        :type  cmd: str
        :param cmd: cmd to be executed

        :type  max_retry: integer
        :param max_retry: maximum times when cmd should be executed

        :type  uecmd_error_type: str
        :param uecmd_error_type: Type of the error which the control and retry to be executed

        .. seealso:: _exec for more details
        """
        if timeout is None:
            timeout = self._uecmd_default_timeout

        if max_retry is None:
            max_retry = self._uecmd_default_timeout / 4

        if uecmd_error_type is None:
            raise AcsToolException(AcsToolException.INVALID_PARAMETER, "No uecmd_error_type selected for retry clause")
        else:
            # To ensure using a str, in case of enum or anything else
            uecmd_error_type = str(uecmd_error_type)
            # Lowering to be case unsensitive
            uecmd_error_type_lower = uecmd_error_type.lower()

        try_count = 1
        connection_success = False
        while try_count <= max_retry:
            output = self._exec(cmd, timeout)

            if output.lower().find(uecmd_error_type_lower) != -1:
                # Fail error due to uecmd_error_type passed in parameter
                self._logger.warning('%d try Fail ("%s"), retrying (max:  %d)', try_count, uecmd_error_type, max_retry)
                time.sleep(1)
                try_count += 1
            else:
                # If not found, command is OK
                connection_success = True
                break

        if connection_success:
            # uecmd_error_type not encountered, log it as debug
            self._logger.debug('"%s" OKAY after %d tries', cmd, try_count)
        else:
            # Still "FAIL" until timeout
            self._logger.error('Unable to perform "%s" after %d tries', cmd, try_count)

        # Return the output anyway
        return output

    def listen_to_logcat(self, keys_filter, stop_condition, start_condition=None, timeout=None):
        """
        Listen to logcat, retrieving all messages from one or more tag
        until stop_condition has been found in the last retrieved message.

        .. warning:: should be used for UEcmd purpose only.

        :param keys_filter: list
        :param keys_filter: list of strings containing messages tag name
        to retrieve

        :type start_condition: str
        :param start_condition: start listening to logcat when a
        message contains this str (optionnal feature, will be ignored
        if set to None or empty str).

        :type stop_condition: str
        :param stop_condition: stop listening to logcat when a
        message contains this str.

        :type timeout: int
        :param timeout: maximum time (in second) allowed to retrieve messages
        (Optionnal parameter, uecmd default timeout is used if not set to a
        specific value).

        :rtype: tuple
        :return: Tuple containing:
        - Status as a boolean, True if stop_condition as been found,
        False otherwise
        - List containing all retrieved messages represented as str.
        """
        self._logger.debug(
            "Listening to logcat in order to retrieve specific information")
        local_timeout = timeout
        if timeout is None:
            local_timeout = self._uecmd_default_timeout
        keys_filter_list = keys_filter

        # parse keys filter if list is in str format
        if isinstance(keys_filter, str):
            keys_filter_list = keys_filter.split()

        # add messages to trigger
        for key in keys_filter_list:
            self._device_logger.add_trigger_message(key)

        end_found = False
        start_to_listen = False
        if start_condition is None or start_condition == "":
            self._logger.debug("No start condition")
            start_to_listen = True
        start_found_id = 0
        start_found_key = ""
        start_time = time.time()
        while (not end_found) and (time.time() - start_time) < local_timeout:
            for key in keys_filter_list:
                msg_list = self._device_logger.get_message_triggered_status(key)
                if msg_list is not None:
                    for msg_id in range(len(msg_list)):
                        msg = msg_list[msg_id]
                        # watch for start condition
                        if not start_to_listen and start_condition in msg:
                            start_to_listen = True
                            start_found_id = msg_id
                            start_found_key = key
                            self._logger.debug("Start condition found!")

                        # retrieve messages, excluding previous messages
                        # of the message which contains the start_condition,
                        # and having the same key
                        if key == start_found_key:
                            if start_to_listen and msg_id > start_found_id and stop_condition in msg:
                                end_found = True
                                self._logger.debug(
                                    "Stop condition found" +
                                    " (same key as start condition)!")
                                break
                        else:
                            if start_to_listen and stop_condition in msg:
                                end_found = True
                                self._logger.debug(
                                    "Stop condition found!")
                                break
        # compute result list
        msg_result_list = []
        for key in keys_filter_list:
            msg_list = self._device_logger.get_message_triggered_status(key)
            if key == start_found_key:
                if msg_list is not None:
                    msg_result_list.extend(msg_list[start_found_id:])
            elif msg_list is not None:
                msg_result_list.extend(msg_list)
        # remove triggers
        for key in keys_filter_list:
            self._device_logger.remove_trigger_message(key)
        return end_found, msg_result_list


    def _format_text_ouput(self, text):
        """
        format text returned by special uecmd, replacing special key
        found by human readable text.

        :type: str
        :param text: text to format

        :rtype: str
        :return: return formated stext
        """
        text = text.replace(self.ALTERNATIVE_FIELDS_SEPARATOR,
                            self.FIELDS_SEPARATOR
                            ).replace(self.ALTERNATIVE_PARAM_SEPARATOR, self.PARAM_SEPARATOR)
        text = text.replace(self.CARRIAGE_RETURN_MARKER, "\n")
        return text

    def is_shell_output_ok(self, output, consider_no_reply=False):
        """
        evaluate shell cmd output and return a the error message found or empty str.

        :type output: str
        :param output: the output of your shell function like  _exec

        :type consider_no_reply:boolean
        :param consider_no_reply: True if you consider no reply as an error
        :rtype: boolean
        :return: True if no error, False otherwise
        """
        for error_msg in self.SHELL_FILE_PARSING_ERROR:
            if output.lower().find(error_msg.lower()) != -1:
                return False
            if consider_no_reply and output.lower().find("no response from adb") != -1:
                return False

        return True

    def _dump_screen_to_text(self, file_name="uiautomator_result.xml"):
        """
        Return an XML file in str format with a dump of the current UI hierarchy.

        :type file_name:str
        :param file_name: name of the file where will be store a the UI dumping

        :rtype: str
        :return: text representing the current screen phone
        """
        file_path = "%s/%s" % (self._device._ext_sdcard_path, file_name)  # pylint:disable=W0212

        cmd = "adb shell uiautomator dump %s" % file_path
        output = self._exec(cmd)
        if output.lower().find("not found") != -1:
            tmp_txt = "_dump_screen_to_text: conversion failed, is uiautomator present? : %s" % output
            self._logger.error(tmp_txt)
            raise DeviceException(DeviceException.OPERATION_FAILED, tmp_txt)

        cmd = "adb shell cat %s" % file_path
        output = self._exec(cmd)
        if not self.is_shell_output_ok(output):
            tmp_txt = "_dump_screen_to_text: conversion failed during file generation : %s" % output
            self._logger.error(tmp_txt)
            raise DeviceException(DeviceException.OPERATION_FAILED, tmp_txt)

        return output.strip()

    def is_uiautomator_present(self):
        """
        Return if uiautomator is available on the board.
        Useful for retro compatibility on usecase as uiautomator is avaible since JB.

        :rtype: boolean
        :return: True if uiautomator is available in the board, False otherwise
        """
        cmd = "adb shell uiautomator"
        output = self._exec(cmd)
        if output.lower().find("usage: uiautomator <subcommand> [options]") != -1:
            return True
        else:
            return False

    def read_xml_setting(self, xml_file, param_name):
        """
        Read a setting value in xml file
        :type xml_file : String
        :param xml_file : Path to the xml file
        :type param_name : String
        :param param_name : name of the setting to edit
        :return : the value
        @rtype: String
        """
        # Check if the file exists
        output = self._exec("adb shell test -f %s && echo 1 || echo 0" % xml_file)
        if output.isdigit() and not bool(int(output)):
            return ""

        # Pull the file
        localpath = self._device.get_report_tree().get_report_path()
        localfile = os.path.join(localpath, "tmp.xml")
        output = self._exec("adb pull %s %s" % (xml_file, localfile))
        if "does not exist" in output.lower():
            return ""

        # Open and edit
        filetree = etree.parse(localfile)
        for match in filetree.findall("*[@name='%s']" % param_name):
            return str(match.attrib['value'])
        return ""

    def update_xml_setting(self, xml_file, param_name, param_value, xml_tag=None):
        """
        Update a setting value in xml file
        :type xml_file : String
        :param xml_file : Path to the xml file
        :type param_name : String
        :param param_name : name of the setting to edit
        :type param_value : String
        :param param_value : new value for the setting
        :return : 0 if success
        @rtype: Integer
        """
        # Check if the file exists
        output = self._exec("adb shell test -f %s && echo 1 || echo 0" % xml_file)
        if output.isdigit() and not bool(int(output)):
            return False

        # Pull the file
        localpath = self._device.get_report_tree().get_report_path()
        localfile = os.path.join(localpath, "tmp.xml")
        output = self._exec("adb pull %s %s" % (xml_file, localfile))
        if "does not exist" in output.lower():
            return False

        # Open and edit
        filetree = etree.parse(localfile)
        node = filetree.findall("*[@name='%s']" % param_name)
        if len(node) > 0:
            for match in node:
                match.attrib['value'] = str(param_value)
                self._logger.debug("Setting value '%s' for parameter '%s' in file %s" %
                                   (str(param_value), param_name, xml_file))
        elif xml_tag is not None:
            # creating element to add
            base_elmt = filetree.getroot()
            child = etree.Element(xml_tag)
            child.set("name", param_name)
            child.set("value", param_value)
            # for 'setting' tags, we need to add id and package
            if "setting" in xml_tag:
                # Get the smaller id not already used
                nodes = filetree.findall("setting")
                if len(nodes):
                    list_id = [ eval(x.attrib['id']) for x in nodes if 'id' in x.attrib and x.attrib['id'].isdigit()]
                    element_id = min([ x + 1 for x in list_id if x + 1 not in list_id])
                    child.set("id", element_id)
                child.set("package", "android")
            base_elmt.append(child)
        else:
            msg = "Parameter %s doesn't exist in file %s and no tag defined to be added" % (param_name, xml_file)
            self._logger.error(msg)
        filetree.write(localfile)

        # Get the file owner
        get_owner_cmd = "adb shell stat -c %%U %s" % xml_file
        _, owner_id = self._device.run_cmd(get_owner_cmd, timeout=5, silent_mode=True)

        # Push back
        self._exec("adb push %s %s" % (localfile, xml_file))

        # Restore ownership
        restore_owner_cmd = "adb shell chown %s %s" % (owner_id.strip(), xml_file)
        self._device.run_cmd(restore_owner_cmd, timeout=5, silent_mode=True)

        return True
