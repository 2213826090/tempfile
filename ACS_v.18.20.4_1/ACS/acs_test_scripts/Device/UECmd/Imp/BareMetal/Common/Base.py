"""
@copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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

:organization: INTEL NDG SW
:summary: UECmd based file for all BareMetal Application Framework Base
:since: 13/03/2014
:author: jreynaux
"""
from Device.UECmd.Imp.BareMetal.Common.PyCmdsUtilities import PyCmdsUtilities
from acs_test_scripts.Device.UECmd.Imp.BareMetal.Common.AndroidAgentBase import AndroidAgentBase as AndroidBase
import time
from ErrorHandling.DeviceException import DeviceException


class Base(AndroidBase):
    """
    Base class containing interactions methods for Clark device.

    Commonly use UART connection and Android App
    """
    def __init__(self, device):
        super(Base, self).__init__(device)

        self._device = device
        self._logger = self._device.get_logger()
        self._uecmd_default_timeout = int(self._device.get_uecmd_timeout())

        self._wait_btwn_cmd = self._device.get_config("waitBetweenCmd", 5, float)
        serial_port = self._device.get_config("serialPort", "/dev/ttyUSB0", str)
        self._py_cmds_api = PyCmdsUtilities(self._logger, serial_port)

    def _internal_exec(self, cmd, timeout=5, remove_triggered_message=True,
                       op_code=None, broadcast=True, instrument=False):
        """
        Internal method that execute std command on device
        :type  cmd: str
        :param cmd: cmd to be executed

        :type timeout: int
        :param timeout: timeout for execution

        :raise DeviceException if an error occurs during command execution
        :raise AcsConfigException if try to run command via UART but no serial connection configured

        :rtype: tuple (bool, str/tuple/dict)
        :return Output status and output log
        """
        silent_mode = False if self._logger else True

        # If no broadcast mean the command is executed directly on UART (pyCmds)
        if not broadcast and "com.acs.action" not in cmd:

            if not self._device.retrieve_serial_trace:
                output = 'No UART connection used, unable to run pyCmds command type'
                self._logger.error(output)
                return ()

            if not silent_mode:
                self._logger.debug("(Base) Run command on UART directly ...")

            status, output = self._py_cmds_api.do_py_cmd_command(cmd=cmd, silent_mode=silent_mode)

        else:
            if not silent_mode:
                self._logger.debug("(Base) Run command on Remote ClarkTestApp ...")
            # Need to override each parameters because may be called from super method _internal_exec_multiple
            output = AndroidBase._internal_exec(self, cmd=cmd, timeout=timeout,
                                                remove_triggered_message=remove_triggered_message,
                                                op_code=op_code, broadcast=True, instrument=False)

        return output

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

        :type timeout: int
        :param timeout: timeout for execution

        :raise DeviceException if an error occurs during command execution
        :raise AcsConfigException if try to run command via UART but no serial connection configured

        :rtype: tuple (bool, str/tuple/dict)
        :return Output status and output log
        """
        silent_mode = False if self._logger else True

        # If no broadcast mean the command is executed directly on UART (pyCmds)
        if not broadcast and "com.acs.action" not in cmd:
            if not silent_mode:
                self._logger.debug("Run command on UART directly ...")
            status, output = self._py_cmds_api.do_py_cmd_command_with_retry(cmd=cmd, max_retry=max_retry,
                                                                            silent_mode=silent_mode)
        else:
            if not silent_mode:
                self._logger.debug("Run command on Remote ClarkTestApp ...")
            # Need to override each parameters because may be called from super method _internal_exec_multiple
            output = AndroidBase._internal_exec_with_retry(cmd=cmd, timeout=None,
                                                              remove_triggered_message=True,
                                                              op_code=None,
                                                              broadcast=False,
                                                              instrument=False,
                                                              max_retry=5,
                                                              uecmd_error_type=None)

        return output

    def _remove_ack_message(self, messages):
        """
        Internal method that is used to remove the message indicating
        that the intent has been received

        :type  messages: list
        :param messages: List of str key for each result sent from Agent

        :rtype: int
        :return: 0 if an ack message have been found, -1 otherwise
        :rtype: list
        :return: a list containing str dictionaries but without acknowledgment message
        """
        index = -1
        for message in messages:
            if ((message.find(Base.STATUS_MARKER + Base.PARAM_SEPARATOR + Base.SUCCESS) != -1) and
                    (message.find(Base.OUTPUT_MARKER + Base.PARAM_SEPARATOR + Base.RECEIVED) != -1)):
                index = messages.index(message)
                self._logger.debug("Acknowledgment message found (index={0}): {1}".format(index, message))
                if index != 0:
                    self._logger.warning("The acknowledgment message was not the First " +
                                         "Received, please check your build's known issues")
                break

        if index != -1:
            self._logger.debug("Remove acknowledgment message")
            messages.pop(index)
            return 0, messages
        else:
            #self._logger.info("Unable to find Acknowledgment message on current results")
            return -1, messages

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
                status_remove, messages = self._remove_ack_message(messages)
                while status_remove == 0:
                    if len(messages) == 0:
                        self._logger.debug("No more received message !")
                        break
                    self._logger.debug("An ack message has been found, try to find another one")
                    status_remove, messages = self._remove_ack_message(messages)
                #self._logger.debug("Message list: {0}".format(messages))

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
        self._logger.debug("Receive message list: {0}".format(messages))
        status_remove, messages = self._remove_ack_message(messages)
        while status_remove == 0:
            if len(messages) == 0:
                self._logger.debug("No more received message !")
                break
            self._logger.debug("An ack message has been found, try to find another one")
            status_remove, messages = self._remove_ack_message(messages)
        return messages

    # TODO: Find a convenient way to do and to initialize agent
    # def finalize(self):
        # self._stop_acs_service()
        # self._py_cmds_api.unload_pycmds()
