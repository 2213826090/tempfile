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

:organization: INTEL OPM PCWTE
:summary: This file implements the YoctoDevice Serial Communication
:since: 20 October 2015
:author: vgomberx
"""
from ErrorHandling.DeviceException import DeviceException
import serial
import time


class SerialComms:
    """
    object dedicated to serial communication with the Yocto device
    """

    def __init__(self, com_port, baud_rate, retry_nb=3, logger=None):
        """
        :type com_port: int
        :param com_port: com port number to connect with
        :type baud_rate: int
        :param baud_rate: baud rate for the serial connection
        :type retry_nb: int
        :param retry_nb: Number of serial connection attempt
        """
        # Initialize local variables
        self.__protocol = None
        self.__retry = retry_nb
        # dirty way to filter  wrong type
        self.__baudrate = int(baud_rate)
        self.__com_port = com_port
        self._logger = logger

    def connect(self, connection_timeout=10):
        """
        open the connection if it was not already opened and working
        """
        if self.__protocol is None or not self.is_connected():
            # close previous connection
            if self.__protocol is not None:
                self.__protocol.close()
            self.__protocol = None
            handle = None
            try_index = 0
            # Try several times
            while try_index < self.__retry:
                try:
                    handle = serial.Serial(port=self.__com_port,
                                           baudrate=self.__baudrate,
                                           timeout=connection_timeout)
                    break
                except serial.SerialException as error:
                    try_index += 1
                    if try_index < self.__retry:
                        self._logger.error(str(error))
                        self._logger.info("Retry the connection...")
                        time.sleep(1)
                    else:
                        msg = "error happen when trying to connect to the COM port %s: %s" % (self.__com_port, str(error))
                        self._logger.error(msg)
                        raise DeviceException(DeviceException.OPERATION_FAILED, msg)

            if handle is None:
                msg = "Serial port connection to %s failed." % self.__com_port
                self._logger.error(msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)
            else:
                # Update handle value
                self.__protocol = handle
                self.__protocol.flushInput()
                self.__protocol.flushOutput()

    def release(self):
        """
        Releases the connection
        """
        if self.__protocol is not None:
            self._logger.info("Release serial connection")
            try:
                self.__protocol.close()
            finally:
                # Update handle value
                self.__protocol = None

    def is_connected(self):
        """
        test if the serial connection is opened
        :rtype: bool
        :return: True if the serial communication is opened and works
        """
        result = False
        if self.__protocol is not None and self.__protocol.isOpen():
            cmd = "AT"
            result = self.send_command(cmd)
            if result and self.read_info(cmd) in ["", None]:
                result = False

        return result

    def send_command(self, cmd):
        """
        send a command through serial

        :type cmd: str
        :param cmd: command to execute

        :rtype: boolean
        :return: True if the command was successfully sent
        """
        result = False
        if self.__protocol is not None:
            self.__protocol.flushInput()
            self.__protocol.flushOutput()
            time.sleep(0.2)
            self.__protocol.write(cmd + "\r\n")
            result = True
        return result

    def read_info(self, timeout=10, cmd=None):
        """
        read all info until prompt or timeout is reached
        :type cmd: str
        :param cmd: last command executed, this is used to clean the reply

        :type timeout: int
        :param timeout: timeout to read the command reply

        :rtype: string
        :return: the last available info from serial port
        """
        msg = None
        if self.__protocol is not None:
            msg_list = []
            # get the first reply , often the sent command
            next_response = self.__protocol.readline().rstrip('\r\n').replace("\r", "").strip()
            if len(next_response) > 0:
                msg_list.append(next_response)

            start_time = time.time()
            while self.__protocol.inWaiting():
                # mega clean
                next_response = self.__protocol.readline().rstrip('\r\n').replace("\r", "").strip()
                if len(next_response) > 0:
                    msg_list.append(next_response)

                if start_time - time.time() > timeout:
                    error_msg = "Timeout exceed : not complete reply has been seen after %ss" % timeout
                    self._logger.error(error_msg)
                    break

            # reconstruct the message
            if len(msg_list) > 0:
                if cmd is not None and msg_list[0].startswith(cmd):
                    msg_list[0] = msg_list[0].replace(cmd, "", 1)

                msg_list = filter(None, msg_list)
                msg = "\n".join(msg_list)

        return msg

    def execute_cmd(self, cmd, timeout=60, wait_for_reply=True):
        """
        execute a command and wait for its reply if necessary

        :type cmd: str
        :param cmd: command to execute

        :type timeout: int
        :param timeout: timeout to read the command reply

        :type wait_for_reply: bool
        :param wait_for_reply: if True, skip the cmd reply reading

        :rtype: tuple
        :return: ( boolean True if the command was successfully executed, cmd reply)
        """
        msg = "not waiting for cmd return"
        verdict = self.send_command(cmd)
        if not verdict:
            msg = "error when executing command [%s]: SERIAL communication is not opened" % cmd
        elif wait_for_reply:
            msg = self.read_info(timeout, cmd)
            if msg is None:
                verdict = False
                msg = "error when reading command [%s] reply: SERIAL communication was not opened" % cmd

        return verdict, msg

    def __del__(self):
        """"
        ensure that we release the connection if destroyed
        """
        self.release()

    def push(self, src_path, dest_path, push_timeout=None):
        """
        Push file on dut

        :param src_path: str
        :param src_path: Source path of file

        :type dest_path: str
        :param dest_path: Destination path on DUT

        :type push_timeout: int
        :param push_timeout: timeout to push the file

        :rtype: tuple
        :return: Boolean indicating operation succeed or not, message
        """
        verdict = False
        msg = "push file not implemented through Serial"
        return verdict, msg

    def pull(self, src_path, dest_path, pull_timeout=None):
        """
        Pull file from dut

        :param src_path: str
        :param src_path: Source path of file on DUT

        :type dest_path: str
        :param dest_path: Destination path on HOST

        :type pull_timeout: int
        :param pull_timeout: timeout to pull the file

        :rtype: tuple
        :return: Boolean indicating operation succeed or not, message
        """
        verdict = False
        msg = "pull file not implemented through Serial"
        return verdict, msg