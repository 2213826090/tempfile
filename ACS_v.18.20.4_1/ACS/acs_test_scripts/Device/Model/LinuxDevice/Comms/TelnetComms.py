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
:summary: This file implements the YoctoDevice Telnet Communication
:since: 15 October 2015
:author: vgomberx
"""

from ErrorHandling.DeviceException import DeviceException
from telnetlib import Telnet, IAC, NOP
import re, socket


class TelnetComms:
    """
    object dedicated to telnet communication with the Yocto device
    """
    TLNT_PROMPT = ":/#"
    TLNT_PATH_SEP = "\\"
    TLNT_CARRIAGE_RETURN = "\n"

    def __init__(self, dut_ip, telnet_port=None, logger=None):
        """
        """
        self.__protocol = None
        self.__dut_ip = dut_ip
        self.__telnet_port = telnet_port
        self._logger = logger

    def connect(self, connection_timeout=10):
        """
        Establish a telnet connection if it was not opened or working
        """
        if self.__protocol is None or not self.is_connected():
            self._logger.info("Connecting to DUT")

            # close previous connection
            if self.__protocol is not None:
                self.__protocol.close()

            # open the telnet connection
            try :
                self.__protocol = Telnet()
                if self.__telnet_port not in ["", None]:
                    self.__protocol.open(self.__dut_ip, self.telnet_port, timeout=connection_timeout)
                else:
                    self.__protocol.open(self.__dut_ip, timeout=connection_timeout)
                # wait for prompt return
                output = self.read_info(10)
                if output != "":
                    self._logger.info(output)

            except Exception as e:
                self.__protocol = None
                msg = "error happen when trying to connect to the target: %s" % str(e)
                self._logger.error(msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

    def release(self):
        """
        Releases the connection
        """
        self._logger.info("Release connection")
        if self.__protocol is not None:
            try :
                self.__protocol.close()
            finally:
                self.__protocol = None

    def is_connected(self):
        """
        test if the telnet connection is opened
        :rtype: bool
        :return: True if the telnet communication is opened and works
        """
        result = False
        if self.__protocol is not None:
            try:
                # send a simple command to see if connection is really up
                result = self.send_command(IAC + NOP)
            except socket.error:
                pass

        return result

    def send_command(self, cmd):
        """
        send a command through telnet

        :type cmd: str
        :param cmd: command to execute

        :rtype: boolean
        :return: True if the command was successfully sent
        """
        result = False
        if self.__protocol is not None:
            # first clean the log queue
            unused_info = self.__protocol.read_very_eager()
            sent_cmd = cmd.strip()
            if not sent_cmd.endswith(self.TLNT_CARRIAGE_RETURN):
                sent_cmd += self.TLNT_CARRIAGE_RETURN
            self.__protocol.write(sent_cmd)
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
        :return: the last available info from telnet prompt
        """
        msg = None
        if self.__protocol is not None:
            msg = self.__protocol.read_until(self.TLNT_PROMPT, timeout).strip()
            if len(msg) > 0:
                # remove terminal control character
                ansi_escape = re.compile(r'\x1b[^m]*m')
                # split and clean
                msg = ansi_escape.sub('', msg)
                split_msg = map(str.strip, msg.split("\n"))

                # remove the last sent cmd on prompt on the first line
                if cmd is not None and split_msg[0].startswith(cmd):
                    split_msg[0] = split_msg[0].replace(cmd, "", 1)

                # remove the prompt message
                if split_msg[-1].endswith(self.TLNT_PROMPT):
                    split_msg.pop()

                # joint the message and clean again
                split_msg = filter(None, split_msg)
                msg = "\n".join(split_msg)
                msg = msg.strip()

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
        verdict = False
        try:
            verdict = self.send_command(cmd)
            if not verdict:
                msg = "error when executing command [%s], TELNET communication is not opened" % cmd
            elif wait_for_reply:
                msg = self.read_info(timeout, cmd)
                if msg is None:
                    verdict = False
                    msg = "error when reading command [%s] reply: TELNET communication was not opened" % cmd
        except Exception as e:
            msg = "error when executing command [%s] : %s" % (cmd, str(e))
            verdict = False

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
        msg = "push file not implemented through Telnet"
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
        msg = "pull file not implemented through Telnet"
        return verdict, msg