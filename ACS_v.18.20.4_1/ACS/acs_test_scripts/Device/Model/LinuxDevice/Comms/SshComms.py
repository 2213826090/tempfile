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
:summary: This file implements the YoctoDevice SSH Communication
:since: 26 October 2015
:author: vgomberx
"""

from ErrorHandling.DeviceException import DeviceException
import paramiko
from scp import SCPClient
from threading import Thread
import weakref


class SshComms:
    """
    object dedicated to ssh communication with the Yocto device
    """
    TLNT_PROMPT = ":/#"
    TLNT_PATH_SEP = "\\"
    TLNT_CARRIAGE_RETURN = "\n"

    def __init__(self, dut_ip, user="", password="", logger=None):
        """
        """
        self.__protocol = None
        self.__dut_ip = dut_ip
        self.__user = user
        self.__password = password
        self._logger = logger

    def connect(self, connection_timeout=10):
        """
        Establish a ssh connection if it was not opened or working
        """
        if self.__protocol is None or not self.is_connected():
            self._logger.info("Connecting to DUT")

            # close previous connection
            if self.__protocol is not None:
                self.__protocol.close()

            # open the ssh connection
            try :
                self.__protocol = None
                self.__protocol = paramiko.SSHClient()
                self.__protocol.load_system_host_keys()
                self.__protocol.set_missing_host_key_policy(paramiko.AutoAddPolicy())

                self.__protocol.connect(hostname=self.__dut_ip,
                                     username=self.__user,
                                     password=self.__password,
                                     timeout=connection_timeout)
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
        test if the ssh connection is opened
        :rtype: bool
        :return: True if the ssh communication is opened and works
        """
        result = False
        if self.__protocol is not None:
            try:
                # send a simple command to see if connection is really up
                result = self.__protocol.get_transport() is not None and self.__protocol.get_transport().is_active()

            except Exception:
                pass

        return result

    class sshCmdResponse(Thread):

        def __init__(self, std_out):
            """
            handle ssh return code to wait for the end of a cmd without
            having blocking problem
            """
            Thread.__init__(self)
            self.__result = None
            self.__std_out = weakref.proxy(std_out)

        def wait_for_response(self):
            """
            wait cmd end, this is a blocking call
            """
            return self.__std_out.channel.recv_exit_status()

        def run(self):
            """
            :TODO: put a lock system to interact with __result
            """
            self.__result = self.wait_for_response()

        def get_return_code(self):
            return self.__result

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
        msg = "SSH communication protocol is not opened!"
        verdict = False
        try:
            if self.__protocol is not None:
                std_out_data = None
                std_err_data = None

                _, ssh_std_out, ssh_std_err = self.__protocol.exec_command(cmd)

                if not wait_for_reply:
                    verdict = True
                    msg = "not waiting for cmd return"
                else:
                    error_code = None
                    # proceed to response acquisition
                    if ssh_std_out is not None:
                        t = SshComms.sshCmdResponse(ssh_std_out)
                        t.start()
                        error_code = t.get_return_code()
                        # start thread only if it is necessary
                        if error_code is None:
                            t.join(timeout)
                            error_code = t.get_return_code()

                        # get standard output
                        if error_code is not None:
                            std_out_data = str(ssh_std_out.read()).rstrip()
                        else:
                            ssh_std_out.channel.close()

                        del t
                    # get error output
                    if ssh_std_err is not None:
                        if error_code is not None:
                            std_err_data = str(ssh_std_err.read()).rstrip()
                        else:
                            ssh_std_err.channel.close()

                    # compute verdict and returned msg
                    verdict = error_code == 0

                    if std_out_data is not None or std_err_data is not None:
                        msg = ""
                        if std_out_data is not None:
                            msg += std_out_data
                        if std_err_data is not None:
                            msg += "\n" + std_err_data
                        msg = msg.strip("\n")
                    else:
                        msg = "Command [%s] did not end before %ss, if its a blocking command try to skip waiting its reply" % (cmd, timeout)


        except Exception as e:
            msg = "Error when executing command [%s] : %s" % (cmd, str(e))
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
        msg = "fail to push file %s onto %s, no opened connection" % (src_path, dest_path)
        if self.__protocol is not None:
            try:
                scp_client = SCPClient(self.__protocol.get_transport())
                scp_client.put(src_path, dest_path)
                verdict = True
                msg = "Pushed file %s onto %s" % (src_path, dest_path)
            except Exception as ex:
                msg = "Error happen when pushing file %s onto %s :%s" % (src_path, dest_path, str(ex))
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
        msg = "fail to pull file %s onto %s, no opened connection" % (src_path, dest_path)
        if self.__protocol is not None:
            try:
                scp_client = SCPClient(self.__protocol.get_transport())
                scp_client.get(src_path, dest_path, recursive=True, preserve_times=True)
                verdict = True
                msg = "Pulled file %s onto %s" % (src_path, dest_path)
            except Exception as ex:
                msg = "Error happen when pulling file %s onto %s :%s" % (src_path, dest_path, str(ex))

        return verdict, msg
