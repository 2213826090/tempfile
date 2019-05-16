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

:organization: INTEL NDG SW DEV
:summary: Utilities class for SSH implementation
:author: jreynaux
:since: 19/08/2010
"""
from os import path
import socket
import sys
from UtilitiesFWK.Utilities import internal_shell_exec, Global


class SshUtilities:
    """
    Utility class containing interactions methods with SSH to device.
    """

    SSH_CONNECTION_ESTABLISHMENT_FAILED_ERROR_MSG = "Unable to establish ssh connection with DUT"
    SSH_CONNECTION_GENERIC_ERROR_MSG = "An error occurred during ssh connection with DUT"
    SSH_GENERIC_ERROR = "SSH Generic error"

    def __init__(self, logger=None):
        self._client = None
        self._logger = logger

        self._user = ""
        self._password = ""
        self._address = "127.0.0.1"

    def configure_client(self):
        """
        Configure a client that will be used for SSH interaction

        :return: None
        """
        import paramiko

        self._client = None
        self._client = paramiko.SSHClient()
        self._client.load_system_host_keys()
        self._client.set_missing_host_key_policy(paramiko.AutoAddPolicy())

    def configure_credentials(self, address, user, password):
        """
        Configure credential used during SSH connection.

        :type  address: str
        :param address: IP address of the device

        :type  user: str
        :param user: User to use for ssh connection

        :type  password: str
        :param password: Password to use for ssh connection

        :return: None
        """
        self._address = address
        self._user = user
        self._password = password

    def clean_user_known_host_file(self, ip_address):
        """
        Clean user known host file. (Linux benches)

        To ensure better compatibility in case of StrictHostKeyChecking=yes on PC.
        And avoid ssh connection aborting after a flash (server keys should change)
        """
        if 'posix' in sys.builtin_module_names:
            self._logger.warning("Clean user know_host file to ensure better ssh compatibility")
            known_host_file = path.expanduser("~/.ssh/known_hosts")
            cmd = "ssh-keygen -f {0} -R {1}".format(known_host_file, ip_address)
            status, value = internal_shell_exec(cmd=cmd, timeout=5, silent_mode=False)
            if status == Global.SUCCESS:
                self._logger.warning("known_host clean done.")
        else:
            self._logger.debug("Not a linux bench, no known_hosts file to clean")

    def force_dut_ssh_key_generation(self, ip_address, timeout=10):
        """
        Clean user known host file. (Linux benches)

        To ensure better compatibility in case of StrictHostKeyChecking=yes on PC.
        And avoid ssh connection aborting after a flash (server keys should change)
        """
        if 'posix' in sys.builtin_module_names:
            cmd = "ssh-keyscan -T {0} {1}".format(timeout, ip_address)
            status, value = internal_shell_exec(cmd=cmd, timeout=5, silent_mode=False)
            if status == Global.SUCCESS:
                self._logger.warning("ssh-keyscan successfully returned.")
        else:
            self._logger.debug("Not a linux bench")

    def ssh_run_cmd(self, cmd, timeout=5, silent_mode=False, wait_for_response=True):
        """
        Run a cmd via SSH to interact with a device

        :type cmd: str or list
        :param cmd: The command(s) to execute on remote device

        :type  timeout: int
        :param timeout: Script execution timeout in ms

        :type silent_mode: bool
        :param silent_mode: Flag indicating some information are display as log.

        :type wait_for_response: bool
        :param wait_for_response: Wait response from the device before stating on command

        :rtype: tuple (bool, str)
        :return: Execution status & output string
        """
        cmd_succeed = False
        output = self.SSH_GENERIC_ERROR

        # If no logger for silent_mode to True
        if self._logger is None:
            silent_mode = True

        if not silent_mode:
            self._logger.debug("Execute SSH command(s): " + str(cmd))

        if self._client is None:
            self.configure_client()

        if type(cmd) is not list:
            commands = [cmd]

        for command in commands:
            try:

                cmd_succeed, output = self._do_ssh_command(command, timeout, silent_mode, wait_for_response)

            except socket.timeout as ex:
                cmd_succeed = False
                output = self.SSH_CONNECTION_ESTABLISHMENT_FAILED_ERROR_MSG + " (%s)" % ex
                if not silent_mode:
                    self._logger.error(output)

            except Exception as e:
                cmd_succeed = False
                output = self.SSH_CONNECTION_GENERIC_ERROR_MSG + " (%s)" % e
                if not silent_mode:
                    self._logger.error(output)

        self._client.close()
        return cmd_succeed, output

    def _do_ssh_command(self, command, timeout, silent_mode, wait_for_response):
        """
        Send to SSH command to client and compute result.

        :type command: str
        :param command: The (linux standard) command to be sent

        :type timeout: int
        :param timeout: Time Out to wait for sending ssh command

        :type silent_mode: bool
        :param silent_mode: Flag indicating some information are display as log.

        :type wait_for_response: bool
        :param wait_for_response: Wait response from the device before stating on command

        :rtype: tuple (int, str)
        :return: Execution status & output string
        """
        # If no logger for silent_mode to True
        if self._logger is None:
            silent_mode = True

        std_out_data = None
        std_err_data = None
        cmd_succeed = False
        output = ""

        self._client.connect(hostname=self._address,
                             username=self._user,
                             password=self._password,
                             timeout=timeout)

        ssh_std_in, ssh_std_out, ssh_std_err = self._client.exec_command(command)

        if not wait_for_response:
            self._logger.debug("Command does not wait for a response")
            return True, output

        # proceed to response acquisition
        if ssh_std_out is not None:
            cmd_succeed = True
            if std_out_data is None:
                std_out_data = ssh_std_out.read().rstrip()
            else:
                std_out_data += ssh_std_out.read().rstrip() + "/n"

        if not silent_mode and std_out_data not in ["", None]:
            self._logger.debug("SSH Output:>%s<" % str(std_out_data))

        if ssh_std_err is not None:
            if std_err_data is None:
                std_err_data = ssh_std_err.read().rstrip()
            else:
                std_err_data += ssh_std_err.read().rstrip() + "/n"

        if not silent_mode and std_err_data not in ["", None]:
            self._logger.debug("SSH Error(s):>%s<" % str(std_err_data))

        error_code = ssh_std_out.channel.recv_exit_status()
        if not silent_mode:
            self._logger.debug("Command error code: {0}".format(error_code))

        cmd_succeed = False if error_code != 0 else True

        output = "%s %s" % (std_out_data, std_err_data) if (
            std_out_data and std_err_data) else (std_out_data or std_err_data)

        return cmd_succeed, output

    def ssh_push_file(self, src_path, dest_path, timeout=30, silent_mode=False):
        """
        Push file on dut via SCP

        :param src_path: str
        :param src_path: Source path of file

        :type dest_path: str
        :param dest_path: Destination path on DUT

        :type timeout: int
        :param timeout: Timeout in second

        :rtype: bool
        :return: Boolean indicating operation succeed or not
        """
        from scp import SCPClient

        # If no logger force silent_mode to True
        if self._logger is None:
            silent_mode = True

        done = False
        try:
            if not silent_mode:
                self._logger.debug('Opening ssh client ...')

            self._client.connect(hostname=self._address,
                                 username=self._user,
                                 password=self._password,
                                 timeout=timeout)

            scp_client = SCPClient(self._client.get_transport())
            if not silent_mode:
                self._logger.debug('Pushing file %s to %s ...' % (src_path, dest_path))

            scp_client.put(src_path, dest_path)

            done = True
        except Exception as ex:
            if not silent_mode:
                self._logger.error(str(ex))
        finally:
            if self._client is not None:
                if not silent_mode:
                    self._logger.debug('Closing ssh client ...')
                self._client.close()

        return done


    def ssh_pull_file(self, src_path, dest_path, timeout=30, silent_mode=False):
        """
        Pull file from dut via SCP

        :param src_path: str
        :param src_path: Source path of file on DUT

        :type dest_path: str
        :param dest_path: Destination path on HOST

        :type timeout: int
        :param timeout: Timeout in second

        :rtype: bool
        :return: Boolean indicating operation succeed or not
        """
        from scp import SCPClient

        # If no logger force silent_mode to True
        if self._logger is None:
            silent_mode = True

        done = False
        try:
            if not silent_mode:
                self._logger.debug('Opening ssh client ...')

            self._client.connect(hostname=self._address,
                                 username=self._user,
                                 password=self._password,
                                 timeout=timeout)

            scp_client = SCPClient(self._client.get_transport())
            if not silent_mode:
                self._logger.debug('Getting file %s to %s ...' % (src_path, dest_path))

            scp_client.get(src_path, dest_path, recursive=True, preserve_times=True)

            done = True
        except Exception as ex:
            if not silent_mode:
                self._logger.error(str(ex))
        finally:
            if self._client is not None:
                if not silent_mode:
                    self._logger.debug('Closing ssh client ...')
                self._client.close()

        return done
