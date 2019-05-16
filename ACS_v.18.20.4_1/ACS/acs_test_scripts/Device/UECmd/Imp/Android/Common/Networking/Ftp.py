"""
:copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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
:summary: This file implements FTP UECmds for telephony use cases
:since: 14/11/2014
:author: mbrisbax
"""
import os
from acs_test_scripts.Device.UECmd.Imp.Android.Common.BaseV2 import BaseV2
from acs_test_scripts.Device.UECmd.Interface.Networking.IFtp import IFtp
from ErrorHandling.AcsBaseException import AcsBaseException
import acs_test_scripts.Device.UECmd.UECmdTypes as UECmdTypes


class Ftp(BaseV2, IFtp):
    def __init__(self, device):
        """
        Constructor
        """
        BaseV2.__init__(self, device)
        IFtp.__init__(self, device)
        self._logger = device.get_logger()
        self._phone_system = device.get_uecmd("PhoneSystem")
        self.__file_uecmds = device.get_uecmd("File")

        self.ftp_module = "acscmd.transfer.FtpModule"
        self.ftp_threaded_module = "acscmd.transfer.FtpThreadedModule"

    def start_ftp(self, direction, server_ip_address, username, password, remote_file, local_path, dut_ip_address, local_file=None):
        """
        Start a ftp transfer

        :type direction: str
        :param direction: Transfer direction (UL/DL)

        :type server_ip_address: str
        :param server_ip_address: IP address of the FTP server

        :type username: str
        :param username: FTP account username

        :type password: str
        :param password: FTP account password

        :type remote_file: str
        :param remote_file: file path on FTP server

        :type local_path: str
        :param local_path: path where to read/save the FTP file (sdcard)

        :type dut_ip_address: str
        :param dut_ip_address: IP address of the DUT networking interface we want to use for FTP
        This parameter is only used on windows (it is added here for compatibility)

        :type local_file: str
        :param local_file: file name on DUT

        :rtype: long
        :return: id of the ftp transfer
        """
        # Check if the directory structure from 'local_path' is valid
        local_dir = os.path.dirname(local_path)
        if self._phone_system.check_directory_exist_from_shell(local_dir) is False:
            self._logger.info("Local path is not valid, create it")
            self._phone_system.create_directory_from_shell(local_dir)

        if local_file is None:
            local_file = os.path.join(local_path, os.path.basename(remote_file)).replace("\\", "/")
        else:
            local_file = os.path.join(local_path, local_file).replace("\\", "/")
        self._logger.info("local file %s remote file %s" % (local_file, remote_file))
        if direction == UECmdTypes.XFER_DIRECTIONS.UL:
            # Preliminary analysis : create file to upload if not
            # already present in the device (IF POSSIBLE)
            try:
                size_kb = self.__file_uecmds.retrieve_size_from_filename(local_file)
                self.__file_uecmds.create_file_if_needed(local_file, size_kb, os.path.dirname(local_file))
                self.__file_uecmds.set_file_permission(local_file, "777")
            except AcsBaseException as excp:
                self._logger.info(
                    "Preliminary analysis failed : we won't be able to "
                    "create file if not already present in the device")
                self._logger.debug("Root cause : " + str(excp))
        elif direction == UECmdTypes.XFER_DIRECTIONS.DL:
            self._phone_system.delete(local_file, False)
        if username is "" and password is "":
            username = "anonymous"
            password = ""

        # Start ftp transfer
        method = "startFtp"
        cmd_args = "--es direction %s --es url %s" \
                   " --es username %s --es password '%s' --es localFile '%s'" \
                   % (str(direction), str(server_ip_address),
                      str(username), str(password), local_file.replace("\\", "/"))

        # Extract the ftp path and ftp file
        filename = os.path.normpath(remote_file).replace("\\", "/")
        ftp_file = os.path.basename(filename)
        ftp_path = os.path.dirname(filename)

        # Add ftp file to command to send
        cmd_args += " --es file %s" % str(ftp_file)
        # Check if the file will transferred into a specific directory
        if ftp_path != "":
            cmd_args += " --es ftpPath %s" % str(ftp_path)

        try:
            ftp_thread_id = self._internal_exec_with_retry_v2(self.ftp_threaded_module, method, cmd_args, is_system=True)
        except AcsBaseException as error:
            # Raise the uecommand exception with the root cause
            raise AcsBaseException('Error during FTP transfer')

        return ftp_thread_id["thread_id"]

    def stop_ftp(self, ftp_id):
        """
        Stops a FTP transfer

        :type ftp_id: long
        :param ftp_id: ftp transfer to stop

        :rtype: tuple(float, str, str)
        :return: ftp transfer result
                - transfer overall throughput
                - transfer status
                - transfer log
        """
        self._logger.info("Stopping FTP transfer...")

        method = "stopFtp"
        cmd_args = " --el threadId %s" % str(ftp_id)
        output = self._internal_exec_v2(self.ftp_threaded_module, method, cmd_args, is_system=True)
        if output["FTP status"].strip() != "notrunning":
            log = self._get_ftp_log_on_device(output["log"].strip())
        else:
            log = "FTP transfer number %s does not exist" % ftp_id

        return (output["throughput"].strip(), output["FTP status"].strip(), log)

    def kill_ftp(self):
        """
        Kill a FTP transfer
        """
        self._logger.info("Killing FTP transfer is not implemented on Android OS")

    def get_ftp_status(self, thread_id):
        """
        get the ftp status.

        :type thread_id: long
        :param thread_id: ftp thread id

        :rtype: str
        :return: ftp status
                - "transfer successful"
                - "connecting"
                - "transferring"
                - "notrunning"
                - "transfer failed"
        """
        self._logger.debug("Get FTP transfer status...")

        method = "getFtpStatus"
        cmd_args = " --el threadId %s" % str(thread_id)
        output = self._internal_exec_v2(self.ftp_threaded_module, method, cmd_args, is_system=True)
        self._logger.debug("Get FTP transfer status: %s" % output)

        return output["FTP status"].strip()

    def _get_ftp_log_on_device(self, log):
        log_cmd = "adb shell cat %s" % log
        del_log_cmd = "adb shell rm -f %s" % log  # read log file on
        log = self._exec(log_cmd, 5)
        self._exec(del_log_cmd, 1)
        self._logger.debug("FTP transfer log: " + log[1])
        return log[1]

    def check_file_size(self, server_ip_address, username, password, remote_file, local_file):
        """
        Check if file has the same size on DUT and FTP server.

        :type server_ip_address: str
        :param server_ip_address: IP address of the FTP server

        :type username: str
        :param username: FTP account username

        :type password: str
        :param password: FTP account password

        :type remote_file: str
        :param remote_file: file path on server

        :type local_file: str
        :param local_file: file path on DUT

        :rtype: bool
        :return: file has the same size on DUT and FTP server.
        """

        self._logger.info("Get FTP transfer results...")

        if username is "" and password is "":
            username = "anonymous"
            password = ""
        method = "getFtpSize"
        cmd_args = "--es file %s --es url %s --es username %s --es password '%s'" \
                   % (str(remote_file), str(server_ip_address),
                      str(username), str(password))

        output = self._internal_exec_with_retry_v2(self.ftp_module, method, cmd_args, is_system=True)

        server_size = int(output["size"])
        dut_size = int(self.__file_uecmds.size(local_file)) / 1000

        self._logger.debug("File size on DUT: %s - On server: %s" % (dut_size, server_size))

        return server_size == dut_size
