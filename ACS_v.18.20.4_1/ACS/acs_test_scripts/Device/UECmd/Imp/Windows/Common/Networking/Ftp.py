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
from acs_test_scripts.Device.UECmd.Imp.Windows.Common.Base import Base
from acs_test_scripts.Device.UECmd.Interface.Networking.IFtp import IFtp
from ErrorHandling.AcsBaseException import AcsBaseException
import acs_test_scripts.Device.UECmd.UECmdTypes as UECmdTypes
import unicodedata


class Ftp(Base, IFtp):
    def __init__(self, device):
        """
        Constructor
        """
        Base.__init__(self, device)
        IFtp.__init__(self, device)
        self._logger = device.get_logger()
        self._phone_system = device.get_uecmd("PhoneSystem")
        self.__file_uecmds = device.get_uecmd("File")

        self._module_name = "Intel.Acs.TestFmk.Networking"
        self._class_name = "Intel.Acs.TestFmk.Networking.FtpActivity"
        # Dictionary where FTP transfer Ids and process Ids are stored
        self._ftp_transfer_ids = {}

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
        self._logger.info("local path %s remote file %s" % (local_path, remote_file))
        local_file = os.path.join(local_path, os.path.basename(remote_file))
        if direction == UECmdTypes.XFER_DIRECTIONS.UL:
            # Preliminary analysis : create file to upload if not
            # already present in the device (IF POSSIBLE)
            try:
                size_kb = self.__file_uecmds.retrieve_size_from_filename(local_file)
                self.__file_uecmds.create_file_if_needed(local_file, size_kb, os.path.dirname(local_file))
            except AcsBaseException as excp:
                self._logger.info(
                    "Preliminary analysis failed : we won't be able to "
                    "create file if not already present in the device")
                self._logger.debug("Root cause : " + str(excp))
        elif direction == UECmdTypes.XFER_DIRECTIONS.DL:
            self._phone_system.delete(local_file, False)
        if username is "" and password is "":
            username = "anonymous"
            password = "ftptest"

        # Extract the ftp path and ftp file
        filename = os.path.normpath(remote_file)
        local_file = os.path.join(os.path.normpath(local_path), os.path.basename(filename))

        # Start ftp transfer
        method = "StartFtp"
        cmdArgs = "direction=%s server_ip_address=%s username=%s password=%s remote_filename=%s local_filename=%s client_ip_address=%s" \
            % (str(direction), str(server_ip_address),
               str(username), str(password), str(filename), str(local_file), dut_ip_address)

        try:
            # Get the method and class name of the UEcommand on the embedded side
            module_name, class_name = self._get_module_and_class_names()
            ftp_thread_id = self._internal_uecmd_exec(module_name, class_name, method, cmdArgs)
        except AcsBaseException as error:
            # Raise the uecommand exception with the root cause
            raise error
        # Store FTP Id in dictionary
        self._ftp_transfer_ids[long(ftp_thread_id["values"]["ftpId"])] = long(ftp_thread_id["values"]["ftpProcessId"])
        return long(ftp_thread_id["values"]["ftpId"])

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

        method = "GetFtpStatus"
        cmdArgs = " ftpId=%s" % str(thread_id)
        # Get the method and class name of the UEcommand on the embedded side
        module_name, class_name = self._get_module_and_class_names()
        output = self._internal_uecmd_exec(module_name, class_name, method, cmdArgs)
        self._logger.debug("Get FTP transfer status: %s" % output)

        return unicodedata.normalize('NFKD', output["values"]["status"]).encode('ascii', 'ignore').strip()

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
        # Get FTP transfer process Id from FTP Id
        if ftp_id in self._ftp_transfer_ids:
            ftp_process_id = self._ftp_transfer_ids[ftp_id]
            del self._ftp_transfer_ids[ftp_id]
        else:
            self._logger.error("FTP transfer Id does not exist: %s" % ftp_id)
            raise AcsBaseException(AcsBaseException.INVALID_PARAMETER, "FTP transfer Id does not exist: %s" % ftp_id)
        method = "StopFtp"
        cmdArgs = " ftpId=%s ftpProcessId=%s" % (str(ftp_id), str(ftp_process_id))
        # Get the method and class name of the UEcommand on the embedded side
        module_name, class_name = self._get_module_and_class_names()
        output = self._internal_uecmd_exec(module_name, class_name, method, cmdArgs)
        return (unicodedata.normalize('NFKD', output["values"]["throughput"]).encode('ascii', 'ignore').strip().strip(),
                unicodedata.normalize('NFKD', output["values"]["status"]).encode('ascii', 'ignore').strip(),
                unicodedata.normalize('NFKD', output["values"]["log"]).encode('ascii', 'ignore').strip().strip())

    def kill_ftp(self):
        """
        Kill a FTP service
        """
        self._logger.info("Kill FTP transfer...")
        method = "KillFTP"
        # Get the method and class name of the UEcommand on the embedded side
        module_name, class_name = self._get_module_and_class_names()
        output = self._internal_uecmd_exec(module_name, class_name, method)

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

        # Windows FTP client check file size on server and DUT before computing throughput
        # So no need to check it here, returns always true

        return True
