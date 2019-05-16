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
:summary: Utilities classes and function for Data
:since: 30/08/2010
:author: cco
"""

import time
from ftplib import FTP, all_errors
import logging
import os
import re

from Core.Report.ACSLogging import ACS_LOGGER_NAME, TEST_SCRIPT_LOGGER_NAME
from ErrorHandling.TestEquipmentException import TestEquipmentException
from ErrorHandling.AcsBaseException import AcsBaseException


class FtpClient():
    """
    provides a ftp client on local computer
    with throughput measurement
    """
    def __init__(self):
        self._ftp = None
        # Set FTP socket size to 4MB
        self._blocksize = 4194304
        self._logger = logging.getLogger("%s.%s.FTP_CLIENT" % (ACS_LOGGER_NAME, TEST_SCRIPT_LOGGER_NAME,))
        self._throughput = 0.0

    def get_data_throughput(self):
        return self._throughput

    def ftp_connect(self, server, port=21, user="anonymous", password="", timeout=60):
        """
        Initialize connection to ftp server

        :type server: str
        :param server: server IP address

        :type port: int
        :param port: port for ftp connection. Defaults to 21

        :type user: str
        :param user: username for FTP credential. Defaults to anonymous

        :type password: str
        :param password: password for FTP credential. Defaults to empty str

        :type timeout: int
        :param timeout: timeout for establishing FTP data connection.
                        Defaults to 60 seconds
        """
        if self._ftp is not None:
            del self._ftp
        self._ftp = FTP()

        try:
            self._logger.info("Connecting to FTP server %s:%s" % (server, port))
            status = self._ftp.connect(server, port, timeout)
            if status.split()[0] == "220":
                self._logger.info("Connection to FTP server %s:%s successfull" % (server, port))
            status = self._ftp.login(user, password)
            if status.split()[0] == "230":
                self._logger.info("Login to FTP server %s:%s successfull"
                                  % (server, port))
        except all_errors as ex:
            exception_text = "FTP connection failed: %s " % ex
            self._logger.error(exception_text)
            raise TestEquipmentException(TestEquipmentException.CONNECTION_ERROR, exception_text)

    def ftp_disconnect(self):
        """
        Close the connection to the ftp server
        """
        if self._ftp is not None and self._ftp.sock is not None:
            try:
                self._ftp.abort()
                self._ftp.quit()
            except all_errors as ex:
                exception_text = "FTP disconnection failed: %s " % ex
                self._logger.error(exception_text)

    def ftp_get(self, ftp_file, ftp_path=None):
        """
        Execute a FTP get request on a dedicated FTP server

        :type ftp_file: str
        :param ftp_file: full path for the file to get from ftp.
        this means: <server address>/<path to file>/<full file name>
        """
        self._throughput = float(0)
        self._dl_size = 0

        if self._ftp is None:
            raise TestEquipmentException(TestEquipmentException.CONNECTION_ERROR,
                                         "Not connected to any FTP server!")
        # change remote folder
        self._ftp.cwd(ftp_path)
        # ensure that the expected file exists before downloading it
        if ftp_file not in self._ftp.nlst():
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                         "Expected file does not exist on remote FTP server")

        # Retrieve file size from FTP server
        ftp_file_size = float(self._ftp.size(ftp_file))

        if ftp_file_size is None:
            self._logger.error("File to download is Empty")
            return

        self._logger.info("Download %s" % ftp_file)
        # Download file from FTP server and compute transfer duration
        d1 = time.clock()
        try:
            self._ftp.retrbinary("RETR %s" % ftp_file, self._ftp_callback,
                                 blocksize=self._blocksize)
        except all_errors as ex:
            exception_text = "FTP GET failed: %s " % ex
            self._logger.error(exception_text)
            raise TestEquipmentException(TestEquipmentException.CONNECTION_LOST, exception_text)

        d2 = time.clock()
        self._logger.info("FTP transfer finished, compute throughput")

        # Compute throughput in B/s
        if self._dl_size == ftp_file_size:
            # Compute throughput in B/s
            self._throughput = ftp_file_size / (d2 - d1)
        else:
            self._logger.error("File size on computer %s is different from size on server %s" % (self._dl_size, ftp_file_size))

    def ftp_put(self, local_file, ftp_path=None):
        """
        Execute a FTP put request on a dedicated FTP server

        :type local_file: str
        :param local_file: full path for the file to upload to ftp.

        :type ftp_path: str
        :param ftp_path: the path to file on remote server

        """
        self._throughput = float(0)

        if self._ftp is None:
            raise TestEquipmentException(TestEquipmentException.CONNECTION_ERROR,
                                         "Not connected to any FTP server!")

        # Upload file to FTP server and compute transfer duration
        local_file_size = float(os.path.getsize(local_file))
        ftp_file = os.path.join(ftp_path, os.path.basename(local_file))
        ftp_file = ftp_file.replace('\\', '/')

        self._logger.info("upload %s to %s " % (local_file, ftp_file))
        d1 = time.clock()
        try:
            self._ftp.storbinary("STOR %s" % ftp_file, open(local_file, "rb"),
                                 blocksize=self._blocksize)
        except all_errors as ex:
            exception_text = "FTP PUT failed: %s " % ex
            self._logger.error(exception_text)
            raise TestEquipmentException(TestEquipmentException.CONNECTION_LOST, exception_text)

        d2 = time.clock()
        ftp_file_size = float(self._ftp.size(ftp_file))
        self._logger.info("FTP transfer finished, compute throughput")

        if local_file_size == ftp_file_size:
            # Compute throughput in B/s
            self._throughput = ftp_file_size / (d2 - d1)
        else:
            self._logger.error("File size on computer %s is different from size on server %s" % (local_file_size, ftp_file_size))

    def start_ftp_xfer(self, direction, server_ip, user, passwd, ftp_file, ftp_path=None):
        """
        interface for starting a transfer over FTP. Either a get or a put

        :type direction: str
        :param direction: UL or DL given the transfer is expected to be a
        put or a get request (respectively)

        :type server_ip: str
        :param server_ip: the FTP server address given in IPv4 or IPv6 format

        :type user: str
        :param user: the usename for FTP connection credential

        :type passwd: str
        :param passwd: the password for FTP connection credential

        :type ftp_file: str
        :param ftp_file: the full name of the ftp file

        :type ftp_path: str
        :param ftp_path: the path to file on remote server

        """

        srv = server_ip.split(":")
        if len(srv) == 2:
            port = srv[1]
            server_ip = srv[0]
        else:
            port = 21
        try:
            self.ftp_connect(server_ip, port, user, passwd)
            if direction == "DL":
                self.ftp_get(ftp_file, ftp_path)
            elif direction == "UL":
                self.ftp_put(ftp_file, ftp_path)
            else:
                raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                             "invalid direction value. Should be 'UL' or 'DL'")
        finally:
            self.ftp_disconnect()

    def retrieve_size_from_filename_and_create_it(self, filename):
        """
        Try to retrieve file size from the filename and create this file.
        Filename must follow a specific format:
        [name][size (integer)][unit (k, ko, kB, m, mo, mB, g, go, gB)][extension (optional)]
        example : put500MB.zip

        :type filename: str
        :param filename: the filename to create

        """
        known_units = ("ko", "kb", "mo", "mb", "go", "gb", "k", "m", "g")
        coef = {"ko": 1024.0, "k": 1024.0, "kb": 1024.0,
                "mo": (1024.0 * 1024.0), "m": (1024.0 * 1024.0), "mb": (1024.0 * 1024.0),
                "go": (1024.0 * 1024.0 * 1024.0), "g": (1024.0 * 1024.0 * 1024.0), "gb": (1024.0 * 1024.0 * 1024.0)}
        full_path = filename
        # retrieve the folder path
        folder_path = os.path.dirname(full_path)
        # retrieve only filename
        filename = os.path.basename(filename)
        # retrieve all integers and unit from the filename
        self._logger.debug("Filename is '%s'" % str(filename))

        regex_search = re.search("^\D*(\d*)(\w).*$", str(filename))

        if regex_search is not None:
            size = str(regex_search.group(1))
            unit = str(regex_search.group(2)).lower()
            self._logger.debug("File size will be '%s'" % str(size))
            self._logger.debug("File unit will be '%s'" % str(unit))

            if unit in known_units:
                if not os.path.isdir(folder_path):
                    os.mkdir(folder_path)
                # convert size in byte and create a file of this size
                with open(full_path, "wb") as out:
                    out.seek(int(int(size) * coef[unit]) - 1)
                    out.write('\0')
                return
            else:
                self._logger.warning("Unkown unit '%s'" % str(unit))

        raise AcsBaseException(AcsBaseException.INVALID_PARAMETER,
                               "Filename doesn't follow the specific format : "
                               "[name][size (integer)][unit %s]"
                               "[extension (optional)]" % str(known_units))

    def _ftp_callback(self, data):
        """
        Callback used for FTP file download (ftplib.ftp.retrbinary function need a callback parameter)

        :type data: str
        :param data: data downloaded by ftp

        """
        # Do nothing to optimize data transfer time
        # Test needs only to have file transferred by FTP with the maximum throughput
        # No need to store file on phone
        self._dl_size += len(data)
        return
