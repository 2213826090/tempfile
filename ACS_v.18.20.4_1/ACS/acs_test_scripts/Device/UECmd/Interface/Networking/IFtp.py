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
:summary: This file implements INetworking UECmds
:since: 14/11/2014
:author: mbrisbax
"""
from ErrorHandling.DeviceException import DeviceException


class IFtp():
    """
    Abstract class that defines the interface to be implemented
    by Ftp handling sub classes.

    All method that shall be redefined in sub-classes raise a
    I{DeviceException} error.
    """
    # pylint: disable=W0613

    def __init__(self, device):
        """
        Initializes this instance.

        Nothing to be done in abstract class.
        """
        pass

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
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

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
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def kill_ftp(self, ftp_id):
        """
        Kill a FTP transfer

        :return: ftp transfer result
                - transfer overall throughput
                - transfer status
                - transfer log
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_ftp_status(self, thread_id):
        """
        get the ftp status.

        :type thread_id: long
        :param thread_id: ftp thread id

        :rtype: str
        :return: ftp status
                - "transfer successful"
                - "transferring"
                - "notrunning"
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

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
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)
