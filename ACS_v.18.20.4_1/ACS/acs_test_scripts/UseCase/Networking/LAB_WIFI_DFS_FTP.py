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
:summary: This file implements the LAB WIFI DFS FTP UC
:since: 04/09/2013
:author: aberthex
"""
import time
import os
from acs_test_scripts.UseCase.Networking.LAB_WIFI_DFS import LabWifiDfs
from acs_test_scripts.UseCase.Networking.LAB_WIFI_BASE import LabWifiBase
from UtilitiesFWK.Utilities import Global
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException


class LabWifiDfsFtp(LabWifiDfs):

    """
    Lab Wifi ftp dfs test.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        LabWifiDfs.__init__(self, tc_name, global_config)

        # Get TC Parameters
        self._direction = self._tc_parameters.get_param_value("DIRECTION")
        # Read the DL_FILE value from UseCase xml Parameter
        self._dlfilename = os.path.join(self._ftp_path, self._tc_parameters.get_param_value("DL_FILE", ""))
        # Read the UL_FILE value from UseCase xml Parameter
        self._ulfilename = os.path.join(self._ftp_path, self._tc_parameters.get_param_value("UL_FILE", ""))
        self._xfer_timeout = int(self._tc_parameters.get_param_value("XFER_TIMEOUT"))

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        LabWifiBase.run_test(self)

        self._networking_api.check_connection_state(self._ssid)

        if self._direction == "DL":
            msg = "FTP transfer " + str(self._direction) + " for " + str(self._dlfilename) + "..."
            filename = self._dlfilename

        elif self._direction == "UL":
            msg = "FTP transfer " + str(self._direction) + " for " + str(self._ulfilename) + "..."
            filename = self._ulfilename
        else:
            msg = "%s is not a known xfer direction" % self._direction
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        self._logger.info(msg)

        task_id = self._networking_api.start_ftp_xfer(self._direction, self._ftp_ip_address, self._ftp_username,
                                                      self._ftp_password, filename, self._device.get_ftpdir_path())

        start = time.time()

        # trigger DFS on AP while FTP Transfer is being done.
        self._trigger_dfs()
        # check that DUT was able to connect after DFS trigger
        self._networking_api.check_connection_state(self._ssid)
        # Check FTP transfer is still ongoing
        ftp_status = self._networking_api.get_ftp_xfer_status()

        if ftp_status != self._networking_api.FTP_TRANSFERRING:
            if self._networking_api.is_ftp_xfer_success(filename, self._direction, task_id):
                msg = "File has already been transferred. Please use a bigger file for this test!"
                # End the transfer
                self._networking_api.stop_ftp_xfer(task_id)
                self._logger.error(msg)
                raise AcsConfigException(AcsConfigException.OPERATION_FAILED, msg)
            else:
                msg = "FTP transfer STOPS while switching channel, ftp status=%s. " % str(ftp_status)
                # End the transfer
                self._networking_api.stop_ftp_xfer(task_id)
                self._logger.error(msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        # Wait for the FTP transfer to end.
        while start + self._xfer_timeout > time.time() and ftp_status == self._networking_api.FTP_TRANSFERRING:
            time.sleep(5)
            ftp_status = self._networking_api.get_ftp_xfer_status()

        # Does Timeout occur?
        if ftp_status == self._networking_api.FTP_TRANSFERRING:
            # End the transfer
            self._networking_api.stop_ftp_xfer(task_id)

            msg = "FTP Transfer timeout. Please use a smaller file or increase the timeout value"
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        # Control that the FTP transfer was success
        if not self._networking_api.is_ftp_xfer_success(self._dlfilename, self._direction, task_id):
            # End the transfer
            self._networking_api.stop_ftp_xfer(task_id)

            msg = "FTP transfer FAILS"
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        return Global.SUCCESS, "No errors"
