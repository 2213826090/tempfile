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
:summary: This file implements the LIVE WIFI FTP UC
:since: 30/03/2010
:author: cbresoli
"""
import time
import os
from acs_test_scripts.UseCase.Networking.LIVE_WIFI_BASE import LiveWifiBase
from ErrorHandling.AcsConfigException import AcsConfigException


class LiveWifiFtp(LiveWifiBase):

    """
    Lab Wifi ftp test.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        LiveWifiBase.__init__(self, tc_name, global_config)

        # Get TC Parameters
        self._direction = self._tc_parameters.get_param_value("DIRECTION")
        # Read the DL_FILE value from UseCase xml Parameter
        self._dlfilename = os.path.join(
            self._ftp_path,
            self._tc_parameters.get_param_value("DL_FILE", ""))
        # Read the UL_FILE value from UseCase xml Parameter
        self._ulfilename = os.path.join(
            self._ftp_path,
            self._tc_parameters.get_param_value("UL_FILE", ""))

        self._xfer_timeout = \
            int(self._tc_parameters.get_param_value("XFER_TIMEOUT"))

        # Intended to test connection failure with wrong passphrase.
        self._wrong_passphrase = None

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        LiveWifiBase.run_test(self)

        time.sleep(self._wait_btwn_cmd)
        if self._direction == "DL":

            self._logger.info("FTP transfer " + str(self._direction) +
                              " for " + str(self._dlfilename) + "...")
            result = self._networking_api.\
                ftp_xfer(self._uecmd_types.XFER_DIRECTIONS.DL,  # pylint: disable=E1101
                         self._server_ip_address,
                         self._username,
                         self._password,
                         self._dlfilename,
                         self._xfer_timeout,
                         self._device.get_ftpdir_path())

        elif self._direction == "UL":
            self._logger.info("FTP transfer " + str(self._direction) +
                              " for " + str(self._ulfilename) + "...")
            result = self._networking_api.\
                ftp_xfer(self._uecmd_types.XFER_DIRECTIONS.UL,  # pylint: disable=E1101
                         self._server_ip_address,
                         self._username,
                         self._password,
                         self._ulfilename,
                         self._xfer_timeout,
                         self._device.get_ftpdir_path())
        else:
            self._error.Msg = "%s is not a known xfer direction" % self._direction
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, self._error.Msg)

        return result
