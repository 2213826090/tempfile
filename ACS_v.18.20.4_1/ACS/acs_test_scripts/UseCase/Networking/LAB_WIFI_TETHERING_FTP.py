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
:summary: This file implements WIFI TETHERING FTP UC
:author: szhen11
:since:27/09/2011
"""

import time
import os
from LAB_WIFI_TETHERING_BASE import LabWifiTetheringBase
from UtilitiesFWK.Utilities import Global


class LabWifiTetheringFtp(LabWifiTetheringBase):

    """
    Lab wifi tethering ftp test.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call LAB_WIFI_TETHERING_CONNECT Init function
        LabWifiTetheringBase.__init__(self, tc_name, global_config)

        # Read the DIRECTION value from UseCase xml Parameter
        self._direction = self._tc_parameters.get_param_value("DIRECTION")
        # Read the DL_FILE value from UseCase xml Parameter
        self._dlfilename = os.path.join(
            self._ftp_path,
            self._tc_parameters.get_param_value("DL_FILENAME", ""))
        # Read the UL_FILE value from UseCase xml Parameter
        self._ulfilename = os.path.join(
            self._ftp_path,
            self._tc_parameters.get_param_value("UL_FILENAME", ""))
        # Read the XFER_TIMEOUT from UseCase xml Parameter
        self._xfer_timeout = \
            int(self._tc_parameters.get_param_value("XFER_TIMEOUT"))

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        LabWifiTetheringBase.run_test(self)

        # Run FTP transfer using FTP parameters :
        # - LAB_SERVER parameters (ip, username, password)
        # - DIRECTION
        # - DL_FILE or UL_FILE
        # - XFER_TIMEOUT
        if self._direction == "DL":
            direction = self._uecmd_types.XFER_DIRECTIONS.DL  # pylint: disable=E1101
            filename = self._dlfilename
        elif self._direction == "UL":
            direction = self._uecmd_types.XFER_DIRECTIONS.UL  # pylint: disable=E1101
            filename = self._ulfilename
        else:
            return (Global.FAILURE, "%s is not a known xfer direction" %
                    self._direction)

        time.sleep(self._wait_btwn_cmd)
        result = self._networking_api2.ftp_xfer(
            direction,
            self._server_ip_address,
            self._username,
            self._password,
            filename,
            self._xfer_timeout,
            self._phone2.get_ftpdir_path())

        # Check that the result of ftp_xfer to compute verdict
        return result

#------------------------------------------------------------------------------
