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
:summary: This file implements TDSCDMA FTP UC
:author: gcharlem
:since:17/11/2014
"""

import time
import os
from LAB_TDSCDMA_BASE import LabTdscdmaBase
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.Utilities.CommunicationUtilities import throughput_targets_string
from acs_test_scripts.Utilities.FtpUtilities import perform_ftp_transfer


class LabTdscdmaFtp(LabTdscdmaBase):

    """
    Lab TDSCDMA ftp test.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call LAB_TDSCDMA_BASE Init function
        LabTdscdmaBase.__init__(self, tc_name, global_config)

        # Get FTP server parameters
        self._server = \
            global_config.benchConfig.get_parameters("LAB_SERVER")
        self._server_ip_address = self._server.get_param_value("IP")
        self._username = self._server.get_param_value("username")
        self._password = self._server.get_param_value("password")
        if self._server.has_parameter("ftp_path"):
            self._ftp_path = self._server.get_param_value("ftp_path")
        else:
            self._ftp_path = ""

        # Read the DL_FILE value from UseCase xml Parameter
        self._dlfilename = os.path.join(
            self._ftp_path,
            self._tc_parameters.get_param_value("DL_FILENAME", ""))

        # Read the UL_FILE value from UseCase xml Parameter
        self._filename = os.path.join(
            self._ftp_path,
            self._tc_parameters.get_param_value("UL_FILENAME", ""))

        # Read the XFER_TIMEOUT from UseCase xml Parameter
        self._xfer_timeout = \
            int(self._tc_parameters.get_param_value("XFER_TIMEOUT"))

        self._failure_targets = \
            str(self._tc_parameters.get_param_value("FAILURE_TARGETS", "FUTE"))

        # Update the failure targets
        self._throughput_targets.set_failure_throughput_from_config(self._dut_config,
                                                                    self._failure_targets)

        # Log Throughput targets for TDSCDMA
        self._logger.info(throughput_targets_string(self._throughput_targets))

        # FTP api
        self._ftp_api = self._device.get_uecmd("Ftp")

# ------------------------------------------------------------------------------
    def set_up(self):
        """
        Initialize the test
        """
        # Call LAB_TDSCDMA_BASE set_up function
        LabTdscdmaBase.set_up(self)

        # Set up DUT to registered
        self._set_up_registration()

        # Set up DUT to pdp activated
        self._set_up_pdp_active()

        return Global.SUCCESS, "No errors"
# ------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        # Call LAB_TDSCDMA_BASE Run function
        LabTdscdmaBase.run_test(self)

        # Run FTP transfer using FTP parameters :
        # - LAB_SERVER parameters (ip, username, password)
        # - DIRECTION
        # - DL_FILE or UL_FILE
        # - XFER_TIMEOUT
        time.sleep(self._wait_btwn_cmd)
        return perform_ftp_transfer(self._direction,
                                    self._server_ip_address,
                                    self._username,
                                    self._password,
                                    self._filename,
                                    self._xfer_timeout,
                                    self._device.multimedia_path,
                                    self._ns_DUT_IP_Address,
                                    self._ftp_api,
                                    self._throughput_targets.ul_failure.value,
                                    self._logger,
                                    self._dlfilename,
                                    self._throughput_targets.dl_failure.value,
                                    self._device.binaries_path)
