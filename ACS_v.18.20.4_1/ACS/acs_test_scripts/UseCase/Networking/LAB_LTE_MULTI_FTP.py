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
:summary: This file implements HSPA multi FTP transfer UC
:author: mbrisbax
:since:18/12/2014
"""
import time
import os
from LAB_LTE_FTP import LabLteFtp
from acs_test_scripts.Utilities.FtpUtilities import MultipleFtpTransfer
from UtilitiesFWK.Utilities import Global


class LabLteMultiFtp(LabLteFtp):

    """
    Lab HSPA ftp test.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call LAB_LTE_FTP Init function
        LabLteFtp.__init__(self, tc_name, global_config)

        # Read the DL_FILE value from UseCase xml Parameter
        self._nb_transfer = self._tc_parameters.get_param_value("NB_TRANSFER", 1, int)

        self._multi_ftp = MultipleFtpTransfer(self._nb_transfer,
                                              self._server_ip_address,
                                              self._direction,
                                              self._username,
                                              self._password,
                                              self._ns_dut_ip_Address,
                                              self._ftp_api,
                                              self._ftp_filename,
                                              self._dl_ftp_filename,
                                              self._logger,
                                              self._xfer_timeout,
                                              self._throughput_targets.ul_failure.value,
                                              self._throughput_targets.dl_failure.value,
                                              self._device.binaries_path,
                                              self._device.binaries_path)

# ------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """

        # Run FTP transfer using FTP parameters :
        # - LAB_SERVER parameters (ip, username, password)
        # - DIRECTION
        # - DL_FILE or UL_FILE
        # - XFER_TIMEOUT
        # Launch FTP transfers
        status, msg = self._multi_ftp.launch()
        if status == Global.FAILURE:
            self._logger.error(msg)
            self._multi_ftp.stop()
            return Global.FAILURE, msg

        # Wait end of data transfers and checks their throughput
        status, msg = self._multi_ftp.wait_end_of_transfers()
        if status == Global.FAILURE:
            self._logger.error(msg)

        return status, msg

# ------------------------------------------------------------------------------
    def tear_down(self):
        """
        End and dispose the test
        """
        # Call UseCaseBase Tear down
        LabLteFtp.tear_down(self)

        self._multi_ftp.stop()

        return Global.SUCCESS, "No errors"

