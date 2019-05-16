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
:summary:  This file implements usecase that do FTP over LTE network
:since: 17/04/2013
:author: hbianx
"""

import os
from LAB_LTE_BASE import LabLteBase
from UtilitiesFWK.Utilities import Global
from ErrorHandling.AcsConfigException import AcsConfigException
from acs_test_scripts.Utilities.FtpUtilities import perform_ftp_transfer
from acs_test_scripts.Utilities.CommunicationUtilities import throughput_targets_string


class LabLteFtp(LabLteBase):

    """
    Lab LTE FTP
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call LAB_LTE_BASE Init function
        LabLteBase.__init__(self, tc_name, global_config)
        # Read the the direction file name from UseCase xml Parameter
        self._direction = self._tc_parameters.get_param_value("DIRECTION")
        # Read the ftp file name from UseCase xml Parameter
        if self._direction == "DL":
            self._ftp_filename = os.path.join(self._ftp_path, self._tc_parameters.get_param_value("DL_FILENAME", ""))
            self._ftp_filename = self._ftp_filename.replace('\\', '/')
            self._dl_ftp_filename = None
        elif self._direction == "UL":
            # Read the UL_FILE value from UseCase xml Parameter
            self._ftp_filename = os.path.join(self._ftp_path, self._tc_parameters.get_param_value("UL_FILENAME", ""))
            self._ftp_filename = self._ftp_filename.replace('\\', '/')
            self._dl_ftp_filename = None
        elif self._direction == "BOTH":
            self._dl_ftp_filename = os.path.join(self._ftp_path, self._tc_parameters.get_param_value("DL_FILENAME", ""))
            self._dl_ftp_filename = self._dl_ftp_filename.replace('\\', '/')
            # Read the UL_FILE value from UseCase xml Parameter
            self._ftp_filename = os.path.join(self._ftp_path, self._tc_parameters.get_param_value("UL_FILENAME", ""))
            self._ftp_filename = self._ftp_filename.replace('\\', '/')
        # Read the XFER_TIMEOUT from UseCase xml Parameter
        self._xfer_timeout = self._tc_parameters.get_param_value("XFER_TIMEOUT")
        if self._xfer_timeout is not None and str(self._xfer_timeout).isdigit():
            self._xfer_timeout = int(self._xfer_timeout)
        else:
            self._xfer_timeout = None

        self._rrc_state = self._tc_parameters.get_param_value("RRC_STATE", "RRC_CONNECTED")

        # Initializing the variable which will contain the IP address to use.
        self._ip_address = None

        self._phone_system = self._device.get_uecmd("PhoneSystem")

        ######################################################
        #             LTE THROUGHPUT SETTINGS                #
        ######################################################
        self._set_lte_throughput_settings()

        # LTE FTP Test are FUTE by default
        if self._failure_targets == "":
            self._failure_targets = "FUTE"

        # Update the failure targets
        self._throughput_targets.set_failure_throughput_from_config(self._dut_config,
                                                                    self._failure_targets)

        # Log Throughput targets for LTE category
        self._logger.info(throughput_targets_string(self._throughput_targets))
        self._ftp_api = self._device.get_uecmd("Ftp")

# ------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        (code, msg) = self._setup_ftp_for_lte()

        if code == Global.SUCCESS:
            # Checking if the entered direction is correct.
            if self._direction not in ("UL", "DL", "BOTH"):
                self._error.Msg = "%s is not a known xfer direction" % \
                    self._direction
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, self._error.Msg)

            if self._xfer_timeout is None:
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                         "XFER_TIMEOUT should be int")

        return code, msg

# ------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        Test steps:
            Launch FTP.
            Wait for the transfer to finish.
            Measure FTP duration
            Get transferred file size on phone
            Compute throughput
            We are using here ftpput and ftpget methods because it is the only method
            which reach target throughput for LTE category 3&4
            For instance ftp_xfer is about 50% of LTE cat 3 max throughput
        """

        # Call LAB_LTE_BASE run_test function
        LabLteBase.run_test(self)
        # If transfer starts from IDLE, reactivate PDP context for windows platform
        if self._rrc_state == "RRC_IDLE":
            self._networking_api.reactivate_pdp_context(self._apn)

        return perform_ftp_transfer(self._direction,
                                    self._ip_address,
                                    self._username,
                                    self._password,
                                    self._ftp_filename,
                                    self._xfer_timeout,
                                    self._device.multimedia_path,
                                    self._ns_dut_ip_Address,
                                    self._ftp_api,
                                    self._throughput_targets.ul_failure.value,
                                    self._logger,
                                    self._dl_ftp_filename,
                                    self._throughput_targets.dl_failure.value,
                                    self._device.binaries_path)

# -----------------------------------------------------------------------------

    def tear_down(self):
        """
        Finishing the test.
        Stopping the FTP service and releasing the equipment.
        """
        # Stopping the FTP service before releasing the equipment.
        self._ns_data_4g.stop_ftp_service()
        LabLteBase.tear_down(self)
        return Global.SUCCESS, "No errors"
