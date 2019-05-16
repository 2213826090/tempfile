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
:summary: This file is the Use Case for Hspa hard handovers
during ftp transfer
:since: 03/09/2013
:author: sjamaoui
"""
import time
import os
from UtilitiesFWK.Utilities import Global
from LAB_HSPA_BASE import LabHspaBase
from ErrorHandling.AcsConfigException import AcsConfigException
import acs_test_scripts.Device.UECmd.UECmdTypes as UECmdTypes


class LabHspaHhoFtp(LabHspaBase):

    """
    Usecase for mobility Hspa hard handover during an ftp transfer
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call LAB_HSPA_BASE Init function
        LabHspaBase.__init__(self, tc_name, global_config)

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

        # Initializing the variable which will contain the IP address to use.
        self._check_data_transfer_state_timeout = 20
        self._ip_address_list = []
        self._ftp_task_id = 0
        self._timeout = None
        self._ftp_api = self._device.get_uecmd("Ftp")

        # IF DIRECTION is DL prepare for download FTP
        if self._direction == "DL":
            self._ftp_filename = self._dlfilename
            self._direction = UECmdTypes.XFER_DIRECTIONS.DL
        # IF DIRECTION is UL prepare for upload FTP
        elif self._direction == "UL":
            self._ftp_filename = self._ulfilename
            self._direction = UECmdTypes.XFER_DIRECTIONS.UL
        # ELSE raise an error
        else:
            # Raise an error in case the direction is not known
            msg = "Unknown ftp direction (%s)" % self._ftp_direction
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

# ------------------------------------------------------------------------------
    def run_test(self):
        """
        Execute the test
        """
        # Call LAB_MOBILITY_3G_HHO_BASE run_test function
        LabHspaBase.run_test(self)

        time.sleep(self._wait_btwn_cmd)

        self._logger.info("FTP transfer " + str(self._direction) +
                          " for " + str(self._ftp_filename) + "...")

        self._ftp_task_id = self._ftp_api.start_ftp(self._direction,
                                                    self._server_ip_address,
                                                    self._username,
                                                    self._password,
                                                    self._ftp_filename,
                                                    self._device.multimedia_path,
                                                    self._ns_DUT_IP_Address)

        timeout = time.time() + self._xfer_timeout
        # Check data state "TRANSFERRING" before timeout
        self._ns_data_3g.check_data_connection_transferring(self._check_data_transfer_state_timeout)

        # perfom a hard handover from cellA to cellB
        self.perform_hard_handover(self._ns_cell_3g)

        # Check data state "TRANSFERRING" before timeout
        self._ns_data_3g.check_data_connection_transferring(self._check_data_transfer_state_timeout)
        # Get RAT from Equipment
        network_type = self._ns_data_3g.get_network_type()
        # Check that DUT is registered on the good RAT
        self._modem_api.check_network_type_before_timeout(network_type,
                                                          self._registration_timeout)

        # perfom a hard handover from cellB to cellA
        self.perform_hard_handover(self._ns_cell_3g)

        # Check data state "TRANSFERRING" before timeout
        self._ns_data_3g.check_data_connection_transferring(self._check_data_transfer_state_timeout, blocking=False)
        # Get RAT from Equipment
        network_type = self._ns_data_3g.get_network_type()
        # Check that DUT is registered on the good RAT
        self._modem_api.check_network_type_before_timeout(network_type,
                                                          self._registration_timeout)
        # number of Hard Handover
        hh_number = 2 * self.get_b2b_iteration()

        # Create success_msg
        self._logger.info("Ftp success and %i Hard Handover(s) done" % hh_number)

        data_connection_state = self._ftp_api.get_ftp_status(self._ftp_task_id)
        self._logger.info("FTP transfer is %s !" % data_connection_state)

        # No need to keep FTP transfer on going. Stop properly FTP client on DUT side
        try:
            self._ftp_api.stop_ftp(self._ftp_task_id)
            self._ftp_api.kill_ftp()
        except:
            pass
        # if ftp transfer is finished
        if data_connection_state in ("transfer successful", "transferring"):
            self._logger.info("FTP %s of file %s finish success !" % (
                self._direction,
                self._ftp_filename))
            return Global.SUCCESS, "No Errors"
        else:
            self._error.Msg = "FTP transfer failed or have been too long! FTP status: %s" % (data_connection_state)
            return Global.FAILURE, self._error.Msg

# ------------------------------------------------------------------------------
    def tear_down(self):
        """
        End and dispose the test
        """
        # Stop properly FTP client on DUT side
        try:
            self._ftp_api.stop_ftp(self._ftp_task_id)
        except:
            pass

        # Call LAB_HSPA_BASE tear_down function
        LabHspaBase.tear_down(self)

        return Global.SUCCESS, "No errors"
