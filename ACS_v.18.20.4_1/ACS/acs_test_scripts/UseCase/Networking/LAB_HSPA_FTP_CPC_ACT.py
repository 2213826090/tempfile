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
:summary: This file implements HSPA FTP with CPC activation during data transfer
:author: lvacheyx
:since:25/02/2013
"""

import time
import os
from LAB_HSPA_BASE import LabHspaBase
from ErrorHandling.TestEquipmentException import TestEquipmentException
from ErrorHandling.DeviceException import DeviceException
from UtilitiesFWK.Utilities import Global
import acs_test_scripts.Device.UECmd.UECmdTypes as UECmdTypes


class LabHspaFtpCpcAct(LabHspaBase):

    """
    Lab HSPA ftp with CPC activation during FTP transfer test.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call LAB_HSPA_BASE Init function
        LabHspaBase.__init__(self, tc_name, global_config)

        self._check_data_transfer_state_timeout = 10
        self._timeout_between_commands = 2

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
        self._ftp_api = self._device.get_uecmd("Ftp")
        self._ftp_task_id = 0
        self._cpc_state = None
        if self._direction == "DL":
            # Read the DL_FILE value from UseCase xml Parameter
            self._ftp_filename = os.path.join(
                self._ftp_path,
                self._tc_parameters.get_param_value("DL_FILENAME"))
            self._filename = str(self._dlfilename)
            self._direction = UECmdTypes.XFER_DIRECTIONS.DL
        elif self._direction == "UL":
            # Read the UL_FILE value from UseCase xml Parameter
            self._ftp_filename = os.path.join(
                self._ftp_path,
                self._tc_parameters.get_param_value("UL_FILENAME"))
            self._filename = str(self._ulfilename)
            self._direction = UECmdTypes.XFER_DIRECTIONS.UL
        else:
            self._error.Msg = "%s is not a known xfer direction" % \
                self._direction
            raise DeviceException(DeviceException.INVALID_PARAMETER,
                                  self._error.Msg)

# ------------------------------------------------------------------------------
    def set_up(self):
        """
        Setup the test
        """
        # Call LAB_HSPA_BASE set_up function
        LabHspaBase.set_up(self)

        # PDP deactivation and CPC deactivation to be sure that PDP will be activated
        # without CPC feature enabled

        # Deactivate PDP context
        self._logger.info("Deactive PDP Context for...")
        self._networking_api.deactivate_pdp_context(self._ssid)

        # Deactivate CPC feature
        self._logger.info("CPC deactivation on the equipment by setting CPC to OFF")
        self._ns_data_3g.set_cpc_state("OFF")
        time.sleep(2)

        # Check CPC activated state is Off
        self._cpc_state = self._ns_data_3g.get_cpc_state()

        # PDP activation without CPC feature enabled

        # Activate PDP context
        time.sleep(self._wait_btwn_cmd)
        self._logger.info("Active PDP Context...")
        self._networking_api.activate_pdp_context(self._ssid)

        # Check Data Connection State => PDP Active before timeout
        self._ns_data_3g.check_data_connection_state("PDP_ACTIVE",
                                                     self._registration_timeout,
                                                     blocking=False)

        return Global.SUCCESS, "No errors"

# ------------------------------------------------------------------------------
    def run_test(self):
        """
        Execute the test
        """
        # Call LAB_HSPA_BASE run_test function
        LabHspaBase.run_test(self)

        # Run FTP transfer using FTP parameters :
        # - LAB_SERVER parameters (ip, username, password)
        # - DIRECTION
        # - DL_FILE or UL_FILE
        # - XFER_TIMEOUT
        # Start an ftp tranfer
        transfer_established_timeout = 20

        if self._cpc_state == "OFF":

            # Start FTP transfer if CPC is deactivated
            # Use private method __start_ftp_xfer
            self.__start_ftp_xfer()

        # CPC deactivation failed
        else:
            self._error.Msg = "CPC deactivation failed. CPC state is still active"
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED,
                                         self._error.Msg)

        # wait 20 seconds for ensure that transfer is established
        self._logger.info("Wait %i seconds for ensure that transfer is established",
                          transfer_established_timeout)
        time.sleep(transfer_established_timeout)

        # Check that data transfer in on-going on DUT before CPC activation
        data_connection_state = self._ftp_api.get_ftp_status(self._ftp_task_id)
        self._logger.info("FTP transfer is %s !" % data_connection_state)

        if data_connection_state != "transferring":

            msg = self._logger.error("FTP connection lost on DUT !")
            raise DeviceException(DeviceException.CONNECTION_LOST, msg)

        # Activate CPC feature
        self._logger.info("CPC activation on the equipment by setting CPC to On")
        self._ns_data_3g.set_cpc_state("ON")
        time.sleep(self._timeout_between_commands)

        # Check CPC activated state is On
        cpc_state = self._ns_data_3g.get_cpc_state()

        if cpc_state == "ON":
            # Check that CPC is active and wait for FTP transfer to finish
            # Use private method __activate_cpc
            self.__activate_cpc()

        # CPC activation failed
        else:
            self._error.Msg = "CPC activation failed. CPC state is still inactive"
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED,
                                         self._error.Msg)

        # Deactivate CPC feature
        self._logger.info("For BTB testing : CPC deactivation on the equipment by setting CPC to Off")
        self._ns_data_3g.set_cpc_state("OFF")

        return Global.SUCCESS, "No errors"

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

        # Deactivate CPC feature
        self._logger.info("CPC deactivation on the equipment by setting CPC to Off")
        self._ns_data_3g.set_cpc_state("OFF")

        # Call LAB_HSPA_BASE tear_down function
        LabHspaBase.tear_down(self)

        return Global.SUCCESS, "No errors"

# ------------------------------------------------------------------------------
    def __start_ftp_xfer(self):
        """
        Start FTP transfer if CPC is deactivated
        """

        # Check CPC activated state is Off for UL DTX and DL DRX
        current_drx_activated_state = self._ns_data_3g.get_current_drx_activated_state()
        current_dtx_activated_state = self._ns_data_3g.get_current_dtx_activated_state()

        # FTP transfer will start only if DL DRX and UL DTX are OFF
        if current_drx_activated_state == "OFF" and current_dtx_activated_state == "OFF":

            time.sleep(self._wait_btwn_cmd)

            self._logger.info("FTP transfer " + str(self._direction) +
                              " for " + str(self._ftp_filename) + "...")
            # Start FTP transfer
            self._ftp_task_id = self._ftp_api.start_ftp(self._direction,
                                                        self._server_ip_address,
                                                        self._username,
                                                        self._password,
                                                        self._ftp_filename,
                                                        self._device.multimedia_path,
                                                        self._ns_DUT_IP_Address)

        # CPC deactivation failed
        else:
            self._error.Msg = "CPC deactivation failed. CPC state is still active"
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED,
                                         self._error.Msg)

# ------------------------------------------------------------------------------
    def __activate_cpc(self):
        """
        Activate CPC and wait for FTP transfer to finish
        """

        # Check CPC activated state is On for UL DTX and DL DRX
        current_drx_activated_state = self._ns_data_3g.get_current_drx_activated_state()
        current_dtx_activated_state = self._ns_data_3g.get_current_dtx_activated_state()

        if current_drx_activated_state == "ON" and current_dtx_activated_state == "ON":

            # Check data state is still "transferring" before 10 seconds
            self._ns_data_3g.check_data_connection_transferring(
                self._check_data_transfer_state_timeout,
                True,
                blocking=False)

            data_connection_state = self._ftp_api.get_ftp_status(self._ftp_task_id)
            self._logger.info("FTP transfer is still %s !" % data_connection_state)
            # No need to keep FTP transfer on going. Stop properly FTP client on DUT side
            try:
                self._ftp_api.stop_ftp(self._ftp_task_id)
                self._ftp_api.kill_ftp()
            except:
                pass

            # Check if transfer was successful or still on going
            if data_connection_state in ("transfer successful", "transferring"):
                self._logger.info("FTP %s of file %s finish success !" % (self._direction, self._filename))
            else:
                self._error.Msg = "FTP transfer failed or have been too long! FTP status: %s" % data_connection_state
                raise DeviceException(DeviceException.PROHIBITIVE_BEHAVIOR,
                                      self._error.Msg)

        # CPC activation failed
        else:
            self._error.Msg = "CPC activation failed. CPC state is still inactive"
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED,
                                         self._error.Msg)
