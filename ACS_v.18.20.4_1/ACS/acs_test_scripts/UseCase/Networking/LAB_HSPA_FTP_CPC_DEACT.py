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
:summary: This file implements HSPA FTP with CPC deactivation during data transfer UC
using three different ways : CPC deactivation, send HS-SCCH ORDER, PDP deactivation
:author: lvacheyx
:since:05/02/2013
"""

import time
import os
from LAB_HSPA_BASE import LabHspaBase
from ErrorHandling.TestEquipmentException import TestEquipmentException
from ErrorHandling.DeviceException import DeviceException
from UtilitiesFWK.Utilities import Global
import acs_test_scripts.Device.UECmd.UECmdTypes as UECmdTypes


class LabHspaFtpCpcDeact(LabHspaBase):

    """
    Lab HSPA ftp with CPC deactivation during FTP transfer test.
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
        self._cpc_deact_type = self._tc_parameters.get_param_value("CPC_DEACT_TYPE")

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

        if self._direction == "DL":
            # Read the DL_FILE value from UseCase xml Parameter
            self._ftp_filename = os.path.join(
                self._ftp_path,
                self._tc_parameters.get_param_value("DL_FILENAME"))
            self._direction = UECmdTypes.XFER_DIRECTIONS.DL
        elif self._direction == "UL":
            # Read the UL_FILE value from UseCase xml Parameter
            self._ftp_filename = os.path.join(
                self._ftp_path,
                self._tc_parameters.get_param_value("UL_FILENAME"))
            self._direction = UECmdTypes.XFER_DIRECTIONS.UL
        else:
            self._error.Msg = "%s is not a known xfer direction" % \
                self._direction
            raise TestEquipmentException(
                TestEquipmentException.INVALID_PARAMETER,
                self._error.Msg)

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

        # Check CPC activated state is Off for UL DTX and DL DRX
        cpc_state = self._ns_data_3g.get_cpc_state()

        if cpc_state == "OFF":
            msg = self._logger.error("CPC has not been successfully enable on UL DTX and on DL DRX")
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, msg)

        else:
            self._logger.info("CPC has been successfully enable on UL DTX and on DL DRX")

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

        # wait 20 seconds to ensure that transfer is established
        self._logger.info(
            "Wait 30 seconds to ensure that transfer is established")
        time.sleep(30)

        # Check data state "TRANSFERRING" before 10 seconds
        self._ns_data_3g.check_data_connection_transferring(self._check_data_transfer_state_timeout,
                                                            True,
                                                            blocking=False)

        if self._cpc_deact_type == "CPC_OFF":
            # Deactivate CPC feature
            self._logger.info(
                "CPC deactivation on the equipment by setting CPC to OFF")
            self._ns_data_3g.set_cpc_state("OFF")
            # wait 3 seconds for ensure that CPC deactivation has been made
            time.sleep(self._timeout_between_commands)

            # Check CPC activated state is OFF
            cpc_state = self._ns_data_3g.get_cpc_state()
            self._logger.info("CPC reported state is : %s", cpc_state)
            if cpc_state == "OFF":
                self._wait_end_of_ftp()
            # CPC deactivation failed
            else:
                self._error.Msg = "CPC deactivation failed. CPC state is still active"
                raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED,
                                             self._error.Msg)

        elif self._cpc_deact_type == "HS_SCCH_ORDER":
            self._logger.info(
                "CPC deactivation on the equipment by sending a HS-SCCH ORDER")

            # Send the HS-SCCH ORDER
            self._ns_data_3g.send_hsscch_order()
            time.sleep(self._timeout_between_commands)

            # Check CPC activated state is ON
            cpc_state = self._ns_data_3g.get_cpc_state()
            self._logger.info("CPC reported state is : %s", cpc_state)
            if cpc_state == "ON":
                self._wait_end_of_ftp()
            # CPC deactivation failed
            else:
                self._error.Msg = "CPC deactivation failed. CPC is inactive"
                raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED,
                                             self._error.Msg)

        elif self._cpc_deact_type == "PDP_DEACT":
            # deactivate PDP context
            self._logger.info(
                "CPC deactivation on the equipment by deactivating PDP context")
            self._networking_api.deactivate_pdp_context(self._ssid)

            time.sleep(self._timeout_between_commands)

            self._logger.info("Data transfer has stopped !")

            # Check CPC activated state is Off for UL DTX and DL DRX
            current_drx_activated_state = self._ns_data_3g.get_current_drx_activated_state()
            current_dtx_activated_state = self._ns_data_3g.get_current_dtx_activated_state()

            if current_drx_activated_state == "OFF" and current_dtx_activated_state == "OFF":

                self._logger.info("CPC has been successfully disable on UL DTX and on DL DRX")
                # Check Data Connection State => ATTACHED before TimeOut
                self._ns_data_3g.check_data_connection_state("ATTACHED",
                                                             self._registration_timeout,
                                                             blocking=False)
            else:
                self._error.Msg = "CPC deactivation on the equipment by deactivating PDP context has failed !"
                raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED,
                                             self._error.Msg)

        else:
            self._error.Msg = "%s is not a known CPC deactivation type" % \
                self._cpc_deact_type
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                         self._error.Msg)

        # Activate CPC feature
        self._logger.info("For BTB testing : CPC activation on the equipment by setting CPC to On")
        self._ns_data_3g.set_cpc_state("ON")

        # deactivate PDP context
        self._logger.info(
            "For Back to Back testing : Reactivating PDP context")
        self._networking_api.activate_pdp_context(self._ssid)

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

    def _wait_end_of_ftp(self):
        # Check CPC activated state is Off for UL DTX and DL DRX
        current_drx_activated_state = self._ns_data_3g.get_current_drx_activated_state()
        current_dtx_activated_state = self._ns_data_3g.get_current_dtx_activated_state()
        if current_drx_activated_state == "OFF" and current_dtx_activated_state == "OFF":
            self._logger.info("CPC has been successfully disable on UL DTX and on DL DRX")
            data_connection_state = self._ftp_api.get_ftp_status(self._ftp_task_id)
            self._logger.info("FTP transfer is still %s !" % data_connection_state)
            # No need to keep FTP transfer on going. Stop properly FTP client on DUT side
            try:
                self._ftp_api.stop_ftp(self._ftp_task_id)
            except:
                pass
            # if ftp transfer is finished

            if data_connection_state in ("transfer successful", "transferring"):
                self._logger.info("FTP %s of file %s finish success !" % (self._direction, self._ftp_filename))
            else:
                self._error.Msg = "FTP transfer failed! FTP status: %s" % (data_connection_state)
                raise DeviceException(DeviceException.CONNECTION_LOST, self._error.Msg)
        else:
            self._error.Msg = "CPC deactivation on the equipment by sending a HS-SCCH ORDER has failed !"
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, self._error.Msg)
