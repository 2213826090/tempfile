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
:summary:  This file implements usecase that do a mobility handover
during ftp transfer
:since: 17/08/2011
:author: ssavrimoutou
"""
import time
import os
import acs_test_scripts.Utilities.RegistrationUtilities as RegUtil

from UtilitiesFWK.Utilities import Global
from LAB_MOBILITY_3GSM_BASE import LabMobility3gsmBase
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException


class LabMobilityExtHoFtp(LabMobility3gsmBase):

    """
    Mobility handover during ftp transfer Usecase
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        # Call LabMobility3gsmBase Init function
        LabMobility3gsmBase.__init__(self, tc_name, global_config)

        self._check_data_transfer_state_timeout = 10

        # Read ftp DIRECTION from testcase xml parameters
        self._direction = self._tc_parameters.get_param_value("DIRECTION")

        # Get FTP server parameters
        self._server = global_config.benchConfig.get_parameters("LAB_SERVER")
        self._server_ip_address = self._server.get_param_value("IP")
        self._server_username = self._server.get_param_value("username")
        self._server_password = self._server.get_param_value("password")
        if self._server.has_parameter("ftp_path"):
            self._ftp_path = self._server.get_param_value("ftp_path")
        else:
            self._ftp_path = ""

        # Read the UL_FILE value from UseCase xml Parameter
        self._ulfilename = os.path.join(self._ftp_path, "put100M")
        # Read the DL_FILE value from UseCase xml Parameter
        self._dlfilename = os.path.join(self._ftp_path, "get500M")

#------------------------------------------------------------------------------
    def set_up(self):
        """
        Set up the test configuration
        """
        # Call LabMobility3gsmBase Set_up function
        LabMobility3gsmBase.set_up(self)

        # Set cell on
        self._ns1_cell.set_cell_on()

        # Disable flight mode
        self._networking_api.set_flight_mode("off")

        # Check registration state is connected using
        # registrationTimeout from Device_Catalog.xml
        self._modem_api.check_cdk_registration_bfor_timeout(self._registration_timeout)

        # Set the APN
        time.sleep(self._wait_btwn_cmd)
        self._logger.info("Setting APN " + str(self._apn) + "...")
        self._networking_api.set_apn(self._ssid, self._apn)

        # Activate PDP context
        time.sleep(self._wait_btwn_cmd)
        self._logger.info("Active PDP Context...")
        self._networking_api.activate_pdp_context(self._ssid, check=False)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------
    def run_test(self):
        """
        Execute the test
        """
        # Call LabMobility3gsmBase Run function
        LabMobility3gsmBase.run_test(self)

        ns1_cell = self._ns1_cell
        ns1_data = self._ns1_data
        ns2_cell = self._ns2_cell
        ns2_data = self._ns2_data

        # Check Data Connection State => PDP_ACTIVE before timeout
        RegUtil.check_dut_data_connection_state_before_timeout("PDP_ACTIVE",
                                                               ns1_cell,
                                                               self._networking_api,
                                                               self._logger,
                                                               self._registration_timeout,
                                                               flightmode_cycle=True,
                                                               blocking=False)

        # Check registration state is connected using
        # registrationTimeout from Device_Catalog.xml
        self._modem_api.check_cdk_registration_bfor_timeout(self._registration_timeout)

        # Check that DUT is registered on the good RAT
        self._modem_api.check_network_type_before_timeout(ns1_data.get_network_type(),
                                                          self._registration_timeout)

        # Set cell on
        ns2_cell.set_cell_on()

        # IF DIRECTION is UL starting upload FTP
        #    Start ftp upload of "put100M" file using  following parameters :
        #        DIRECTION, IP, username, password
        if self._direction == "UL":
            ftp_params = \
                [self._uecmd_types.XFER_DIRECTIONS.UL,  # pylint: disable=E1101
                 self._server_ip_address,
                 self._server_username,
                 self._server_password,
                 self._ulfilename,
                 self._device.get_ftpdir_path()]

        # ELIF DIRECTION is DL starting download FTP
        #    Start ftp download of "get500M" file using  following parameters :
        #        DIRECTION, IP, username, password
        elif self._direction == "DL":
            ftp_params = \
                [self._uecmd_types.XFER_DIRECTIONS.DL,  # pylint: disable=E1101
                 self._server_ip_address,
                 self._server_username,
                 self._server_password,
                 self._dlfilename,
                 self._device.get_ftpdir_path()]

        # ELSE raise an error
        # Raise an error in case the direction is not known
        else:
            self._logger.error("Unknown ftp direction (%s)"
                               % self._direction)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                   "Unknown ftp direction.")

        # Start an ftp tranfer
        ftp_task_id = self._networking_api.start_ftp_xfer(*ftp_params)  # pylint: disable=W0142

        # wait 20 seconds for ensure that transfer is established
        self._logger.info(
            "Wait 20 seconds for ensure that transfer is established")
        time.sleep(20)

        # Check data state "TRANSFERRING" before timeout
        ns1_data.check_data_connection_transferring(self._check_data_transfer_state_timeout)

        # Log the current Hard Handover iteration
        self._logger.info("Performing Hard Handover number 1 of 2")

        # Perform handover
        ns1_cell.execute_external_handover()

        self._logger.info("Wait 60 seconds for ensure that" +
                          " external handover is done")
        time.sleep(60)

        # Check data state "TRANSFERRING" on Neighbour NS before timeout
        ns2_data.check_data_connection_transferring(self._check_data_transfer_state_timeout,
                                                    False,
                                                    blocking=True)
        # Check data state "TRANSFERRING" on DUT
        data_connection_state = self._networking_api.get_ftp_xfer_status()
        self._logger.info("FTP transfer is %s !" % data_connection_state)

        # Raise an exception if data state is not "transferring" on the DUT
        if data_connection_state is not "transferring":
            msg = self._logger.error("FTP connection has been lost !")
            raise DeviceException(DeviceException.CONNECTION_LOST, msg)

        # Check that DUT is registered on the good RAT
        self._modem_api.check_network_type_before_timeout(ns2_data.get_network_type(),
                                                          self._registration_timeout)

        # Stop ftp transfer
        self._networking_api.stop_ftp_xfer(ftp_task_id)

        # Deactivate PDP context
        self._networking_api.deactivate_pdp_context(self._ssid)

        # Log the current Hard Handover iteration
        self._logger.info("Performing Hard Handover number 2 of 2")

        # Perform handover
        ns2_cell.execute_external_handover()

        self._logger.info("Wait 60 seconds for ensure that" +
                          " external handover is done")
        time.sleep(60)

        # Check data state "TRANSFERRING" on Neighbour NS before timeout
        ns1_data.check_data_connection_transferring(self._check_data_transfer_state_timeout,
                                                    False,
                                                    blocking=True)
        # Check data state "TRANSFERRING" on DUT
        data_connection_state = self._networking_api.get_ftp_xfer_status()
        self._logger.info("FTP transfer is %s !" % data_connection_state)

        # Raise an exception if data state is not "transferring" on the DUT
        if data_connection_state is not "transferring":
            msg = self._logger.error("FTP connection has been lost !")
            raise DeviceException(DeviceException.CONNECTION_LOST, msg)

        # Check that DUT is registered on the good RAT
        self._modem_api.check_network_type_before_timeout(ns1_data.get_network_type(),
                                                          self._registration_timeout)

        # Stop ftp transfer
        self._networking_api.stop_ftp_xfer(ftp_task_id)

        # Deactivate PDP context
        self._networking_api.deactivate_pdp_context(self._ssid)

        return Global.SUCCESS, "No errors"
