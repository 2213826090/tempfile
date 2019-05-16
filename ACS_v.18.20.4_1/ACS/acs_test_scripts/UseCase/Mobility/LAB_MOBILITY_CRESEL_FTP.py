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
:summary: Several cell reselection 2G/3G during data transfer
:since: 16/09/2011
:author: ccontreras
"""

import os
import time

from LAB_MOBILITY_3GSM_BASE import LabMobility3gsmBase
from UtilitiesFWK.Utilities import Global
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.TestEquipmentException import TestEquipmentException
from UtilitiesFWK.Utilities import str_to_bool
import acs_test_scripts.Device.UECmd.UECmdTypes as UECmdTypes


class LabMobilityCreselFtp(LabMobility3gsmBase):

    """
    Several cell reselection 2G/3G during data transfer
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        # Call LabMobilityBase Init function
        LabMobility3gsmBase.__init__(self, tc_name, global_config)

        # Read DECREMENTATION_STEP_POWER from testcase xml parameters
        self._decrementation_step_power = \
            float(self._tc_parameters.get_param_value("DECREMENTATION_STEP_POWER"))

        # Set INCREMENTATION_STEP_POWER
        self._incrementation_step_power = \
            float(self._tc_parameters.get_param_value("INCREMENTATION_STEP_POWER"))

        # Read DECREMENTATION_STEP_TIMER from testcase xml parameters
        self._decrementation_step_timer = \
            float(self._tc_parameters.get_param_value("DECREMENTATION_STEP_TIMER"))

        # Sets INCREMENTATION_STEP_TIMER
        self._incrementation_step_timer = \
            float(self._tc_parameters.get_param_value("INCREMENTATION_STEP_TIMER"))

        # Read NS1 parameters from testcase xml parameters
        self._ns1_cell_power = \
            float(self._tc_parameters.get_param_value("NS1_CELL_POWER"))

        self._ns1_cell_service = \
            str(self._tc_parameters.get_param_value("NS1_CELL_SERVICE"))

        self._ns1_arfcn = \
            int(self._tc_parameters.get_param_value("NS1_ARFCN"))

        self._ns1_lac = \
            int(self._tc_parameters.get_param_value("NS1_LAC"))

        self._ns1_rac = \
            int(self._tc_parameters.get_param_value("NS1_RAC"))

        self._ns1_limit_power = \
            float(self._tc_parameters.get_param_value("NS1_LIMIT_POWER"))

        # Read NS2 parameters from testcase xml parameters
        self._ns2_cell_power = \
            float(self._tc_parameters.get_param_value("NS2_CELL_POWER"))

        self._ns2_cell_service = \
            str(self._tc_parameters.get_param_value("NS2_CELL_SERVICE"))

        self._ns2_arfcn = \
            int(self._tc_parameters.get_param_value("NS2_ARFCN"))

        self._ns2_lac = \
            int(self._tc_parameters.get_param_value("NS2_LAC"))

        self._ns2_rac = \
            int(self._tc_parameters.get_param_value("NS2_RAC"))
        # Get B2B_RESEL parameter from testcase xml parameters
        # If true it means that the TC will perform reselection from cell 1 to cell 2 and then to cell 1
        # If false it means that the TC will perform only reselection from cell 1 to cell 2
        self._b2b_resel = \
            str_to_bool(self._tc_parameters.get_param_value("B2B_RESEL", "True"))

        self._ns2_limit_power = \
            float(self._tc_parameters.get_param_value("NS2_LIMIT_POWER"))

        # Set camped cell power and neighbor cell power for
        # the first re-selection (ns1 to ns2)
        self._ns_camped_power = self._ns1_cell_power
        self._ns_neighbour_cell_power = self._ns2_cell_power

        # Read XFER_TIMEOUT from testcase xml parameters
        self._xfer_timeout = \
            float(self._tc_parameters.get_param_value("XFER_TIMEOUT"))

        # Read NS1_LIMIT_POWER_FIRST_RESEL from testcase xml parameters
        self._ns1_limit_power_1st_resel = \
            float(self._tc_parameters.get_param_value("NS1_LIMIT_POWER_FIRST_RESEL", "-110"))

        # Read COMPRESSED_MODE_ACTIVE from testcase xml parameters
        self._compress_mode_active = \
            str_to_bool(self._tc_parameters.get_param_value("COMPRESSED_MODE_ACTIVE", "False"))

        # Read GSM_CELL_RESELECTION_MIN_RXLEVEL from testcase xml parameters
        self._gsm_cell_reselection_min_rx_level = \
            int(self._tc_parameters.get_param_value("GSM_CELL_RESELECTION_MIN_RXLEVEL", "-80"))

        # Get FTP server parameters
        self._server = global_config.benchConfig.get_parameters("LAB_SERVER")
        self._server_ip_address = self._server.get_param_value("IP")
        self._server_username = self._server.get_param_value("username")
        self._server_password = self._server.get_param_value("password")
        if self._server.has_parameter("ftp_path"):
            self._ftp_path = self._server.get_param_value("ftp_path")
        else:
            self._ftp_path = ""

        # Read DIRECTION from testcase xml parameters
        self._direction = \
            str(self._tc_parameters.get_param_value("DIRECTION"))

        # IF DIRECTION is DL prepare for download FTP
        if self._direction == "DL":
            # Read the DL_FILE value from UseCase xml Parameter
            self._ftp_filename = os.path.join(
                self._ftp_path,
                self._tc_parameters.get_param_value("DL_FILENAME", ""))
            self._direction = UECmdTypes.XFER_DIRECTIONS.DL
        # ELIF DIRECTION is UL prepare for upload FTP
        elif self._direction == "UL":
            # Read the UL_FILE value from UseCase xml Parameter
            self._ftp_filename = os.path.join(
                self._ftp_path,
                self._tc_parameters.get_param_value("UL_FILENAME", ""))
            self._direction = UECmdTypes.XFER_DIRECTIONS.UL
        else:
            self._error.Msg = "%s is not a known xfer direction" % \
                self._direction
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     self._error.Msg)
        self._ftp_api = self._device.get_uecmd("Ftp")
        self._ftp_task_id = 0

# -----------------------------------------------------------------------------

    def set_up(self):
        """
        Set up the test configuration
        """
        # Call LabMobilityBase Setup function
        LabMobility3gsmBase.set_up(self)

        # Set camped cell power and neighbor cell power for the first re-selection
        # in case of B2B non-continuous test case.
        self._ns_camped_power = self._ns1_cell_power
        self._ns_neighbour_cell_power = self._ns2_cell_power

        # Set camped cell power and neighbor cell power for the first re-selection
        # in case of B2B non-continuous test case.
        self._ns_camped_power = self._ns1_cell_power
        self._ns_neighbour_cell_power = self._ns2_cell_power

        # Set Cell Band  and ARFCN using NS1CELL_BAND
        # and NS1ARFCN parameters
        # Set cell service using NS1_CELL_SERVICE parameter
        # Set Cell Power using ns1_cell_power parameter
        self._ns1_cell.configure_basic_cell_parameters(
            self._ns1_cell_service, self._ns1_cell_band,
            self._ns1_arfcn, self._ns_camped_power)

        # Set Cell Band  and ARFCN using NS2_CELL_BAND
        # and NS2_ARFCN parameters
        # Set cell service using NS2_CELL_SERVICE parameter
        # Set Cell Power using NS2_CELL_POWER parameter
        self._ns2_cell.configure_basic_cell_parameters(
            self._ns2_cell_service, self._ns2_cell_band,
            self._ns2_arfcn, self._ns_neighbour_cell_power)

        # Set NS1_LAC and NS1_RAC parameters
        self._ns1_cell.set_lac(self._ns1_lac)
        self._ns1_cell.set_rac(self._ns1_rac)

        # Set NS2_LAC and NS2_RAC parameters
        self._ns2_cell.set_lac(self._ns2_lac)
        self._ns2_cell.set_rac(self._ns2_rac)

        # Configure Compressed mode if needed
        if self._compress_mode_active:
            if not(self._ns1_cell_tech == "3G" and self._ns2_cell_tech == "2G"):
                msg = "It is an IRAT test with compressed mode test cell1 must be 3G and cell2 2G"
                self._logger.error(msg)
                return (Global.FAILURE,
                        msg)

            # Set compress mode measurement config for InteRAT measurement
            self._ns1_cell.set_compress_mode_measurement_config("ITRRat")
            # Define compress mode parameters with one gap measuring GSM RSSI
            # (equivalent to use CM gap GSM RSSI presets on 8960)
            # see http://wireless.agilent.com/rfcomms/refdocs/wcdma/wcdma_gen_bse_compressed_mode.html
            # table  Compressed Mode Preset Configurations collunm Inter-RAT GSM Meas
            self._ns1_cell.set_compress_mode_gap_config(1, "GSMR", 4, 12, 7, 0, 0, 0, 0, 0, 270)

            if "384" in self._ul_rab or "384" not in self._dl_rab:
                self._logger.warning("The RAB configuration is not correct, compressed mode may not work: \n" +
                                     "    it is better to use (ul 64k/dl 384k)")

            # Loads DL Channel code preset configuration for compressed mode when 384k DL DPCH is active (non HSDPA).
            self._ns1_cell.set_compress_mode_channelization_codes_preset_for_384k_dl_dpch()
            # Set 2G cell PS handover state to ON
            self._ns2_cell.set_ps_handover_state(1)

        if(self._ns1_cell_tech == "3G"):
            self._set_gprs_bearer(self._ns1_cell, self._ns1_data, 1)
            self._ns1_cell.set_gsm_cell_reselection_min_rx_level(self._gsm_cell_reselection_min_rx_level)
        if(self._ns2_cell_tech == "3G"):
            self._set_gprs_bearer(self._ns2_cell, self._ns2_data, 2)
            self._ns2_cell.set_gsm_cell_reselection_min_rx_level(self._gsm_cell_reselection_min_rx_level)

        # Due to HSPA cell power overwritting
        self._reset_cells_power()

        # Set NS1 cell on
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

        return (Global.SUCCESS, "No errors")
# -----------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """

        # Call LabMobilityBase Run function
        LabMobility3gsmBase.run_test(self)

        check_data_is_transferring_timeout = 10
        ftp_transferring_timeout = 60
        compress_mode_enabling_time = 10
        camp_cco_timeout = 20
        sys_info_check_time = 80

        # Check Data Connection State => PDP_ACTIVE before timeout
        self._ns1_data.check_data_connection_state("PDP_ACTIVE",
                                                   self._registration_timeout)

        # Get RAT from Equipment
        network_type = self._ns1_data.get_network_type()

        # Check that DUT is registered on the good RAT
        self._modem_api.check_network_type_before_timeout(network_type,
                                                          self._registration_timeout)
        # Set NS2 cell on
        self._ns2_cell.set_cell_on()

        self._logger.info("FTP transfer " + str(self._direction) +
                          " for " + str(self._ftp_filename) + "...")
        # Start FTP transfer
        self._ftp_task_id = self._ftp_api.start_ftp(self._direction,
                                                    self._server_ip_address,
                                                    self._server_username,
                                                    self._server_password,
                                                    self._ftp_filename,
                                                    self._device.multimedia_path,
                                                    self._ns1_ip_dut)

        # Check if continuous data transfer is etablished before timeout
        self._ns1_data.check_data_connection_transferring_time(ftp_transferring_timeout, check_data_is_transferring_timeout, True, True)
        # Log the current cell re-selection iteration
        self._logger.info("Performing cell reselection number 1 of 2")

        # Set camped cell power and neighbour cell power for the first reselection
        self._ns_camped_power = self._ns1_cell_power
        self._ns_neighbour_cell_power = self._ns2_cell_power

        # Configure Compressed mode if needed
        if not self._compress_mode_active:
            # First re-selection from ns1 (self._ns_camped_power) to ns2 (self._ns_neighbour_cell_power)
            try:
                self.decrease_and_increase_cell_power_while_ftp(self._ns1_cell,
                                                                self._ns_camped_power,
                                                                self._ns2_cell,
                                                                self._ns2_data,
                                                                self._ns_neighbour_cell_power,
                                                                self._decrementation_step_power,
                                                                self._decrementation_step_timer,
                                                                self._ns1_limit_power,
                                                                self._incrementation_step_power,
                                                                self._incrementation_step_timer,
                                                                self._ns2_limit_power,
                                                                self._ns1_cell_tech,
                                                                self._ns1_data)
                # As Re-selection has succeed update camped and neighbor cell power
                self._ns_camped_power = self._ns2_limit_power
                self._ns_neighbour_cell_power = self._ns1_limit_power
            except DeviceException as error:
                # Reset cell power for ns1 and ns2
                self._reset_cells_power()
                msg = str(error)
                self._logger.error(msg)
                return (Global.FAILURE, "Re-selection 1 of 2 failed")

        else:
            # Decrease 3G cell
            self.decrease_cell_power_while_ftp(self._ns1_cell,
                                               self._ns_camped_power,
                                               self._decrementation_step_power,
                                               self._decrementation_step_timer,
                                               self._ns1_limit_power)
            # Enable compress mode
            self._ns1_cell.set_compress_mode_state(1)
            # Check if 2G cell has been measured before timeout
            self.check_2G_measurement_while_compress_mode(compress_mode_enabling_time, self._ns1_cell, int(self._ns2_arfcn))
            self._logger.info("2G cell has been measured by 3G cell during compressed mode")
            self._ns1_cell.set_compress_mode_state(0)
            time.sleep(2)

            # Trigger CCO 3G to 2G
            self._ns1_cell.execute_external_handover()
            # Check Data Connection State => PDP_ACTIVE before timeout
            try:
                self._ns2_data.check_data_connection_state("PDP_ACTIVE",
                                                           camp_cco_timeout)
            except TestEquipmentException as error:
                # Reset cell power for ns1 and ns2
                self._reset_cells_power()
                return (Global.FAILURE, "CCO has failed")

            # Reduce 3G cell power
            self._ns1_cell.set_cell_power(self._ns1_limit_power_1st_resel)
            self._ns_camped_power = self._ns_neighbour_cell_power
            self._ns_neighbour_cell_power = self._ns1_limit_power_1st_resel

        data_connection_state = self._ftp_api.get_ftp_status(self._ftp_task_id)
        self._logger.info("FTP transfer is %s !" % data_connection_state)

        if data_connection_state != "transferring":
            msg = "FTP transfer is not on-going: %s" % data_connection_state
            self._logger.error(msg)
            self._reset_cells_power()
            return (Global.FAILURE, msg)

        # Check if continuous data transfer is etablished before timeout
        try:
            self._ns2_data.check_data_connection_transferring_time(ftp_transferring_timeout, check_data_is_transferring_timeout, True, True)
        except TestEquipmentException as error:
            # Reset cell power for ns1 and ns2
            self._reset_cells_power()
            return (Global.FAILURE, "FTP transfer has been lost after 1st re-selection or CCO")

        if self._b2b_resel:
            # Log the current cell re-selection iteration
            self._logger.info("Performing cell re-selection number 2 of 2")
            # Configure Compressed mode if needed
            if self._compress_mode_active:
                self._ns1_cell.set_cell_power(self._ns1_cell_power)
                self._ns_neighbour_cell_power = self._ns1_cell_power
                limit_power = self._ns2_limit_power
            else:
                limit_power = self._ns2_cell_power

                # Add again 3G neighbor cell in 2G cell neighbor list to force 8960 to send its cell list to UE
                # If not done, re-selection will be failed as UE will not have 8960 cell list
                # Get UTRAN cell list to reload it later
                utran_cell_list = self._ns2_cell.get_utran_neighbor_cell_list()
                # Change UTRAN cell list to force update
                self._ns2_cell.set_utran_neighbor_cell(1, self._ns1_arfcn, 0, 1, 1)
                time.sleep(self._wait_btwn_cmd)
                # Set the retrieved UTRAN cell list if self._ns1_arfcn is in
                if self._ns1_arfcn in utran_cell_list:
                    self._ns2_cell.set_utran_neighbor_cell_list(utran_cell_list)
                else:
                    # Force self._ns1_arfcn in 3G neighbor cell with scrambling code set to 1 as in CellConfigurationNSx.xml files
                    # if in this file scrambling code is changed, please update here
                    self._logger.info("3G cell UARFCN is not set in cell list forcing it")
                    self._ns2_cell.set_utran_neighbor_cell(1, self._ns1_arfcn, 1, 1, 1)
                # Wait 2 minute before DUT receives information on 3G neighbour cell from 2G cell
                time.sleep(sys_info_check_time)
                # Second re-selection from ns2 (self._ns_camped_power) to ns1 (self._ns_neighbour_cell_power)
            try:
                self.decrease_and_increase_cell_power_while_ftp(self._ns2_cell,
                                                                self._ns_camped_power,
                                                                self._ns1_cell,
                                                                self._ns1_data,
                                                                self._ns_neighbour_cell_power,
                                                                self._decrementation_step_power,
                                                                self._decrementation_step_timer,
                                                                limit_power,
                                                                self._incrementation_step_power,
                                                                self._incrementation_step_timer,
                                                                self._ns1_cell_power,
                                                                self._ns2_cell_tech,
                                                                self._ns2_data)
                # As Re-selection has succeed update camped and neighbor cell power
                self._ns_camped_power = self._ns1_cell_power
                self._ns_neighbour_cell_power = limit_power
            except DeviceException as error:
                # Reset cell power for ns1 and ns2
                self._reset_cells_power()

                msg = str(error)
                self._logger.error(msg)
                return (Global.FAILURE, "Re-selection 2 of 2 failed")

            # Check if continuous data transfer is etablished before timeout
            self._ns1_data.check_data_connection_transferring_time(ftp_transferring_timeout, check_data_is_transferring_timeout, True, True)

            self._logger.info("Cell re-selections are successful. Waiting for FTP transfer to finish")

        if not self._b2b_resel:
            self._reset_cells_power()
        if self._compress_mode_active:
            # Reset cell 2 power in case compress mode is active
            self._ns_neighbour_cell_power = self._ns2_cell_power
            self._ns2_cell.set_cell_power(self._ns_neighbour_cell_power)

        # Check if FTP transfer is still ongoing or successful
        # in other case test is failed
        # in all cases stop FTP transfer
        data_connection_state = self._ftp_api.get_ftp_status(self._ftp_task_id)
        self._logger.info("FTP transfer is %s !" % data_connection_state)
        # Stop properly FTP client on DUT side
        try:
            self._ftp_api.stop_ftp(self._ftp_task_id)
        except:
            pass
        if data_connection_state in ("transferring", "transfer successful"):
            return (Global.SUCCESS, "No Errors")
        else:
            msg = self._logger.error("FTP connection lost on DUT, FTP status is %s !" % data_connection_state)
            return (Global.FAILURE, self._error.Msg)

# -----------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """
        # Stop FTP transfer
        try:
            self._ftp_api.stop_ftp(self._ftp_task_id)
        except:
            pass

        # Call LabMobilityBase Run function
        LabMobility3gsmBase.tear_down(self)

        return Global.SUCCESS, "No Errors"

# -----------------------------------------------------------------------------

    def _reset_cells_power(self):
        """
        Set camped cell power and neighbor cell power for the first re-selection
        """
        # init cell powers
        self._ns_camped_power = self._ns1_cell_power
        self._ns_neighbour_cell_power = self._ns2_cell_power

        # Reset cells power
        self._ns1_cell.set_cell_power(self._ns_camped_power)
        self._ns2_cell.set_cell_power(self._ns_neighbour_cell_power)

        # Set Cell Off
        self._ns2_cell.set_cell_off()
