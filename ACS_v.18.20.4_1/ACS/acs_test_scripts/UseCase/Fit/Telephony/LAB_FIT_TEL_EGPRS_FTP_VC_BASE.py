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
:summary: This file is the base Use Case for FTP and voice call both active with DTM ON/OFF over 2G
:since: 05/04/2012
:author: Lvacheyx
"""

import time
import os
from acs_test_scripts.Utilities.CommunicationUtilities import ConfigsParser
import acs_test_scripts.Utilities.RegistrationUtilities as RegUtil

from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from ErrorHandling.AcsConfigException import AcsConfigException
from acs_test_scripts.Utilities.EquipmentUtilities import get_nw_sim_bench_name


class LabFitTelEgprsftpVcBase(UseCaseBase):

    """
    Usecase base for 2G DTM ON/OFF FTP and Voice call both active
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        UseCaseBase.__init__(self, tc_name, global_config)

        self._ftp_task_id = 0
        self._ftp_filename = None
        self._ftp_direction = None

        # Read registrationTimeout from DeviceCatalog.xml
        self._registration_timeout = \
            int(self._dut_config.get("registrationTimeout"))

        # Read callSetupTimeout from Device_Catalog.xml
        self._call_setup_timeout = \
            int(self._dut_config.get("callSetupTimeout"))

        # Read the LAB_SERVER parameters from BenchConfig.xml
        self._server = \
            global_config.benchConfig.get_parameters("LAB_SERVER")
        self._server_ip_address = self._server.get_param_value("IP")
        self._username = self._server.get_param_value("username")
        self._password = self._server.get_param_value("password")
        if self._server.has_parameter("ftp_path"):
            self._ftp_path = self._server.get_param_value("ftp_path")
        else:
            self._ftp_path = ""

        # Read ftp DIRECTION from testcase xml parameters
        self._direction = self._tc_parameters.get_param_value("DIRECTION")

        # Read the DL_FILE value from UseCase xml Parameter
        self._dlfilename = os.path.join(
            self._ftp_path,
            self._tc_parameters.get_param_value("DL_FILENAME"))
        # Read the UL_FILE value from UseCase xml Parameter
        self._ulfilename = os.path.join(
            self._ftp_path,
            self._tc_parameters.get_param_value("UL_FILENAME"))

        # Read CELLULAR_NETWORK parameters from BenchConfig.xml
        self._network = \
            global_config.benchConfig.get_parameters("CELLULAR_NETWORK")
        self._apn = self._network.get_param_value("APN")
        self._ssid = self._network.get_param_value("SSID")

        # Retrieve valid bench name for 2G capability
        self._bench_name = get_nw_sim_bench_name("2G", global_config, self._logger)

        # Read NETWORK_SIMULATOR from BenchConfig.xml
        self._ns_node = \
            global_config.benchConfig.get_parameters(self._bench_name)
        self._ns_IP_Lan1 = self._ns_node.get_param_value("IP_Lan1")
        self._ns_IP_Lan2 = \
            self._ns_node.get_param_value("IP_Lan2")
        self._ns_DUT_IP_Address = \
            self._ns_node.get_param_value("DUT_IP_Address")
        self._ns_DNS1 = self._ns_node.get_param_value("DNS1")
        self._ns_DNS2 = self._ns_node.get_param_value("DNS2")
        self._ns_Subnet_Mask = \
            self._ns_node.get_param_value("Subnet_Mask")
        self._ns_Default_Gateway = \
            self._ns_node.get_param_value("Default_Gateway")

        # Read the xml Template
        self._band_name = self._tc_parameters.get_param_value("CELL_BAND")
        self._bch_arfcn = int(self._tc_parameters.get_param_value("BCH_ARFCN"))
        self._pdtch_arfcn = \
            int(self._tc_parameters.get_param_value("PDTCH_ARFCN"))
        self._cell_power = \
            int(self._tc_parameters.get_param_value("CELL_POWER"))
        self._multislot = \
            self._tc_parameters.get_param_value("MULTISLOT_CONFIG")
        self._ul_mcs = self._tc_parameters.get_param_value("UL_MCS")
        self._dl_mcs = self._tc_parameters.get_param_value("DL_MCS")
        self._ps_mcs = self._tc_parameters.get_param_value("PS_MCS")

        # Get UECmdLayer for Data and Voice call Use Cases
        self._networking_api = self._device.get_uecmd("Networking")
        self._modem_api = self._device.get_uecmd("Modem")
        self._voicecall_api = self._device.get_uecmd("VoiceCall")

        # set VOICE CODER RATE to FR_AMR_NB_1220
        self._voice_coder_rate = "FR_AMR_NB_1220"

        # Read CALL_DURATION from test case xml file
        self._call_duration = \
            int(self._tc_parameters.get_param_value("CALL_DURATION"))

        # Create cellular network simulator and retrieve 2G data interface
        self._ns = self._em.get_cellular_network_simulator(self._bench_name)

        # Shortcut to get Cell 2G parameters
        self._ns_cell_2g = self._ns.get_cell_2g()
        self._ns_data_2g = self._ns_cell_2g.get_data()
        self._ns_voice_call_2g = self._ns_cell_2g.get_voice_call()

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        UseCaseBase.set_up(self)

        # Clear all data connections
        self._networking_api.clean_all_data_connections()
        time.sleep(self._wait_btwn_cmd)

        # Connect to equipment
        self._ns.init()

        # Set the equipment application format GSM/GPRS
        self._ns.switch_app_format("GSM/GPRS")

        # Perform a full preset
        self._ns.perform_full_preset()

        # Set cell off
        self._ns_cell_2g.set_cell_off()

        # Set serving cell to EGPRS
        self._ns_cell_2g.set_cell_service("EGPRS")

        # Set Frequency points using frequency list
        # Set Amplitude Offset correction using offset list
        # Turning amplitude offset state to ON
        self._ns.configure_amplitude_offset_table()

        # Set the equipment IP address 1
        self._ns.set_ip4_lan_address(self._ns_IP_Lan1)

        # Set the equipment IP_Lan2 to the equipment
        self._ns.set_ip4_lan_address2(self._ns_IP_Lan2)

        # Set the equipment subnet mask
        self._ns.set_ip4_subnet_mask(self._ns_Subnet_Mask)

        # Set the equipment default gateway
        self._ns.set_ip4_default_gateway(self._ns_Default_Gateway)

        # Set the DUT IP address 1
        self._ns_data_2g.set_dut_ip_address(1, self._ns_DUT_IP_Address)

        # Set the DUT DNS1
        self._ns_data_2g.set_dut_primary_dns(self._ns_DNS1)

        # Set the DUT DNS2
        self._ns_data_2g.set_dut_secondary_dns(self._ns_DNS2)

        # Set Cell Power using CELL_POWER parameter
        self._ns_cell_2g.set_cell_power(self._cell_power)

        # Set VOICE_CODER_RATE
        self._ns_voice_call_2g.set_audio_codec(self._voice_coder_rate)

        # Set connection type to auto
        self._ns_data_2g.set_connection_type("AUTO")

        # Set Cell Band  using CELL_BAND parameter
        self._ns_cell_2g.set_band(self._band_name)

        # Set Broadcast Channel Arfcn using BCH_ARFCN parameter
        self._ns_cell_2g.set_bcch_arfcn(self._bch_arfcn)

        # Set PDTCH Arfcn using PDTCH_ARFCN parameter
        self._ns_cell_2g.set_pdtch_arfcn(self._pdtch_arfcn)

        # Set the multislot configuration
        self._ns_data_2g.set_multislot_config(self._multislot)

        # Set the downlink and uplink Modulation Coding Schema
        # (DL_MCS and UL_MCS)
        self._ns_data_2g.set_pdtch_modulation_coding_scheme(
            self._dl_mcs, self._ul_mcs)

        # Set the Puncturing Modulation Coding Schema (PS_MCS)
        self._ns_data_2g.set_pdtch_puncturing_scheme(self._ps_mcs)

        # Set DTM state to ON
        self._ns_cell_2g.set_dtm_state("ON")

        # Set cell on
        self._ns_cell_2g.set_cell_on()

        # Check Data Connection State => ATTACHED before timeout
        RegUtil.check_dut_data_connection_state_before_timeout("ATTACHED",
                                                               self._ns_cell_2g,
                                                               self._networking_api,
                                                               self._logger,
                                                               self._registration_timeout)

        # Check registration state is connected using
        # registrationTimeout from Device_Catalog.xml (Non blocking
        # for this test if function isn't implemented on CDK)
        self._modem_api.check_cdk_registration_bfor_timeout(
            self._registration_timeout)

        # Get RAT from Equipment
        network_type = self._ns_data_2g.get_network_type()

        # Check that DUT is registered on the good RAT
        self._modem_api.check_network_type_before_timeout(network_type,
                                                          self._registration_timeout)

        # Set the APN
        time.sleep(self._wait_btwn_cmd)
        self._logger.info("Setting APN " + str(self._apn) + "...")
        self._networking_api.set_apn(self._ssid, self._apn)

        # Activate PDP context
        time.sleep(self._wait_btwn_cmd)
        self._logger.info("Active PDP Context...")
        self._networking_api.activate_pdp_context(self._ssid)

        # Check Data Connection State => PDP Active before timeout
        self._ns_data_2g.check_data_connection_state("PDP_ACTIVE",
                                                     self._registration_timeout)

        # IF DIRECTION is UL starting upload FTP
        #    Start ftp upload of "put100M" file using  following parameters :
        #        DIRECTION, IP, username, password
        if self._direction == "UL":
            self._ftp_direction = self._uecmd_types.XFER_DIRECTIONS.UL  # pylint: disable=E1101
            self._ftp_filename = self._ulfilename

        # ELIF DIRECTION is DL starting download FTP
        #    Start ftp download of "get500M" file using  following parameters :
        #        DIRECTION, IP, username, password
        elif self._direction == "DL":
            self._ftp_direction = self._uecmd_types.XFER_DIRECTIONS.DL  # pylint: disable=E1101
            self._ftp_filename = self._dlfilename

        # ELSE raise an error
        # Raise an error in case the direction is not known
        else:
            self._logger.error("Unknown ftp direction (%s)" % self._direction)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "Unknown ftp direction.")

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------
    def tear_down(self):
        """
        End and dispose the test
        """

        # Call UseCaseBase Tear down
        UseCaseBase.tear_down(self)

        # Set Cell OFF
        self._ns_cell_2g.set_cell_off()

        # Disconnect Equipment
        self._ns.release()

        return Global.SUCCESS, "No errors"
