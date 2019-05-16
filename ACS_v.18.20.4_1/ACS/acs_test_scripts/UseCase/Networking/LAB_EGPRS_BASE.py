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

:summary: Use Case EGPRS Base for SMOKE and BAT tests.
:organization: INTEL MCG PSI
:author: ccontreras
:since: 02/09/2010
"""

import time
import acs_test_scripts.Utilities.RegistrationUtilities as RegUtil

from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from acs_test_scripts.Utilities.CommunicationUtilities import TelephonyConfigsParser
from acs_test_scripts.Utilities.EquipmentUtilities import get_nw_sim_bench_name


class LabEgprsBase(UseCaseBase):

    """
    Lab EGRPS Data Transfer base class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        UseCaseBase.__init__(self, tc_name, global_config)

        # Read registrationTimeout from DeviceCatalog.xml
        self._registration_timeout = \
            int(self._dut_config.get("registrationTimeout"))

        # Read maxDlMultislotConfig from DeviceCatalog.xml
        max_dl_multislot_config = \
            str(self._dut_config.get("maxDlMultislotConfig"))

        # Read maxUlMultislotConfig from DeviceCatalog.xml
        max_ul_multislot_config = \
            str(self._dut_config.get("maxUlMultislotConfig"))

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

        # Read CELLULAR_NETWORK parameters from BenchConfig.xml
        self._network = \
            global_config.benchConfig.get_parameters("CELLULAR_NETWORK")
        self._apn = self._network.get_param_value("APN")
        self._ssid = self._network.get_param_value("SSID")

        # Cell Tech is 2G
        self.ns_cell_tech = "2G"

        # Retrieve valid bench name for 2G capability
        self._bench_name = get_nw_sim_bench_name(self.ns_cell_tech, global_config, self._logger)
        # bench_name should be either NETWORK_SIMULATOR1 or NETWORK_SIMULATOR2
        # In both cases, we need to retrieve the NS number (position in the bench config, either 1 or 2)
        self._ns_number = int(self._bench_name[-1])

        # Read NETWORK_SIMULATOR from BenchConfig.xml
        self._ns_node = \
            global_config.benchConfig.get_parameters(self._bench_name)

        # Read the Model of the NETWORK_SIMULATOR
        self._ns_model = self._ns_node.get_param_value("Model")
        self._ns_IP_Lan1 = self._ns_node.get_param_value("IP_Lan1")
        self._ns_IP_Lan2 = \
            self._ns_node.get_param_value("IP_Lan2")
        self._ns_DUT_IP_Address = \
            self._ns_node.get_param_value("DUT_IP_Address")
        self._ns_DUT_IP_Address2 = \
            self._ns_node.get_param_value("DUT_IP_Address2", "")
        self._ns_DNS1 = self._ns_node.get_param_value("DNS1")
        self._ns_DNS2 = self._ns_node.get_param_value("DNS2")
        self._ns_Subnet_Mask = \
            self._ns_node.get_param_value("Subnet_Mask")
        self._ns_Default_Gateway = \
            self._ns_node.get_param_value("Default_Gateway")

        # Read the xml Template
        self._band_name = self._tc_parameters.get_param_value("CELL_BAND")

        # NS_CELL_REL
        self._ns_cell_rel = 7

        self._cell_power = \
            int(self._tc_parameters.get_param_value("CELL_POWER"))

        # Read the DIRECTION value from UseCase xml Parameter
        self._direction = self._tc_parameters.get_param_value("DIRECTION")

        # Read the Multislot Configuration value from UseCase xml Parameter
        # Check if this value is set to MAX or not
        multislot_config = \
            str(self._tc_parameters.get_param_value("MULTISLOT_CONFIG", "MAX"))
        if multislot_config != "" or multislot_config != "None":
            if multislot_config == "MAX" and self._direction == "DL":
                self._multislot = max_dl_multislot_config

            elif multislot_config == "MAX" and self._direction == "UL":
                self._multislot = max_ul_multislot_config

            elif multislot_config == "MAX" and (self._direction == "" or self._direction is None):
                self._multislot = "D3U2"

            elif multislot_config != "MAX":
                self._multislot = multislot_config
        else:
            self._logger.info("Unknown Multislot Configuration %s has been chosen" % multislot_config)

        self._ul_mcs = self._tc_parameters.get_param_value("UL_MCS", "MCS9")
        self._dl_mcs = self._tc_parameters.get_param_value("DL_MCS", "MCS9")
        self._ps_mcs = self._tc_parameters.get_param_value("PS_MCS", "PS1")

        # Read the DIRECTION value from UseCase xml Parameter
        self._direction = self._tc_parameters.get_param_value("DIRECTION")

        # Get UECmdLayer for Data Use Cases
        self._networking_api = self._device.get_uecmd("Networking")
        self._modem_api = self._device.get_uecmd("Modem")

        # Create cellular network simulator and retrieve 2G data interface
        self._ns = self._em.get_cellular_network_simulator(self._bench_name)
        self._ns_cell_2g = self._ns.get_cell_2g()
        self._ns_data_2g = self._ns_cell_2g.get_data()

        # Read the throughput targets
        self._throughput_targets = TelephonyConfigsParser("Throughput_Targets").\
            parse_gprs_egprs_theoretical_targets("EGPRS", self._multislot,
                                               self._dl_mcs, self._ul_mcs)

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        UseCaseBase.set_up(self)

        # Clear all data connections
        self._networking_api.clean_all_data_connections()
        time.sleep(self._wait_btwn_cmd)

        # Activate the flight mode.
        self._networking_api.set_flight_mode("on")

        # Connect to equipment
        self._ns.init()

        # Set the equipment application format GSM/GPRS
        self._ns.switch_app_format("GSM/GPRS")
        time.sleep(self._wait_btwn_cmd)
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
        # Set the DUT IP address 2 if not empty str.
        if self._ns_DUT_IP_Address2 != "":
            self._ns_data_2g.set_dut_ip_address(2, self._ns_DUT_IP_Address2)

        # Set the DUT DNS1
        self._ns_data_2g.set_dut_primary_dns(self._ns_DNS1)

        # Set the DUT DNS2
        self._ns_data_2g.set_dut_secondary_dns(self._ns_DNS2)

        # Call specific configuration functions
        # Some parameters are defined by function
        RegUtil.setup_cell(self._ns_number,
                           self._ns_model,
                           self.ns_cell_tech,
                           self._band_name,
                           self._ns_cell_rel,
                           self._logger)

        # Set Cell Power using CELL_POWER parameter
        self._ns_cell_2g.set_cell_power(self._cell_power)

        # Set connection type to auto
        self._ns_data_2g.set_connection_type("AUTO")

        # Set the multislot configuration
        self._ns_data_2g.set_multislot_config(self._multislot)

        # Set the downlink and uplink Modulation Coding Schema
        # (DL_MCS and UL_MCS)
        self._ns_data_2g.set_pdtch_modulation_coding_scheme(
            self._dl_mcs, self._ul_mcs)

        # Set the Puncturing Modulation Coding Schema (PS_MCS)
        self._ns_data_2g.set_pdtch_puncturing_scheme(self._ps_mcs)

        # Set cell on
        self._ns_cell_2g.set_cell_on()

        self._modem_api.check_cdk_no_registration_bfor_timeout(self._registration_timeout)

        # Deactivate the flight mode.
        self._networking_api.set_flight_mode("off")

        # Check registration state is connected using
        # registrationTimeout from Device_Catalog.xml (Non blocking
        # for this test if function isn't implemented on CDK)
        self._modem_api.check_cdk_registration_bfor_timeout(self._registration_timeout)

        # Set the APN
        time.sleep(self._wait_btwn_cmd)
        self._logger.info("Setting APN " + str(self._apn) + "...")
        self._networking_api.set_apn(self._ssid, self._apn)

        # Activate PDP context
        time.sleep(self._wait_btwn_cmd)
        self._logger.info("Active PDP Context...")
        self._networking_api.activate_pdp_context(self._ssid, check=False)

        # Check Data Connection State => PDP Active before timeout
        self._ns_data_2g.check_data_connection_state("PDP_ACTIVE",
                                                     self._registration_timeout)

        # Get RAT from Equipment
        network_type = self._ns_data_2g.get_network_type()

        # Check that DUT is registered on the good RAT
        self._modem_api.check_network_type_before_timeout(network_type,
                                                          self._registration_timeout)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------
    def tear_down(self):
        """
        End and dispose the test
        """

        # Call UseCaseBase Tear down
        UseCaseBase.tear_down(self)

        try:
            # Activate the flight mode.
            self._networking_api.set_flight_mode("on")
        except:
            pass

        # Set cell off
        self._ns_cell_2g.set_cell_off()

        # Disconnect Equipment
        self._ns.release()

        return Global.SUCCESS, "No errors"
