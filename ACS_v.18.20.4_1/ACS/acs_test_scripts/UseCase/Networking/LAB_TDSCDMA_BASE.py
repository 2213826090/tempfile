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
:summary: This file implements LAB TDSCDMA Base UC
:author: mbrisbax
:since:21/07/2014
"""

from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.Utilities.CommunicationUtilities import TelephonyConfigsParser
import time
from acs_test_scripts.Utilities.EquipmentUtilities import get_nw_sim_bench_name


class LabTdscdmaBase(UseCaseBase):

    """
    Lab TD-SCDMA Data Transfer base class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        UseCaseBase.__init__(self, tc_name, global_config)

        # Get TC parameters
        self._registration_timeout = \
            int(self._dut_config.get("registrationTimeout"))

        # Get Network configuration
        self._network = \
            global_config.benchConfig.get_parameters("CELLULAR_NETWORK")
        self._apn = self._network.get_param_value("APN")
        self._ssid = self._network.get_param_value("SSID")

        # Retrieve valid bench name for 3G capability
        self._bench_name = get_nw_sim_bench_name("3G", global_config, self._logger)
        # bench_name should be either NETWORK_SIMULATOR1 or NETWORK_SIMULATOR2
        # In both cases, we need to retrieve the NS number (position in the bench config, either 1 or 2)
        self._ns_number = int(self._bench_name[-1])

        # Read NETWORK_SIMULATOR from BenchConfig.xml
        self._ns_node = \
            global_config.benchConfig.get_parameters(self._bench_name)
        # Retrieve the model of the equipment
        self._ns_model = self._ns_node.get_param_value("Model")
        self._ns_IP_Lan1 = self._ns_node.get_param_value("IP_Lan1")
        self._ns_IP_Lan2 = self._ns_node.get_param_value("IP_Lan2")
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
        self._ns_fast_dormancy = \
            self._ns_node.get_param_value("Fast_Dormancy", "disable")

        # Read the xml Template
        self._cell_power = int(self._tc_parameters.get_param_value("CELL_POWER", "-60"))
        self._cell_band = int(self._tc_parameters.get_param_value("CELL_BAND", "34"))

        # Read the DIRECTION value from UseCase xml Parameter
        self._direction = self._tc_parameters.get_param_value("DIRECTION", "DL")
        self._ip_version = self._tc_parameters.get_param_value("IP_VERSION", "IPV4")

        # Get UECmdLayer for Data Use Cases
        self._networking_api = self._device.get_uecmd("Networking")
        self._modem_api = self._device.get_uecmd("Modem")
        self._phone_system = self._device.get_uecmd("PhoneSystem")

        # Create cellular network simulator and retrieve 3G data API
        self._ns = self._em.get_cellular_network_simulator(self._bench_name, "TD-SCDMA")
        self._ns_cell_3g = self._ns.get_cell_3g()
        self._ns_data_3g = self._ns_cell_3g.get_data()

        self._throughput_targets = TelephonyConfigsParser("Throughput_Targets").\
            parse_tdscdma_theoretical_targets(self._direction)

# ------------------------------------------------------------------------------

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

        # Set the equipment application format WCDMA
        self._ns.switch_app_format("TD-SCDMA")

        # Perform a full preset
        self._ns.perform_full_preset()

        # Set cell off
        self._ns_cell_3g.set_cell_off()

        # Set Frequency points using frequency list
        # Set Amplitude Offset correction using offset list
        # Turning amplitude offset state to ON
        self._ns.configure_amplitude_offset_table()

        # Configure TD-SCDMA cell
        self._set_tdscdma_cell_config(self._ns_number, self._cell_band, self._direction)

        # Set the Equipment IP Address 1
        self._ns.set_ip4_lan_address(self._ns_IP_Lan1)

        # Set the equipment IP_Lan2 to the equipment
        self._ns.set_ip4_lan_address2(self._ns_IP_Lan2)

        # Set the Equipment Subnet mask
        self._ns.set_ip4_subnet_mask(self._ns_Subnet_Mask)

        # Set the Equipment gateway
        self._ns.set_ip4_default_gateway(self._ns_Default_Gateway)

        # Set the DUT IP address 1
        self._ns_data_3g.set_dut_ip_address(1, self._ns_DUT_IP_Address)

        # Set the DUT DNS1
        self._ns_data_3g.set_dut_primary_dns(self._ns_DNS1)

        # Set the DUT DNS2
        self._ns_data_3g.set_dut_secondary_dns(self._ns_DNS2)

        # Set basic cell parameters
        self._ns_cell_3g.set_cell_power(self._cell_power)

        # Phone has to see the cell off!
        self._modem_api.check_cdk_no_registration_bfor_timeout(self._registration_timeout)

        # Set cell on
        self._ns_cell_3g.set_cell_on()

        return Global.SUCCESS, "No errors"

# ------------------------------------------------------------------------------
    def tear_down(self):
        """
        End and dispose the test
        """

        # Call UseCaseBase Tear down
        UseCaseBase.tear_down(self)

        # deactivate PDP context
        self._networking_api.clean_all_data_connections()

        try:
            # Avoid exception during tear down
            # Set Flight Mode On
            self._networking_api.set_flight_mode("on")
        except:
            pass

        # Set Cell OFF
        self._ns_cell_3g.set_cell_off()

        # Disconnect Equipment
        self._ns.release()

        return Global.SUCCESS, "No errors"

    def _set_tdscdma_cell_config(self, eqt_id, cell_band=34, direction="DL"):
        """
        Set the config of the TD-SCDMA cell to optimize the throughput depending on the direction

        :type eqt_id: int
        :param eqt_id: Equipment number

        :type cell_band: int
        :param cell_band: Band to be set on the cell

        :type direction: str
        :param direction: (UL|DL|BOTH)
        """
        self._em.configure_equipments("CellConfigurationNS%d" % eqt_id,
                                      {"type": "COMMON_TD-SCDMA"})

        # Set specific cell parameters for cell band
        self._em.configure_equipments("CellConfigurationNS%d" % eqt_id,
                                      {"type": cell_band})

        if direction.upper() == "UL":
            self._em.configure_equipments("CellConfigurationNS%d" % eqt_id,
                                          {"type": "UL_TD-SCDMA"})
        elif direction.upper() == "DL":
            self._em.configure_equipments("CellConfigurationNS%d" % eqt_id,
                                          {"type": "DL_TD-SCDMA"})
        elif direction.upper() == "BOTH":
            self._em.configure_equipments("CellConfigurationNS%d" % eqt_id,
                                          {"type": "FULL_DUPLEX_TD-SCDMA"})

    def _set_up_registration(self):
        self._networking_api.set_flight_mode("off")

        # Check Data Connection State => ATTACHED before timeout
        self._ns_data_3g.check_data_connection_state("ATTACHED",
                                                     self._registration_timeout,
                                                     False)

        # Check registration state is connected using
        # registrationTimeout from Device_Catalog.xml (Non blocking
        # for this test if function isn't implemented on CDK)
        self._modem_api.check_cdk_registration_bfor_timeout(self._registration_timeout)

    def _set_up_pdp_active(self):
        # Set the APN
        time.sleep(self._wait_btwn_cmd)

        if self._ip_version == "IPV6":
            self._logger.info("Setting APN " + str(self._apn) + " on " + str(self._ip_version))
            self._networking_api.set_apn(self._ssid, self._apn, None, None, self._ip_version)
        elif self._ip_version == "IPV4V6":
            self._logger.info("Setting APN " + str(self._apn) + " on " + str(self._ip_version))
            self._networking_api.set_apn(self._ssid, self._apn, None, None, self._ip_version)
        else:
            self._logger.info("Setting APN " + str(self._apn) + " on IPV4 ")
            self._networking_api.set_apn(self._ssid, self._apn)

        # Activate PDP context
        time.sleep(self._wait_btwn_cmd)
        self._logger.info("Active PDP Context...")
        self._networking_api.activate_pdp_context(self._ssid, False)

        # Force screen on to avoid end of PDP context due to fast dormancy
        self._phone_system.wake_screen()
        self._phone_system.set_phone_screen_lock_on(1)
        # Check Data Connection State => PDP Active before timeout
        self._ns_data_3g.check_data_connection_state("PDP_ACTIVE",
                                                     self._registration_timeout,
                                                     blocking=False)

        # Get RAT from Equipment
        network_type = self._ns_data_3g.get_network_type()

        # Check that DUT is registered on the good RAT
        self._modem_api.check_network_type_before_timeout(network_type,
                                                          self._registration_timeout)
        # Force screen on to avoid end of PDP context due to fast dormancy
        self._phone_system.wake_screen()
        self._phone_system.set_phone_screen_lock_on(1)
        if self._ns_data_3g.get_data_connection_status() != "PDP_ACTIVE":
            self._ns_data_3g.check_data_connection_state("PDP_ACTIVE",
                                                         self._registration_timeout,
                                                         blocking=False)
