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
:summary: Use Case Base to perform Web Browsing over Network simulator
(for 2G / 3G)
:since: 08/07/2013
:author: lvacheyx
"""
import time
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
import acs_test_scripts.Utilities.RegistrationUtilities as RegUtil
from acs_test_scripts.Utilities.EquipmentUtilities import get_nw_sim_bench_name


class LabWebBrowsingBase(UseCaseBase):
    """
    Constructor
    """
    def __init__(self, tc_name, global_config):

        # Call UseCaseBase init function.
        UseCaseBase.__init__(self, tc_name, global_config)

        # Read the configuration from test case
        self._registration_timeout = \
            int(self._dut_config.get("registrationTimeout"))

        # Read NS_CELL_TECH from test case xml file
        self._cell_tech = \
            str(self._tc_parameters.get_param_value("CELL_TECH","3G"))

        # Read CELL_BAND from xml UseCase parameter file
        self._cell_band = self._tc_parameters.get_param_value("CELL_BAND")

        # NS_CELL_REL
        self._ns_cell_rel = 7

        # Read CELL_SERVICE from test case xml file
        self._cell_service = \
            str(self._tc_parameters.get_param_value("CELL_SERVICE"))

        # Read CELL_POWER from xml UseCase parameter file
        self._cell_power = \
            int(self._tc_parameters.get_param_value("CELL_POWER"))

        # Read the DL_ARFCN value from UseCase xml Parameter
        self._arfcn = \
            int(self._tc_parameters.get_param_value("ARFCN"))

        # Read the CELL_LAC value from UseCase xml Parameter
        self._lac = int(self._tc_parameters.get_param_value("LAC"))
        if self._lac is None:
            self._lac = 10

        # Read the CELL_RAC value from UseCase xml Parameter
        self._rac = int(self._tc_parameters.get_param_value("RAC"))
        if self._rac is None:
            self._rac = 20

        # Read the CELL_MCC value from UseCase xml Parameter
        self._mcc = int(self._tc_parameters.get_param_value("MCC"))
        if self._mcc is None:
            self._mcc = 1

        # Read the CELL_MNC value from UseCase xml Parameter
        self._mnc = int(self._tc_parameters.get_param_value("MNC"))
        if self._mnc is None:
            self._mnc = 1

        # Get Network configuration
        self._network = \
            global_config.benchConfig.get_parameters("CELLULAR_NETWORK")
        self._apn = self._network.get_param_value("APN")
        self._ssid = self._network.get_param_value("SSID")

        # Retrieve valid bench name for provided capability
        self._bench_name = get_nw_sim_bench_name(self._cell_tech, global_config, self._logger)
        # bench_name should be either NETWORK_SIMULATOR1 or NETWORK_SIMULATOR2
        # In both cases, we need to retrieve the NS number (position in the bench config, either 1 or 2)
        self._ns_number = int(self._bench_name[-1])

        # Read NETWORK_SIMULATOR from BenchConfig.xml
        self._ns_node = \
            global_config.benchConfig.get_parameters(self._bench_name)

        # Create cellular network simulator
        self._ns = self._em.get_cellular_network_simulator(self._bench_name)

        # Retrieve the model of the equipment
        self._ns_model = self._ns_node.get_param_value("Model")
        # Retrieve all IP addresses
        self._ns_ip_lan1 = self._ns_node.get_param_value("IP_Lan1")
        self._ns_ip_lan2 = self._ns_node.get_param_value("IP_Lan2")
        self._ns_ip_dut = \
            self._ns_node.get_param_value("DUT_IP_Address")
        self._ns_ip_dns1 = self._ns_node.get_param_value("DNS1")
        self._ns_ip_dns2 = self._ns_node.get_param_value("DNS2")
        self._ns_ip_subnet_mask = \
            self._ns_node.get_param_value("Subnet_Mask")
        self._ns_ip_default_gateway = \
            self._ns_node.get_param_value("Default_Gateway")

        # Retrieve phonesystem and messaging APIs
        self._phonesystem_api = self._device.get_uecmd("PhoneSystem")
        self._networking_api = self._device.get_uecmd("Networking")
        self._modem_api = self._device.get_uecmd("Modem")

#------------------------------------------------------------------------------
    def set_up(self):
        """
        Setting up the test
        """
        # Call UseCaseBase set_up function.
        UseCaseBase.set_up(self)

        # Ensure flight mode off so that GSM sim operator info can be retrieved
        self._networking_api.set_flight_mode("off")
        time.sleep(self._wait_btwn_cmd)

        # Determinates the kind of registration state to wait
        # by comparing test case MCC/MNC parameters to sim MCC/MNC
        sim_info = self._modem_api.get_sim_operator_info()
        if sim_info["MCC"] != self._mcc or\
                sim_info["MNC"] != self._mnc:
            self._wanted_reg_state = "roaming"
            self._networking_api.set_roaming_mode("ON")
        else:
            self._wanted_reg_state = "registered"

        # Clear all data connections
        self._networking_api.clean_all_data_connections()
        time.sleep(self._wait_btwn_cmd)
        # Activate the flight mode.
        self._networking_api.set_flight_mode("on")

        # Instantiate Network Simulator APIs and
        # Initialize the Network Simulator
        self.set_ns_apis_instances()

        # Perform full preset
        self._ns.perform_full_preset()

        # Set cell off
        self._ns_cell.set_cell_off()

        # Set IP addresses for the 2 network simulators
        self._ns.set_ip_addresses(self._ns_ip_lan1,
                                  self._ns_ip_lan2,
                                  self._ns_ip_subnet_mask,
                                  self._ns_ip_default_gateway,
                                  self._ns_ip_dut,
                                  self._ns_ip_dns1,
                                  self._ns_ip_dns2)

        # Call specific configuration functions
        RegUtil.setup_cell(self._ns_number,
                           self._ns_model,
                           self._cell_tech,
                           self._cell_band,
                           self._ns_cell_rel,
                           self._logger)

        # Set Cell Band and ARFCN using CELL_BAND
        # and ARFCN parameters
        # Set cell service using CELL_SERVICE parameter
        # Set Cell Power using CELL_POWER parameter
        # Set Cell LAC using LAC parameter
        # Set Cell RAC using RAC parameter
        # Set Cell MCC using MCC parameter
        # Set Cell MNC using MNC parameter
        self._ns_cell.configure_basic_cell_parameters(
            self._cell_service, self._cell_band,
            self._arfcn, self._cell_power,
            self._lac, self._rac, self._mcc, self._mnc)

        # Disable flight mode
        self._networking_api.set_flight_mode("off")

        # Set the MMS APN in the DUT
        time.sleep(self._wait_btwn_cmd)
        self._logger.info("Setting APN " + str(self._apn) + "...")
        self._networking_api.set_apn(self._ssid,
                                     self._apn)

        # Activate PDP context
        time.sleep(self._wait_btwn_cmd)
        self._logger.info("Active PDP Context...")
        self._networking_api.activate_pdp_context(self._ssid, check=False)

        # Set cell on
        self._ns_cell.set_cell_on()

        # Check Data Connection State => PDP Active before timeout
        self._ns_data.check_data_connection_state("PDP_ACTIVE",
                                                  self._registration_timeout,
                                                  blocking=True)

        # Check registration state is connected using
        # registrationTimeout from Device_Catalog.xml
        self._logger.info("Check network registration status is %s on DUT" %
                          self._wanted_reg_state)
        if self._wanted_reg_state == "roaming":
            self._modem_api.check_cdk_state_bfor_timeout(
                self._wanted_reg_state,
                self._registration_timeout)
        else:
            self._modem_api.check_cdk_registration_bfor_timeout(self._registration_timeout)

        # Check that DUT is registered on the good RAT
        self._modem_api.check_network_type_before_timeout(self._ns_data.get_network_type(),
                                                          self._registration_timeout)

        return Global.SUCCESS, "No error"

#------------------------------------------------------------------------------
    def tear_down(self):
        """
        Tear down
        """
        # Call UseCaseBase tear_down function.
        UseCaseBase.tear_down(self)

        # Lock the screen.
        self._phonesystem_api.set_phone_lock("on")
        self._phonesystem_api.display_off()

        # deactivate PDP context status
        self._networking_api.deactivate_pdp_context()
        if self._wanted_reg_state == "roaming":
            self._networking_api.set_roaming_mode("OFF")
        time.sleep(self._wait_btwn_cmd)

        # Check Data Connection State => ATTACHED before TimeOut
        self._ns_data.check_data_connection_state("ATTACHED",
                                                  self._registration_timeout,
                                                  blocking=False)
        # Set Cell OFF
        self._ns_cell.set_cell_off()

        # Disconnect Equipment
        self._ns.release()

        return Global.SUCCESS, "No error"

#------------------------------------------------------------------------------
    def set_ns_apis_instances(self):
        """
        Instantiate Network Simulator Apis based on
        NS_CELL_TECH parameter value
        """
        if self._cell_tech == "2G":

            self._ns_cell = self._ns.get_cell_2g()
            self._ns_data = self._ns_cell.get_data()
            self._ns_vc = self._ns_cell.get_voice_call()
            self._ns_messaging = self._ns_cell.get_messaging()

            # Connect to equipment
            self._ns.init()

            # Set the equipment application format depending on 3GSM_CELL_TECH.
            self._ns.switch_app_format("GSM/GPRS")

        elif self._cell_tech == "3G":

            self._ns_cell = self._ns.get_cell_3g()
            self._ns_data = self._ns_cell.get_data()
            self._ns_vc = self._ns_cell.get_voice_call()
            self._ns_messaging = self._ns_cell.get_messaging()

            # Connect to equipment
            self._ns.init()

            # Set the equipment application format depending on 3GSM_CELL_TECH.
            self._ns.switch_app_format("WCDMA")
