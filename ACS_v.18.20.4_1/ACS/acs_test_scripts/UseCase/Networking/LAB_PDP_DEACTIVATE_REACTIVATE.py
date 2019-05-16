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
:summary: This file implements the PDP DEACTIVATE REACTIVATE
:since: 01/07/2013
:author: gcharlex
"""
import time
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from acs_test_scripts.Utilities.EquipmentUtilities import NetworkSimulatorIP
import acs_test_scripts.Utilities.RegistrationUtilities as RegUtil
from acs_test_scripts.Utilities.EquipmentUtilities import get_nw_sim_bench_name


class LabPdpDeactivateReactivate(UseCaseBase):

    """
    PDP Activation and Deactivation Use Case
    """
    def __init__(self, tc_name, global_config):

        # Call UseCaseBase init function.
        UseCaseBase.__init__(self, tc_name, global_config)

        # Get TC parameters
        self._registration_timeout = \
            int(self._dut_config.get("registrationTimeout"))

        # Read NS_CELL_TECH from test case xml file
        self._ns_cell_tech = \
            str(self._tc_parameters.get_param_value("CELL_TECH"))

        self._ns_cell_generation = self._ns_cell_tech
        if self._ns_cell_tech in ["3G", "HSPA"]:
            self._ns_cell_generation = "3G"

        # Read CELL_BAND from xml UseCase parameter file
        self._ns_cell_band = self._tc_parameters.get_param_value("CELL_BAND")

        # NS_CELL_REL
        self._ns_cell_rel = 7

        # Read CELL_SERVICE from test case xml file
        self._ns_cell_service = \
            str(self._tc_parameters.get_param_value("CELL_SERVICE"))

        # Read CELL_POWER from xml UseCase parameter file
        self._ns_cell_power = \
            int(self._tc_parameters.get_param_value("CELL_POWER"))

        # Read the DARFCN value from UseCase xml Parameter
        self._ns_arfcn = \
            int(self._tc_parameters.get_param_value("ARFCN"))

        self._cpc_tti_value = str(self._tc_parameters.get_param_value("CPC", ""))
        # Get the CPC parameters (only if user wants to use CPC feature) .
        if self._cpc_tti_value.strip().isdigit():
            self._cpc_tti_value = int(self._cpc_tti_value)
            if self._cpc_tti_value != 2 and self._cpc_tti_value != 10:
                self._cpc_tti_value = None
                self._logger.warning("Wrong CPC parameter : %s ; Value should be 2 or 10 "
                                     % self._cpc_tti_value)
        else:
            self._cpc_tti_value = None

        # Get Network configuration
        self._network = \
            global_config.benchConfig.get_parameters("CELLULAR_NETWORK")
        self._apn_name = self._network.get_param_value("APN")
        self._apn_ssid = self._network.get_param_value("SSID")

        # Retrieve valid bench name for 3G capability
        self._bench_name = get_nw_sim_bench_name(self._ns_cell_generation, global_config, self._logger)
        # bench_name should be either NETWORK_SIMULATOR1 or NETWORK_SIMULATOR2
        # In both cases, we need to retrieve the NS number (position in the bench config, either 1 or 2)
        self._ns_number = int(self._bench_name[-1])

        # Read NETWORK_SIMULATOR from BenchConfig.xml
        self._ns_node = \
            global_config.benchConfig.get_parameters(self._bench_name)

        # Retrieve the model of the equipment
        self._ns_model = self._ns_node.get_param_value("Model")

        # Retrieve the model IP addresses
        ns_ips = NetworkSimulatorIP(self._logger)
        ns_ips.setup_config(global_config, self._ns_number)
        (self._ns_ip_lan1, self._ns_ip_lan2, self._ns_ip_dut, self._ns_ip_dut2,
         self._ns_ip_dns1, self._ns_ip_dns2, self._ns_ip_subnet_mask,
         self._ns_ip_default_gateway) = ns_ips.get_config()

        # Get UECmdLayer for Data Use Cases
        self._modem_api = self._device.get_uecmd("Modem")
        self._phonesystem_api = self._device.get_uecmd("PhoneSystem")
        self._networking_api = self._device.get_uecmd("Networking")

        # Create cellular network simulator and retrieve 3G data API
        self._ns = self._em.get_cellular_network_simulator(self._bench_name)

        # Get server parameters
        self._server = \
            global_config.benchConfig.get_parameters("LAB_SERVER")
        self._server_ip_address = self._server.get_param_value("IP")

        # Set the default duration of waiting after changing pdp context
        self._sleep_duration = 2

        # Set the default duration of ping checking
        self._ping_check_duration = 10

#------------------------------------------------------------------------------
    def set_up(self):
        # Call UseCaseBase set_up function.
        UseCaseBase.set_up(self)

        # Disable data and PDP context
        self._networking_api.deactivate_pdp_context()

        # Enable flight mode
        self._networking_api.set_flight_mode("on")

        # Set the Network Simulators APIs instances
        self._set_ns_apis_instances()

        # Perform full preset
        self._ns.perform_full_preset()

        # Set cell off
        self._ns_cell.set_cell_off()

        # Set IP addresses
        self._ns.set_ip_addresses(self._ns_ip_lan1,
                                  self._ns_ip_lan2,
                                  self._ns_ip_subnet_mask,
                                  self._ns_ip_default_gateway,
                                  self._ns_ip_dut,
                                  self._ns_ip_dns1,
                                  self._ns_ip_dns2)

        # Call specific configuration functions
        # Some parameters are defined by function
        RegUtil.setup_cell(self._ns_number,
                           self._ns_model,
                           self._ns_cell_generation,
                           self._ns_cell_band,
                           self._ns_cell_rel,
                           self._logger)

        # Set Cell Band and ARFCN using 3GSM_CELL_BAND
        # and 3GSM_ARFCN parameters
        # Set cell service using 3GSM_CELL_SERVICE parameter
        # Set Cell Power using 3GSM_CELL_POWER parameter
        # Set Cell LAC using 3GSM_LAC_VALUE parameter
        # Set Cell RAC using 3GSM_RAC_VALUE parameter
        self._ns_cell.configure_basic_cell_parameters(
            self._ns_cell_service, self._ns_cell_band,
            self._ns_arfcn, self._ns_cell_power)

        # Set CPC Feature
        if self._cpc_tti_value is not None:
            self._em.configure_equipments("CPCFeature%d" % self._ns_number,
                                          {"type": self._cpc_tti_value})
            self._logger.info("CPC Feature has been set on the equipment with TII equal to %s ms."
                              % self._cpc_tti_value)

        if self._ns_cell_tech == "HSPA":
            hsdpa_cat = str(self._dut_config.get("maxDlHspaRab"))[9:]
            self._em.configure_equipments("HSDPACategories%d" % self._ns_number, {"category": hsdpa_cat})
            hsupa_cat = str(self._dut_config.get("maxUlHspaRab"))[9:]
            self._em.configure_equipments("HSDPACategories%d" % self._ns_number, {"category": hsupa_cat})

        # Set cell on
        self._ns_cell.set_cell_on()

        # Disable flight mode
        self._networking_api.set_flight_mode("off")

        # Set the APN in the DUT
        time.sleep(self._wait_btwn_cmd)
        self._logger.info("Setting APN " + str(self._apn_name) + "...")
        self._networking_api.set_apn(self._apn_ssid,
                                     self._apn_name)

        # Activate PDP context
        time.sleep(self._wait_btwn_cmd)
        self._logger.info("Active PDP Context...")
        self._networking_api.activate_pdp_context(self._apn_ssid, check=False)

        # Get RAT from Equipment
        network_type = self._ns_data.get_network_type()

        # Check that DUT is registered on the good RAT
        self._modem_api.check_network_type_before_timeout(network_type,
                                                          self._registration_timeout)
        # Force screen on to avoid end of PDP context due to fast dormancy
        self._phonesystem_api.wake_screen()
        self._phonesystem_api.set_phone_screen_lock_on(1)
        if self._ns_data.get_data_connection_status() != "PDP_ACTIVE":
            self._ns_data.check_data_connection_state("PDP_ACTIVE",
                                                      self._registration_timeout,
                                                      blocking=False)

        self._logger.info("Start continuous ping to " + self._server_ip_address)
        self._networking_api.start_continuous_ping(self._server_ip_address)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------
    def run_test(self):
        """
        Execute the test
        """

        # Call UseCaseBase run_test function.
        UseCaseBase.run_test(self)

        self._logger.info("Check that the ping session still success")
        (ping_success, average_rtt) = \
            self._networking_api.check_continuous_ping_success(self._ping_check_duration)
        if not ping_success:
            return Global.FAILURE, "Ping failed when it should not"
        else:
            self._logger.info("Ping succeeded during %.0fs with average rtt of %.3f ms"
                              % (self._ping_check_duration, average_rtt))

        # De-activate pdp context
        self._networking_api.deactivate_pdp_context(self._apn_ssid)
        time.sleep(self._sleep_duration)

        self._logger.info("Check that the ping session fails")
        (ping_result, rtt) = self._networking_api.get_current_continuous_ping()
        if ping_result:
            return Global.FAILURE, "Ping succeeded when it should not"
        else:
            self._logger.info("Ping failed")

        # Check pdp is attached
        self._ns_data.check_data_connection_state("ATTACHED")

        self._logger.info("Check that the ping session still fails")
        ping_failed = self._networking_api.check_continuous_ping_failure(self._ping_check_duration)
        if not ping_failed:
            return Global.FAILURE, "Ping succeed when it should npt"
        else:
            self._logger.info("Ping failed during %.0fs" % self._ping_check_duration)

        # Re-activate pdp context
        self._networking_api.activate_pdp_context(self._apn_ssid)
        time.sleep(self._sleep_duration)

        self._logger.info("Check that the ping session success")
        (ping_result, rtt) = self._networking_api.get_current_continuous_ping()
        if not ping_result:
            return Global.FAILURE, "Ping failed when it should npt"
        else:
            self._logger.info("Ping succeeded")

        # Check pdp is transfering
        self._ns_data.check_data_connection_state(self._active_state)

        self._logger.info("Check that the ping session still success")
        (ping_success, average_rtt) = \
            self._networking_api.check_continuous_ping_success(self._ping_check_duration)
        if not ping_success:
            return Global.FAILURE, "Ping succeeded when it should not"
        else:
            self._logger.info("Ping succeeded during %.0fs with average rtt of %.3f ms"
                              % (self._ping_check_duration, average_rtt))

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------
    def tear_down(self):

        # Call UseCaseBase tear_down function.
        UseCaseBase.tear_down(self)

        self._logger.info("Stop continuous ping to " + self._server_ip_address)
        self._networking_api.stop_continuous_ping()

        # Set cell off
        self._ns_cell.set_cell_off()

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------
    def _set_ns_apis_instances(self):
        """
        Instantiate Network Simulator Apis based on
        NS_CELL_TECH parameter value
        """
        if self._ns_cell_tech == "2G":
            self._ns_cell = self._ns.get_cell_2g()
            self._ns_data = self._ns_cell.get_data()
            self._active_state = "TRANSFERRING"

            # Connect to equipment
            self._ns.init()
            # Set the equipment application format depending on 3GSM_CELL_TECH.
            self._ns.switch_app_format("GSM/GPRS")

        elif self._ns_cell_tech in ["3G", "HSPA"]:
            self._ns_cell = self._ns.get_cell_3g()
            self._ns_data = self._ns_cell.get_data()
            self._active_state = "PDP_ACTIVE"

            # Connect to equipment
            self._ns.init()
            # Set the equipment application format depending on CELL_TECH.
            self._ns.switch_app_format("WCDMA")
