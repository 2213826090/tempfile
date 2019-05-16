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
:summary: This file is the base Use Case for LTE Mobility
:since: 04/06/2013
:author: lvacheyx
"""
import time
from UtilitiesFWK.Utilities import Global
from LAB_MOBILITY_BASE import LabMobilityBase
import acs_test_scripts.Utilities.RegistrationUtilities as RegUtil
from ErrorHandling.AcsConfigException import AcsConfigException


class LabMobilityLte3gsmBase(LabMobilityBase):

    """
    Usecase base for mobility use cases
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        LabMobilityBase.__init__(self, tc_name, global_config)

        # Read the xml Template for LTE Cell Configuration
        # Read LTE_MIMO from test case xml file
        self._ns_lte_mimo_temp = self._tc_parameters.get_param_value("LTE_MIMO")
        if self._ns_lte_mimo_temp not in (None, ''):
            self._ns_lte_mimo = (str(self._ns_lte_mimo_temp).lower() == "true")
        # Read LTE_SIGNAL_MODE from test case xml file
        self._ns_lte_signal_mode = str(self._tc_parameters.get_param_value("LTE_SIGNAL_MODE"))

        # Read LTE_PHYSICAL_CELL_ID from test case xml file
        self._ns_lte_physical_cell_id = str(self._tc_parameters.get_param_value("LTE_PHYSICAL_CELL_ID"))
        # Read LTE_CELL_ID from test case xml file
        self._ns_lte_cell_id = str(self._tc_parameters.get_param_value("LTE_CELL_ID"))
        # Read LTE_CELL_POWER_RFO1 from test case xml file
        self._ns_lte_cell_power_rf1 = float(self._tc_parameters.get_param_value("LTE_CELL_POWER_RFO1"))
        # Read LTE_CELL_POWER_RFO2 from test case xml file
        self._ns_lte_cell_power_rf2 = float(self._tc_parameters.get_param_value("LTE_CELL_POWER_RFO2"))

        # Read NS1 Mobile Country Code (MCC) from test case xml file
        self._ns_lte_mcc = int(self._tc_parameters.get_param_value("LTE_MCC"))

        # Read NS1 Mobile Network Code (MNC) from test case xml file
        self._ns_lte_mnc = int(self._tc_parameters.get_param_value("LTE_MNC"))

        # Read NS2 Mobile Country Code (MCC) from test case xml file
        self._ns_3gsm_mcc = int(self._tc_parameters.get_param_value("3GSM_MCC", "1"))

        # Read NS2 Mobile Network Code (MNC) from test case xml file
        self._ns_3gsm_mnc = int(self._tc_parameters.get_param_value("3GSM_MNC", "1"))
        # Read LTE_CELL_BAND from test case xml file
        self._ns_lte_cell_band = int(self._tc_parameters.get_param_value("LTE_CELL_BAND"))
        # Read LTE_DL_EARFCN from test case xml file
        self._ns_lte_dl_earfcn = int(self._tc_parameters.get_param_value("LTE_DL_EARFCN"))

        # Read Duplex type (Frequency Division Duplex (FDD) or Time Division Duplex (TDD)) from test case xml file
        self._ns_lte_duplex_type = self._tc_parameters.get_param_value("DUPLEX_TYPE", "FDD")
        # Read ANTENNAS NUMBER from test case xml file
        self._antennas_number = self._tc_parameters.get_param_value("ANTENNAS_NUMBER", '')

        # Read LTE_SCENARIO_PATH from test case xml file
        self._ns_lte_scenario_path = str(self._tc_parameters.get_param_value("LTE_SCENARIO_PATH"))

        # Read the xml Template for 3GSM Cell Configuration
        # Read 3GSM_CELL_TECH from test case xml file
        self._ns_3gsm_cell_tech = str(self._tc_parameters.get_param_value("3GSM_CELL_TECH"))

        # Read 3GSM_CELL_BAND from test case xml file
        self._ns_3gsm_cell_band = str(self._tc_parameters.get_param_value("3GSM_CELL_BAND"))

        # NS1_CELL_REL
        self._ns_3gsm_cell_rel = 8

        # Read 3GSM_DL_EARFCN from test case xml file
        self._ns_3gsm_dl_arfcn = int(self._tc_parameters.get_param_value("3GSM_DL_ARFCN"))

        # Read 3GSM_CELL_SERVICE from test case xml file
        self._ns_3gsm_cell_service = str(self._tc_parameters.get_param_value("3GSM_CELL_SERVICE"))

        # Read 3GSM_CELL_POWER from test case xml file
        self._ns_3gsm_cell_power = float(self._tc_parameters.get_param_value("3GSM_CELL_POWER"))

        # Read 3GSM_LAC_VALUE from test case xml file
        self._ns_3gsm_lac = int(self._tc_parameters.get_param_value("3GSM_LAC"))

        # Read 3GSM_RAC_VALUE from test case xml file
        self._ns_3gsm_rac = int(self._tc_parameters.get_param_value("3GSM_RAC"))

# -----------------------------------------------------------------------------

    def set_up(self):
        """
        Set up the test configuration
        """

        # Call LabMobilityBase set_up function
        LabMobilityBase.set_up(self)

        # Ensure flight mode off so that GSM sim operator info can be retrieved
        self._networking_api.set_flight_mode("off")
        time.sleep(self._wait_btwn_cmd)

        # Determinates the kind of registration state to wait
        # by comparing test case MCC/MNC parameters to sim MCC/MNC
        sim_info = self._modem_api.get_sim_operator_info()
        if sim_info["MCC"] != self._ns_lte_mcc or \
           sim_info["MNC"] != self._ns_lte_mnc:
            self._wanted_reg_state = "roaming"
            self._networking_api.set_roaming_mode("ON")
        else:
            self._wanted_reg_state = "registered"

        # Disable data and PDP context
        self._networking_api.deactivate_pdp_context()

        # Enable flight mode
        self._networking_api.set_flight_mode("on")

        # Set the Network Simulators APIs instances
        self._set_ns_apis_instances()

        # Set the equipment application format depending on ACTIVE_CELL_TECH.
        # If 4G switch to ""LTE FDD""
        self._ns_lte.switch_app_format("LTE FDD")

        # Perform full preset on LTE cell
        self._ns_lte.perform_full_preset()

        # Perform full preset on 3GSM cell
        self._ns_3gsm.perform_full_preset()

        # Set EPC off, not for CMW500 because DAU starting is a long lasting time operation
        self._ns_lte_data.set_epc_off()

        # Set cell off on LTE cell
        self._ns_lte_cell.set_cell_off()

        # Set cell off
        self._ns_3gsm_cell.set_cell_off()

        # Set IP addresses for the 3GSM NS
        self._ns_3gsm.set_ip_addresses(self._ns_3gsm_ip_lan1,
                                       self._ns_3gsm_ip_lan2,
                                       self._ns_3gsm_ip_subnet_mask,
                                       self._ns_3gsm_ip_default_gateway,
                                       self._ns_3gsm_ip_dut,
                                       self._ns_3gsm_ip_dns1,
                                       self._ns_3gsm_ip_dns2)

        # Call specific configuration functions
        # Some parameters are defined by function
        RegUtil.setup_cell(self._ns_number,
                           self._ns_model,
                           self._ns_3gsm_cell_tech,
                           self._ns_3gsm_cell_band,
                           self._ns_3gsm_cell_rel,
                           self._logger)

        # Set Cell Band and ARFCN using 3GSM_CELL_BAND
        # and 3GSM_ARFCN parameters
        # Set cell service using 3GSM_CELL_SERVICE parameter
        # Set Cell Power using 3GSM_CELL_POWER parameter
        # Set Cell LAC using 3GSM_LAC_VALUE parameter
        # Set Cell RAC using 3GSM_RAC_VALUE parameter
        self._ns_3gsm_cell.configure_basic_cell_parameters(
            self._ns_3gsm_cell_service, self._ns_3gsm_cell_band,
            self._ns_3gsm_dl_arfcn, self._ns_3gsm_cell_power,
            self._ns_3gsm_lac, self._ns_3gsm_rac)

        # set mcc and mnc of 3gsm network
        self._ns_3gsm_cell.set_mcc(self._ns_3gsm_mcc)
        self._ns_3gsm_cell.set_mnc(self._ns_3gsm_mnc)
        if self._ns_lte_duplex_type == "FDD":
            # Call specific configuration functions for LTE Cell
            RegUtil.setup_cell_lte(self._ns_lte,
                                   self._ns_lte_mcc,
                                   self._ns_lte_mnc,
                                   self._ns_lte_ip_dut,
                                   self._ns_lte_signal_mode,
                                   self._ns_lte_cell_id,
                                   self._ns_lte_physical_cell_id,
                                   self._ns_lte_mimo,
                                   self._ns_lte_cell_power_rf1,
                                   self._ns_lte_cell_power_rf2,
                                   self._ns_lte_scenario_path,
                                   self._ns_lte_cell_band,
                                   self._ns_lte_dl_earfcn,
                                   self._apn)
        elif self._ns_lte_duplex_type == "TDD":
            # Set the DUT IP address
            self._ns_lte_data.set_dut_ip_address(self._ns_dut_ip_Address)
            # Load LTE TDD optimal configuration
            self._ns_lte.load_cell_config("TDD_COMMON", self._ns_number)
            if hasattr(self, "_lte_tdd_config"):
                self._ns_lte.load_cell_config(self._lte_tdd_config, self._ns_number)
            if self._cell_band is not None:
                self._ns_lte_cell.set_cell_band(self._ns_lte_cell_band)
                self._ns_lte_cell.set_downlink_earfcn(self._ns_lte_dl_earfcn)

            # Set MIMO mode
            self._ns_cell_4g.set_lte_tdd_configuration(self._antennas_number,
                                                       self._mimo)
        # Set EPC on
        self._ns_lte_data.set_epc_on()

        # Deactivate E-DCH Cell and HSDPA Cell capabilities
        # if 3GSM NS cell tech is 3G
        if self._ns_3gsm_cell_tech == "3G":
            self._ns_3gsm_data.set_edch_cell_capability("OFF")
            self._ns_3gsm_data.set_hsdpa_cell_capability("OFF")
        # if 3GSM NS cell tech is 2G
        else:
            # Set the multislot configuration
            self._ns_3gsm_data.set_multislot_config(self._multislot)

        return Global.SUCCESS, "No errors"

# -----------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """
        # Call LabMobilityBase tear_down function
        LabMobilityBase.tear_down(self)

        try:
            # Activate the flight mode.
            self._networking_api.set_flight_mode("on")
        except:
            pass

        # Disconnect from external EPC
        self._ns_3gsm.disconnect_from_external_epc()

        # Set cell off on Both NS
        self._ns_3gsm_cell.set_cell_off()
        self._ns_lte_cell.set_cell_off()

        # Disconnect from equipments
        self._ns_3gsm.release()
        self._ns_lte.release()

        return Global.SUCCESS, "No errors"

# ------------------------------------------------------------------------------
    def _set_ns_apis_instances(self):
        """
        Set the Network Simulators APIs instances and the IP addresses
        """
        if self._ns1_model == "AGILENT_E6621A" and self._ns2_model == "AGILENT_8960":

            # Set the IP addresses for 3GSM NS
            self._ns_3gsm_ip_lan1 = self._ns2_ip_lan1
            self._ns_3gsm_ip_lan2 = self._ns2_ip_lan2
            self._ns_3gsm_ip_subnet_mask = self._ns2_ip_subnet_mask
            self._ns_3gsm_ip_default_gateway = self._ns2_ip_default_gateway
            self._ns_3gsm_ip_dut = self._ns2_ip_dut
            self._ns_3gsm_ip_dns1 = self._ns2_ip_dns1
            self._ns_3gsm_ip_dns2 = self._ns2_ip_dns2

            # Set the IP addresses for LTE NS
            self._ns_lte_ip_dut = self._ns1_ip_dut

            # Switch the NS1 APIs instances into LTE APIs instances
            self._ns_lte = self._ns1
            self._ns_lte_cell = self._ns_lte.get_cell_4g()
            self._ns_lte_data = self._ns_lte_cell.get_data()

            # Set self._ns_number to 2 as 8960 is in second position in the bench_config
            # Set self._ns_model with self._ns2_model
            # it will be used to configure the equipment with CellConfigurationNS2.xml
            self._ns_number = 2
            self._ns_model = self._ns2_model

            # Switch the NS2 APIs instances into 3GSM APIs instances
            self._ns_3gsm = self._ns2
            self._ns_lte_ip_lan1 = self._ns1_ip_lan1

            if self._ns_3gsm_cell_tech == "2G":

                self._ns_3gsm_cell = self._ns_3gsm.get_cell_2g()
                self._ns_3gsm_data = self._ns_3gsm_cell.get_data()
                self._ns_3gsm_vc = self._ns_3gsm_cell.get_voice_call()
                self._ns_3gsm_messaging = self._ns_3gsm_cell.get_messaging()

                # Set the equipment application format depending on 3GSM_CELL_TECH.
                self._ns_3gsm.switch_app_format("GSM/GPRS")

            elif self._ns_3gsm_cell_tech == "3G":

                self._ns_3gsm_cell = self._ns_3gsm.get_cell_3g()
                self._ns_3gsm_data = self._ns_3gsm_cell.get_data()
                self._ns_3gsm_vc = self._ns_3gsm_cell.get_voice_call()
                self._ns_3gsm_messaging = self._ns_3gsm_cell.get_messaging()

                # Set the equipment application format depending on 3GSM_CELL_TECH.
                self._ns_3gsm.switch_app_format("WCDMA")

            else:
                self._error.Msg = "Technology %s not supported by the equipment %s" % self._ns2_cell_tech, self._ns2_model
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, self._error.Msg)

        elif self._ns1_model == "AGILENT_8960" and self._ns2_model == "AGILENT_E6621A":

            # Set the IP addresses for 3GSM NS
            self._ns_3gsm_ip_lan1 = self._ns1_ip_lan1
            self._ns_3gsm_ip_lan2 = self._ns1_ip_lan2
            self._ns_3gsm_ip_subnet_mask = self._ns2_ip_subnet_mask
            self._ns_3gsm_ip_default_gateway = self._ns1_ip_default_gateway
            self._ns_3gsm_ip_dut = self._ns1_ip_dut
            self._ns_3gsm_ip_dns1 = self._ns1_ip_dns1
            self._ns_3gsm_ip_dns2 = self._ns1_ip_dns2

            # Set the IP addresses for LTE NS
            self._ns_lte_ip_dut = self._ns2_ip_dut

            # Switch the NS2 APIs instances into LTE APIs instances
            self._ns_lte = self._ns2
            self._ns_lte_cell = self._ns_lte.get_cell_4g()
            self._ns_lte_data = self._ns_lte_cell.get_data()

            # Set self._ns_number to 1 as 8960 is in first position in the bench_config
            # Set self._ns_model with self._ns1_model
            # it will be used to configure the equipment with CellConfigurationNS1.xml
            self._ns_number = 1
            self._ns_model = self._ns1_model

            # Switch the NS1 APIs instances into 3GSM APIs instances
            self._ns_3gsm = self._ns1
            self._ns_lte_ip_lan1 = self._ns2_ip_lan1

            if self._ns_3gsm_cell_tech == "2G":

                self._ns_3gsm_cell = self._ns_3gsm.get_cell_2g()
                self._ns_3gsm_data = self._ns_3gsm_cell.get_data()
                self._ns_3gsm_vc = self._ns_3gsm_cell.get_voice_call()
                self._ns_3gsm_messaging = self._ns_3gsm_cell.get_messaging()

                # Set the equipment application format depending on 3GSM_CELL_TECH.
                self._ns_3gsm.switch_app_format("GSM/GPRS")

            elif self._ns_3gsm_cell_tech == "3G":

                self._ns_3gsm_cell = self._ns_3gsm.get_cell_3g()
                self._ns_3gsm_data = self._ns_3gsm_cell.get_data()
                self._ns_3gsm_vc = self._ns_3gsm_cell.get_voice_call()
                self._ns_3gsm_messaging = self._ns_3gsm_cell.get_messaging()

                # Set the equipment application format depending on 3GSM_CELL_TECH.
                self._ns_3gsm.switch_app_format("WCDMA")

            else:
                self._error.Msg = "Technology %s not supported by the equipment %s" % self._ns1_cell_tech, self._ns1_model
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, self._error.Msg)

        else:
            self._error.Msg = "Bench configuration is not supported by the Usecase"
            raise AcsConfigException(AcsConfigException.INVALID_BENCH_CONFIG, self._error.Msg)

# ------------------------------------------------------------------------------
    def check_call_connection(self):
        """
        Check call state "CONNECTED"
        """
        # Check call state "CONNECTED" before callSetupTimeout seconds
        self._ns_3gsm_vc.check_call_connected(self._call_setup_time, blocking=False)

        # Check call status before callSetupTimeout on the DUT
        self._voicecall_api.wait_for_state(self._uecmd_types.VOICE_CALL_STATE.ACTIVE,  # pylint: disable=E1101
                                           self._call_setup_time)
