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
:summary: This file implements LAB WCDMA Base UC
:author: cbresoli
:since:30/03/2010
"""

import time
import acs_test_scripts.Utilities.RegistrationUtilities as RegUtil
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException
from acs_test_scripts.Utilities.CommunicationUtilities import TelephonyConfigsParser
from acs_test_scripts.Utilities.EquipmentUtilities import get_nw_sim_bench_name


class LabWcdmaBase(UseCaseBase):

    """
    Lab Wcdma Data Transfer base class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        UseCaseBase.__init__(self, tc_name, global_config)

        # Get TC parameters
        self._registration_timeout = \
            int(self._dut_config.get("registrationTimeout"))

        # Read maxDlWcdmaRab from DeviceCatalog.xml
        max_dl_wcdma_rab = \
            str(self._dut_config.get("maxDlWcdmaRab"))

        # Read maxUlWcdmaRab from DeviceCatalog.xml
        max_ul_wcdma_rab = \
            str(self._dut_config.get("maxUlWcdmaRab"))

        # Get FTP server parameters
        self._server = \
            global_config.benchConfig.get_parameters("LAB_SERVER")
        self._server_ip_address = self._server.get_param_value("IP")
        self._username = self._server.get_param_value("username")
        self._password = self._server.get_param_value("password")
        if self._server.has_parameter("ftp_path"):
            self._ftp_path = self._server.get_param_value("ftp_path")
        else:
            self._ftp_path = ""

        # Initialize the server ipv6 address to None.
        self._server_ip_v6_address = None

        # Get the server ipv6 address from the BenchConfig, if the key
        # is present in the file.
        if self._server.has_parameter("IPV6"):
            self._server_ip_v6_address = self._server.get_param_value("IPV6")

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
        self._band = self._tc_parameters.get_param_value("CELL_BAND")
        self._dl_uarfcn = int(self._tc_parameters.get_param_value("DL_UARFCN"))
        # NS_CELL_REL
        self._ns_cell_rel = 7
        self._cell_power = \
            int(self._tc_parameters.get_param_value("CELL_POWER"))
        self._cell_service = self._tc_parameters.get_param_value("CELL_SERVICE")

        # Read the DIRECTION value from UseCase xml Parameter
        self._direction = self._tc_parameters.get_param_value("DIRECTION")
        self._ip_version = self._tc_parameters.get_param_value("IP_VERSION", "IPV4")

        # Read the UL_RAB value from UseCase xml Parameter
        # Check if this value is set to MAX or not
        ul_rab_config = str(self._tc_parameters.get_param_value("UL_RAB"))

        if ul_rab_config != "" or ul_rab_config != "None":
            if ul_rab_config == "MAX":
                self._ul_rab = max_ul_wcdma_rab
            else:
                self._ul_rab = ul_rab_config
        else:
            self._logger.info("Unkown UL RAB Configuration %s has been chosen" % ul_rab_config)

        # Read the DL_RAB value from UseCase xml Parameter
        # Check if this value is set to MAX or not
        dl_rab_config = str(self._tc_parameters.get_param_value("DL_RAB"))

        if dl_rab_config != "" or dl_rab_config != "None":
            if dl_rab_config == "MAX":
                self._dl_rab = max_dl_wcdma_rab
            else:
                self._dl_rab = dl_rab_config
        else:
            self._logger.info("Unkown UL RAB Configuration %s has been chosen" % dl_rab_config)

        # Read Home Public Land Mobile Network (HPLMN) Coverage
        # from test case xml file and accept all the different
        # written for True and False values
        self._hplmn_coverage = \
            self._tc_parameters.get_param_value("HPLMN_COVERAGE", "True")

        # Read Mobile Country Code (MCC) from test case xml file
        self._mcc = \
            int(self._tc_parameters.get_param_value("MCC", "1"))

        # Read Mobile Network Code (MNC) from test case xml file
        self._mnc = \
            int(self._tc_parameters.get_param_value("MNC", "1"))

        # Get UECmdLayer for Data Use Cases
        self._networking_api = self._device.get_uecmd("Networking")
        self._modem_api = self._device.get_uecmd("Modem")

        # Create cellular network simulator and retrieve 3G data API
        self._ns = self._em.get_cellular_network_simulator(self._bench_name)
        self._ns_cell_3g = self._ns.get_cell_3g()
        self._ns_data_3g = self._ns_cell_3g.get_data()

        # Read the throughput targets
        self._throughput_targets = TelephonyConfigsParser("Throughput_Targets").\
            parse_wcdma_theoretical_targets(self._dl_rab, self._ul_rab)

        # init wanted registration parameters
        self._wanted_reg_state = "None"
# ------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
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

        # Connect to equipment using GPIBAddress and GPIBBoardId
        self._ns.init()

        # Set the equipment application format WCDMA
        self._ns.switch_app_format("WCDMA")
        time.sleep(self._wait_btwn_cmd)
        # Perform a full preset
        self._ns.perform_full_preset()

        # Set cell off
        self._ns_cell_3g.set_cell_off()

        # Set IP addresses for the 3GSM NS
        self._ns.set_ip_addresses(self._ns_IP_Lan1,
                                  self._ns_IP_Lan2,
                                  self._ns_Subnet_Mask,
                                  self._ns_Default_Gateway,
                                  self._ns_DUT_IP_Address,
                                  self._ns_DNS1,
                                  self._ns_DNS2)

        if self._ip_version == "IPV6":
            # Setting the version of the first DUT IP to IPV6
            self._ns_data_3g.set_dut_ip_version(1, "IP6")
            # Setting the IPv6 Prefix and IID
            self._ns_data_3g.set_ip6_network_parameters(self._server_ip_v6_address)
            # Setting the DUT IPv6 Prefix and IID
            self._ns_data_3g.set_dut_ip6_network_parameters(self._server_ip_v6_address)
            # Setting the Agilent to IPV6 router mode.
            self._ns.set_ipv6_router_mode("ON")
        elif self._ip_version == "IPV4V6":
            # Setting the version of the first DUT IP to IPV6
            self._ns_data_3g.set_dut_ip_version(1, "DUAL")
            # Setting the IPv6 Prefix and IID
            self._ns_data_3g.set_ip6_network_parameters(self._server_ip_v6_address)
            # Setting the DUT IPv6 Prefix and IID
            self._ns_data_3g.set_dut_ip6_network_parameters(self._server_ip_v6_address)
            # Setting the Agilent to IPV6 router mode.
            self._ns.set_ipv6_router_mode("ON")

        # Set the DUT IP address 2 if not empty str.
        if self._ns_DUT_IP_Address2 != "":
            self._ns_data_3g.set_dut_ip_address(2, self._ns_DUT_IP_Address2)

        # 3G default cell setup
        RegUtil.setup_cell(self._ns_number,
                           self._ns_model,
                           "3G",
                           self._band,
                           self._ns_cell_rel,
                           self._logger)

        # Set Cell Band and ARFCN using 3GSM_CELL_BAND
        # and 3GSM_ARFCN parameters
        # Set cell service using 3GSM_CELL_SERVICE parameter
        # Set Cell Power using 3GSM_CELL_POWER parameter
        # Set Cell LAC using 3GSM_LAC_VALUE parameter
        # Set Cell RAC using 3GSM_RAC_VALUE parameter
        self._ns_cell_3g.configure_basic_cell_parameters(
            self._cell_service, self._band,
            self._dl_uarfcn, self._cell_power,
            1, 1, self._mcc, self._mnc)

        message = "Expected network registration state to be %s while " \
            "HPMLN coverage is %s, MCCODE is %s and MNCODE is %s" % (
                str(self._wanted_reg_state),
                str(self._hplmn_coverage),
                str(self._mcc),
                str(self._mnc))
        self._logger.info(message)

        # Set BCH update page state to AUTO
        self._ns_cell_3g.set_bch_update_page_state("AUTO")

        # Set Frequency points using frequency list
        # Set Amplitude Offset correction using offset list
        # Turning amplitude offset state to ON
        self._ns.configure_amplitude_offset_table()

        # If the fast dormancy parameter is set to disable do nothing to
        # avoid regressions.
        if self._ns_fast_dormancy.lower() == "enable":
            # Setting fast dormancy support according to the parameter
            # present in the bench config.
            self._ns_data_3g.\
                set_fast_dormancy_support("enable")

        # Set Uplink channel mode control to auto
        self._ns_cell_3g.set_uplink_channel_mode("ON")

        # Active SRB  Configuration Control to AUTO
        self._ns_cell_3g.set_srb_config_control("ON")

        # Set the initial PS Data RRC State to DCH
        self._ns_data_3g.set_initial_ps_data_rrc_state("DCH")

        # Sets the GPRS Radio Access Bearer
        self._ns_data_3g.set_gprs_radio_access_bearer(self._ul_rab, self._dl_rab)

        # Deactivate HSUPA and HSDPA capabilities
        self._ns_data_3g.set_edch_cell_capability("OFF")
        self._ns_data_3g.set_hsdpa_cell_capability("OFF")

        # Phone has to see the cell off!
        self._modem_api.check_cdk_no_registration_bfor_timeout(self._registration_timeout)

        # Set cell on
        self._ns_cell_3g.set_cell_on()

        # Deactivate the flight mode.
        self._networking_api.set_flight_mode("off")

        # Check Data Connection State => ATTACHED before timeout
        self._ns_data_3g.check_data_connection_state("ATTACHED", self._registration_timeout)

        # Check registration state is connected using
        # registrationTimeout from Device_Catalog.xml (Non blocking
        # for this test if function isn't implemented on CDK)
        self._modem_api.check_cdk_state_bfor_timeout(self._wanted_reg_state,
                                                     self._registration_timeout)

        # Set the APN
        time.sleep(self._wait_btwn_cmd)

        if self._ip_version == "IPV6":
            self._logger.info("Setting APN " + str(self._apn) + " on " + str(self._ip_version) + "...")
            self._networking_api.set_apn(self._ssid, self._apn, None, None, self._ip_version)
        elif self._ip_version == "IPV4V6":
            self._logger.info("Setting APN " + str(self._apn) + " on " + str(self._ip_version) + "...")
            self._networking_api.set_apn(self._ssid, self._apn, None, None, self._ip_version)
        else:
            self._logger.info("Setting APN " + str(self._apn) + " on IPV4 ")
            self._networking_api.set_apn(self._ssid, self._apn)

        if self._wanted_reg_state == "registered":
            # Activate PDP context
            time.sleep(self._wait_btwn_cmd)
            self._logger.info("Active PDP Context...")
            self._networking_api.activate_pdp_context(self._ssid)

            # Check Data Connection State => PDP Active before timeout
            self._ns_data_3g.check_data_connection_state("PDP_ACTIVE",
                                                         self._registration_timeout,
                                                         blocking=False)

            # Get RAT from Equipment
            network_type = self._ns_data_3g.get_network_type()

            # Check that DUT is registered on the good RAT
            self._modem_api.check_network_type_before_timeout(network_type,
                                                              self._registration_timeout)

        return Global.SUCCESS, "No errors"

# ------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """

        # Call Use case base Tear down
        UseCaseBase.tear_down(self)

        # Deactivate PDP context
        if self._wanted_reg_state == "roaming":
            self._networking_api.set_roaming_mode("OFF")

            # Deactivate PDP context
            self._logger.info("Deactive PDP Context...")
            self._networking_api.deactivate_pdp_context(self._ssid)

            # Check Data Connection State => IDLE before TimeOut
            self._ns_data_3g.check_data_connection_state("ATTACHED",
                                                         self._registration_timeout,
                                                         blocking=False)
        else:
            self._networking_api.clean_all_data_connections()
            try:
                # Activate the flight mode.
                self._networking_api.set_flight_mode("on")
            except:
                pass
        # Set Cell OFF
        self._ns_cell_3g.set_cell_off()

        # Clear UE Info
        self._ns_cell_3g.clear_ue_info()

        # Close equipment connection
        self._ns.release()

        return Global.SUCCESS, "No errors"

# ------------------------------------------------------------------------------
    def perform_hard_handover(self, ns_cell):
        """
        Perform Hard Handover

        :type ns_cell: str
        :param ns_cell: the network simulator 's cell

        """

        self._logger.info("Performing a hard handover")

        # Retrieve current downlink channel set by previously
        # loaded configuration
        current_arfcn = ns_cell.get_downlink_arfcn()

        # Retrieve PCR downlink channel set by previously loaded configuration
        pcr_arfcn = ns_cell.get_pcr_downlink_arfcn()

        # Verify that hard handover can be done and checked
        if current_arfcn == pcr_arfcn:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "Current arfcn and pcr arfcn are equal,"
                                     + "hard handover can't be checked")

        # Perform a hard handover using 10 seconds timeout
        ns_cell.execute_hard_handover()
        time.sleep(10)

        # Retrieve the new current ARFCN
        new_current_arfcn = ns_cell.get_downlink_arfcn()

        # Retrieve the new pcr ARFCN
        new_pcr_arfcn = ns_cell.get_pcr_downlink_arfcn()

        # Check that current and pcr arfcn have been inverted
        if new_current_arfcn != pcr_arfcn:
            msg = "Hard handover has failed: current and pcr arfcn" \
                + "haven't been inverted"
            self._logger.error(msg)
            raise DeviceException(DeviceException.PROHIBITIVE_BEHAVIOR, msg)
        # Log the hard handover success
        else:
            self._logger.info("Hard handover succeeded")

        # Update current and pcr ARFCN attributes
        current_arfcn = new_current_arfcn
        pcr_arfcn = new_pcr_arfcn
