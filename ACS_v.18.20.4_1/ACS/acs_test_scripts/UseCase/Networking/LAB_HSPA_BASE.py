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
:summary: This file implements LAB HSPA Base UC
:author: ccontreras
:since:17/10/2010
"""

from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.Utilities.CommunicationUtilities import TelephonyConfigsParser
import acs_test_scripts.Utilities.RegistrationUtilities as RegUtil
import time
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsConfigException import AcsConfigException
from acs_test_scripts.Utilities.EquipmentUtilities import get_nw_sim_bench_name
from acs_test_scripts.Utilities.RegistrationUtilities import check_and_set_minimal_cell_power


class LabHspaBase(UseCaseBase):

    """
    Lab Hspa Data Transfer base class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        UseCaseBase.__init__(self, tc_name, global_config)

        # Get TC parameters
        self._registration_timeout = \
            int(self._dut_config.get("registrationTimeout"))

        # Read maxDlHspaRab from DeviceCatalog.xml
        max_dl_hspa_rab = \
            str(self._dut_config.get("maxDlHspaRab"))

        # Read maxUlHspaRab from DeviceCatalog.xml
        max_ul_hspa_rab = \
            str(self._dut_config.get("maxUlHspaRab"))

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
        self._band = str(self._tc_parameters.get_param_value("CELL_BAND"))
        self._dl_uarfcn = int(self._tc_parameters.get_param_value("DL_UARFCN"))
        # NS_CELL_REL
        self._ns_cell_rel = 7
        self._cell_power = \
            int(self._tc_parameters.get_param_value("CELL_POWER"))
        self._cell_service = \
            str(self._tc_parameters.get_param_value("CELL_SERVICE"))

        # Read the PDP_ACTIVATION value from UseCase xml Parameter (True or False)
        self._pdp_activation = \
            self._tc_parameters.get_param_value("PDP_ACTIVATION", "TRUE")

        self._cqi_scheme = \
            str(self._tc_parameters.get_param_value("CQI_SCHEME"))
        self._mac_d_pdu_size = \
            str(self._tc_parameters.get_param_value("MAC_D_PDU_SIZE"))

        # Read the DIRECTION value from UseCase xml Parameter
        self._direction = self._tc_parameters.get_param_value("DIRECTION")
        self._ip_version = self._tc_parameters.get_param_value("IP_VERSION", "IPV4")

        self._kpi_test = self._tc_parameters.get_param_value("KPI_TEST", False, "str_to_bool")

        # Read the UL_RAB value from UseCase xml Parameter
        # Check if this value is set to MAX or not
        ul_rab_config = str(self._tc_parameters.get_param_value("UL_RAB"))

        if ul_rab_config != "" or ul_rab_config != "None":
            if ul_rab_config == "MAX":
                self._ul_rab = max_ul_hspa_rab
            else:
                self._ul_rab = ul_rab_config
        else:
            self._logger.info("Unknown UL RAB Configuration %s has been chosen" % ul_rab_config)

        # Read the DL_RAB value from UseCase xml Parameter
        # Check if this value is set to MAX or not
        dl_rab_config = str(self._tc_parameters.get_param_value("DL_RAB"))

        if dl_rab_config != "" or dl_rab_config != "None":
            if dl_rab_config == "MAX":
                self._dl_rab = max_dl_hspa_rab
            else:
                self._dl_rab = dl_rab_config
        else:
            self._logger.info("Unkown UL RAB Configuration %s has been chosen" % dl_rab_config)

        # Get the HSDPA category (only if user
        # wants to use HSDPA.
        if self._dl_rab is not None:
            # Retrieve all after "HSDPA_CAT"
            self._hsdpa_cat = self._dl_rab[9:]
        else:
            self._logger.debug("HSDPA Cat parameter not found "
                               "on TC parameters, retrieve from DeviceModel")
            self._hsdpa_cat = \
                int(self._dut_config.get("maxDlHspaRab"))

        self._logger.debug("HSDPA CAT: " + str(self._hsdpa_cat))
        # Get the HSUPA category (only if user
        # wants to use HSUPA.
        if self._ul_rab.find("HSUPA_CAT") != -1:
            # Retrieve all after "HSUPA_CAT"
            self._hsupa_cat = int(self._ul_rab[9:])
            self._logger.debug("HSUPA CAT: " + str(self._hsupa_cat))
        else:
            self._hsupa_cat = None

        self._cpc_tti_value = str(self._tc_parameters.get_param_value("CPC"))
        # Get the CPC parameters (only if user wants to use CPC feature) .
        if self._cpc_tti_value.strip().isdigit():
            self._cpc_tti_value = int(self._cpc_tti_value)
            if self._cpc_tti_value != 2 and self._cpc_tti_value != 10:
                self._cpc_tti_value = None
                self._logger.warning("Wrong CPC parameter : %s ; Value should be 2 or 10 " % self._cpc_tti_value)
        else:
            self._cpc_tti_value = None

        # Initialize further parameters
        self._rbt_channel_type = "HSPA"
        self._ps_data_ergch_information_state = None
        self._cqi = None
        self._tti = None
        self._throughput_targets = None
        self._rrc_inactivity_timer = None

        # Get the HSUPA MS reported category (only if user
        # wants to use HSUPA.
        if self._ul_rab.find("HSUPA_CAT") != -1:
            ul_rab = None
        else:
            ul_rab = self._ul_rab

        # Read the throughput targets
        hsupa_cat = self._hsupa_cat
        # If TC use CPC with a TTI of 10ms and UL category is CAT6
        # Max throughput is 2Mbps in this case so get category 5 max throughput
        # CAT5 has a TTI=10ms
        if self._cpc_tti_value is not None:
            if self._cpc_tti_value == 10 and hsupa_cat == 6:
                hsupa_cat = 5
        (self._throughput_targets, self._cqi, self._tti) = \
            TelephonyConfigsParser("Throughput_Targets").\
            parse_hspa_theoretical_targets(self._hsdpa_cat, hsupa_cat, ul_rab)

        # Get UECmdLayer for Data Use Cases
        self._networking_api = self._device.get_uecmd("Networking")
        self._modem_api = self._device.get_uecmd("Modem")
        self._phone_system = self._device.get_uecmd("PhoneSystem")

        # Create cellular network simulator and retrieve 3G data API
        self._ns = self._em.get_cellular_network_simulator(self._bench_name)
        self._ns_cell_3g = self._ns.get_cell_3g()
        self._ns_data_3g = self._ns_cell_3g.get_data()

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

        else:
            # Setting the version of the first DUT IP to IPV4
            self._ns_data_3g.set_dut_ip_version(1, "IP4")
            # Setting the Agilent to IPV6 router mode.
            self._ns.set_ipv6_router_mode("OFF")

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
            self._dl_uarfcn, self._cell_power)

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
                set_fast_dormancy_support("enable", self._rrc_inactivity_timer)

        # Set Uplink channel mode control to auto
        self._ns_cell_3g.set_uplink_channel_mode("ON")

        # Active SRB  Configuration Control to AUTO
        self._ns_cell_3g.set_srb_config_control("ON")

        # Set the initial PS Data RRC State to DCH
        self._ns_data_3g.set_initial_ps_data_rrc_state("DCH")

        # Sets the GPRS Radio Access Bearer
        if self._ul_rab.find("HSUPA_CAT") != -1:
            self._ns_data_3g.set_gprs_radio_access_bearer("HSUPA", "HSDPA")
        else:
            self._ns_data_3g.set_gprs_radio_access_bearer(self._ul_rab, "HSDPA")

        # Set the category configuration to AUTO
        self._ns_data_3g.set_category_control_mode("ON")

        # Set channel type, the default value is "HSPA"
        self._ns_cell_3g.set_rbt_channel_type(self._rbt_channel_type)

        # Call specific configuration functions
        self._setup_high_categories(self._ns_number)

        # configure HS-DSCH config type (FIXED, REPORTED, USER_DEFINED)
        self._ns_data_3g.set_ps_data_configuration_type(self._cqi_scheme)

        if self._cqi_scheme == "FIXED":
            self._ns_data_3g.set_cqi_value(self._cqi)

        # Set the TTI value
        if self._tti:
            self._ns_data_3g.set_hsupa_tti(self._tti)

        # Call specific configuration functions
        self._setup_cpc_feature(self._ns_number)
        # Set Ps Data E-RGCH Information state to off
        # Only if this param is overridden in the UC
        # The default value is ON to transmit an E-RGCH from test set
        if self._ps_data_ergch_information_state == "OFF":
            self._ns_cell_3g.set_ps_data_ergch_information_state(
                self._ps_data_ergch_information_state)

        # Set PS Data HS-DSCH MAC-d PDU Size
        self._ns_data_3g.set_macd_pdu_size(self._mac_d_pdu_size)

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
        self._modem_api.check_cdk_registration_bfor_timeout(self._registration_timeout)

        # Set the APN
        time.sleep(self._wait_btwn_cmd)
        # Increase cell power if needed
        check_and_set_minimal_cell_power(self._ns_cell_3g, self._modem_api, self._dut_config, self._cell_power)

        if self._ip_version == "IPV6":
            self._logger.info("Setting APN " + str(self._apn) + " on " + str(self._ip_version))
            self._networking_api.set_apn(self._ssid, self._apn, None, None, self._ip_version)
        elif self._ip_version == "IPV4V6":
            self._logger.info("Setting APN " + str(self._apn) + " on " + str(self._ip_version))
            self._networking_api.set_apn(self._ssid, self._apn, None, None, self._ip_version)
        else:
            self._logger.info("Setting APN " + str(self._apn) + " on IPV4 ")
            self._networking_api.set_apn(self._ssid, self._apn)

        RegUtil.configure_ims_for_8960(self._ns_data_3g, self._networking_api, self._logger)

        if self._pdp_activation.lower() == "true":

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
                self._networking_api.reactivate_pdp_context(self._ssid, False)
                self._ns_data_3g.check_data_connection_state("PDP_ACTIVE",
                                                             self._registration_timeout,
                                                             blocking=False)
            # Check IP protocol if needed (not needed for KPI test)
            if not self._kpi_test:
                self._networking_api.check_ip_protocol(self._ip_version)

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

        # Clear UE Info
        self._ns_cell_3g.clear_ue_info()

        # Disconnect Equipment
        self._ns.release()

        return Global.SUCCESS, "No errors"

# ------------------------------------------------------------------------------
    def _setup_high_categories(self, eqt_id=1):
        """
        Configure equipment to specific categories via .xml files
        in order to reach specific throughput
        :type eqt_id: int
        :param eqt_id: Equipment number
        """

        # Booleans to know if specific categories must be configured
        hsupa_spec_cat = False
        hsdpa_spec_cat = False

        if self._hsdpa_cat is not None and self._hsdpa_cat > 8:
            hsdpa_spec_cat = True

        if self._hsupa_cat is not None and self._hsupa_cat > 5:
            hsupa_spec_cat = True

        # If the HSDPA or HSUPA categories needs to tune equipment
        if hsdpa_spec_cat or hsupa_spec_cat:

            # Sets PS Data HS-DSCH MAC-d PDU Size to 656
            self._mac_d_pdu_size = "BITS656"
            self._logger.warning(
                'Override MAC-d PDU size to "BITS656" '
                'in order to reach throughput target')

            # If the HSDPA category needs to tune equipment
            if hsdpa_spec_cat:

                # set cqi scheme to "USER_DEFINED"
                self._cqi_scheme = "USER_DEFINED"
                self._logger.warning(
                    'Override CQI scheme to "USER_DEFINED" '
                    'in order to reach throughput target')

                # Configure network simulator with
                # HSDPA_Categories file

                self._em.configure_equipments("HSDPACategories%d" % eqt_id,
                                              {"category": self._hsdpa_cat})

                self._logger.info("HSDPACategories %s has been set on the equipment" % self._hsdpa_cat)
            # End if hsdpa_spec_cat

            # If the HSUPA category needs to tune equipment
            if hsupa_spec_cat:

                # Configure network simulator with
                # HSUPA_Categories file
                self._em.configure_equipments("HSUPACategories%d" % eqt_id,
                                              {"category": self._hsupa_cat})

                self._logger.info("HSUPACategories %s has been set on the equipment" % self._hsupa_cat)
            # End if hsupa_spec_cat

# ------------------------------------------------------------------------------
    def _setup_cpc_feature(self, eqt_id=1):
        """
        Configure equipment to specific categories via .xml files
        in order to reach specific throughput
        :type eqt_id: int
        :param eqt_id: Equipment number
        """

        if self._cpc_tti_value is not None:
            # Configure network simulator with
            # CPCFeature file
            self._em.configure_equipments("CPCFeature%d" % eqt_id,
                                          {"type": self._cpc_tti_value})
            self._logger.info("CPC Feature has been set on the equipment with TII equal to %s ms." % self._cpc_tti_value)

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
                                     "Current arfcn and pcr arfcn are equal,hard handover can't be checked")

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
