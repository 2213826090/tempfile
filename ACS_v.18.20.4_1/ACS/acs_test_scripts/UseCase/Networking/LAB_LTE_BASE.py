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
:summary:  This file is the base Use Case for FTP, PING, IPERF, browsing UC [data transfers] over LTE network
:since: 23/04/2012
:author: lvacheyx
.. note:: BZ3071 - PING MO over LTE network
"""
import time
import acs_test_scripts.Utilities.RegistrationUtilities as RegUtil
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from acs_test_scripts.Utilities.CommunicationUtilities import TelephonyConfigsParser
from ErrorHandling.AcsConfigException import AcsConfigException
from acs_test_scripts.Utilities.NetworkingUtilities import wait_for_dut_ipv4_address
from acs_test_scripts.Utilities.EquipmentUtilities import get_nw_sim_bench_name
from acs_test_scripts.Utilities.RegistrationUtilities import check_and_set_minimal_cell_power


class LabLteBase(UseCaseBase):

    """
    Lab Lte Data Transfer base class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        UseCaseBase.__init__(self, tc_name, global_config)

        # Boot mode selected is MOS
        self._boot_mode_selected = "MOS"

        # Get TC parameters
        self._registration_timeout = \
            int(self._dut_config.get("registrationTimeout"))

        # Wait 15 seconds before UE detach
        self._wait_before_ue_detach = 15

        # Get FTP server parameters
        self._server = \
            global_config.benchConfig.get_parameters("LAB_LTE_SERVER")
        self._server_ip_address = self._server.get_param_value("IP")
        self._username = self._server.get_param_value("username")
        self._password = self._server.get_param_value("password")
        if self._server.has_parameter("ftp_path"):
            self._ftp_path = self._server.get_param_value("ftp_path")
        else:
            self._ftp_path = ""

        # Read the ip_version file name from UseCase xml Parameter
        self._ip_version = self._tc_parameters.get_param_value("IP_VERSION", "IPV4")

        # Get the server ipv6 address from the BenchConfig, if the key
        # is present in the file.
        if self._server.has_parameter("IPV6"):
            self._server_ip_v6_address = self._server.get_param_value("IPV6")

        # Get the apn_ip_version from UseCase xml Parameter
        self._apn_ip_version = self._tc_parameters.get_param_value("APN_IP_VERSION")
        if self._apn_ip_version is None:
            self._apn_ip_version = self._ip_version

        # Get Network configuration
        self._network = \
            global_config.benchConfig.get_parameters("CELLULAR_NETWORK")
        self._apn = self._network.get_param_value("APN")
        self._ssid = self._network.get_param_value("SSID")

        # Retrieve valid bench name for 4G capability
        self._bench_name = get_nw_sim_bench_name("4G", global_config, self._logger)
        # bench_name should be either NETWORK_SIMULATOR1 or NETWORK_SIMULATOR2
        # In both cases, we need to retrieve the NS number (position in the bench config, either 1 or 2)
        self._ns_number = int(self._bench_name[-1])

        # Read NETWORK_SIMULATOR from BenchConfig.xml
        self._ns_node = \
            global_config.benchConfig.get_parameters(self._bench_name)
        self._ns_dut_ip_Address = \
            self._ns_node.get_param_value("DUT_IP_Address")
        # It is not mandatory to have a IPV6 address for a LTE test
        if self._ns_node.has_parameter("DUT_IPV6_Address"):
            self._ns_dut_ipv6_Address = \
                self._ns_node.get_param_value("DUT_IPV6_Address")
        else:
            self._ns_dut_ipv6_Address = None

        self._ns_model = self._ns_node.get_param_value("Model")

        # Read the xml Template
        # Read MIMO from test case xml file
        self._mimo_temp = self._tc_parameters.get_param_value("MIMO", '')
        if self._mimo_temp not in (None, ''):
            self._mimo = (str(self._mimo_temp).lower() == "true")
        else:
            self._mimo = self._mimo_temp
        # Read CARRIER_AGGREGATION
        self._carrier_aggregation = self._tc_parameters.get_param_value('CARRIER_AGGREGATION', 'False', 'str_to_bool')
        # Read SIGNAL_MODE from test case xml file
        self._signal_mode = \
            str(self._tc_parameters.get_param_value("SIGNAL_MODE"))

        # Read PHYSICAL_CELL_ID from test case xml file
        self._physical_cell_id = \
            str(self._tc_parameters.get_param_value("PHYSICAL_CELL_ID"))
        # Read CELL_ID from test case xml file
        self._cell_id = \
            str(self._tc_parameters.get_param_value("CELL_ID", "A"))
        # Read CELL_POWER_RFO1 from test case xml file
        self._cell_power_rf1 = \
            str(self._tc_parameters.get_param_value("CELL_POWER_RFO1", "-40"))
        # Read CELL_POWER_RFO2 from test case xml file
        self._cell_power_rf2 = \
            str(self._tc_parameters.get_param_value("CELL_POWER_RFO2"))

        # Read ANTENNAS NUMBER from test case xml file
        self._antennas_number = \
            self._tc_parameters.get_param_value("ANTENNAS_NUMBER", '')
        # Read CH_BANDWIDTH from test case xml file
        self._bandwidth = \
            self._tc_parameters.get_param_value("CELL_CHANNEL_BANDWIDTH", '')
        # Read TRANSMISSION_MODE from test case xml file
        self._transmission_mode = \
            self._tc_parameters.get_param_value("TRANSMISSION_MODE")
        # Read BANDWIDTH from test case xml file
        self._type0_bitmap = \
            str(self._tc_parameters.get_param_value("TYPE0_BITMAP", "63"))
        # Read DL_RB_SIZE from test case xml file
        self._dl_nb_rb = \
            self._tc_parameters.get_param_value("DL_RB_SIZE")
        # Read UL_RB_SIZE from test case xml file
        self._ul_nb_rb = \
            self._tc_parameters.get_param_value("UL_RB_SIZE")
        # Read I_MCS from test case xml file
        self._dl_i_mcs = \
            self._tc_parameters.get_param_value("DL_I_MCS")
        # Read I_MCS from test case xml file
        self._ul_i_mcs = \
            self._tc_parameters.get_param_value("UL_I_MCS")
        # Read UL Grant Mode from test case xml file
        self._ul_grant_mode = \
            self._tc_parameters.get_param_value("UL_GRANT_MODE", "FIXEDMAC")

        # Read Home Public Land Mobile Network (HPLMN) Coverage
        # from test case xml file and accept all the different
        # written for True and False values
        self._hplmn_coverage = \
            self._tc_parameters.get_param_value("HPLMN_COVERAGE")

        # Read Mobile Country Code (MCC) from test case xml file
        self._mcc = \
            int(self._tc_parameters.get_param_value("MCC", 1))

        # Read Mobile Network Code (MNC) from test case xml file
        self._mnc = \
            int(self._tc_parameters.get_param_value("MNC", 1))

        # Read Duplex type (Frequency Division Duplex (FDD) or Time Division Duplex (TDD)) from test case xml file
        self._duplex_type = \
            self._tc_parameters.get_param_value("DUPLEX_TYPE", "FDD")

        if self._duplex_type == "FDD":
            # Read CELL_BAND from test case xml file
            self._cell_band = \
                int(self._tc_parameters.get_param_value("CELL_BAND", 1))
        else:
            # Read CELL_BAND from test case xml file
            self._cell_band = \
                self._tc_parameters.get_param_value("CELL_BAND", None)
            if self._cell_band in [None, "None"]:
                self._cell_band = None
            else:
                self._cell_band = int(self._cell_band)
        # Read DL_EARFCN from test case xml file
        self._dl_earfcn = \
            int(self._tc_parameters.get_param_value("DL_EARFCN", 300))

        if self._carrier_aggregation:
            # Read SCC_BAND from test case xml file
            self._scc_band = int(self._tc_parameters.get_param_value("SCC_BAND"))
            # Read SCC_EARFCN from test case xml file
            self._scc_earfcn = int(self._tc_parameters.get_param_value("SCC_EARFCN"))
            # Read SCC_BANDWIDTH from test case xml file
            self._scc_bandwidth = self._tc_parameters.get_param_value("SCC_BANDWIDTH")
            # Read SCC_DL_RB_SIZE from test case xml file
            self._scc_dl_nb_rb = \
                self._tc_parameters.get_param_value("SCC_DL_RB_SIZE")
            # Read SCC_I_MCS from test case xml file
            self._scc_dl_i_mcs = \
                self._tc_parameters.get_param_value("SCC_DL_I_MCS")

        # Read SCENARIO_PATH from test case xml file
        self._scenario_path = \
            str(self._tc_parameters.get_param_value("SCENARIO_PATH"))
        # Read IMS from test case xml file to know if IMS service should be enabled or not
        self._ims = \
            str(self._tc_parameters.get_param_value("IMS", "off")).lower()

        # Get UECmdLayer for Data Use Cases
        self._networking_api = self._device.get_uecmd("Networking")
        self._modem_api = self._device.get_uecmd("Modem")

        # Read if we need to switch configuration between TDD/FDD at the end of an iteration
        self._switch_tdd_fdd = self._tc_parameters.get_param_value("SWITCH_TDD_FDD", "False", "str_to_bool")

        # Create cellular network simulator
        self._ns = self._em.get_cellular_network_simulator(self._bench_name)
        self._ns_cell_4g = self._ns.get_cell_4g()
        self._ns_data_4g = self._ns.get_cell_4g().get_data()

        # init wanted registration parameters to a value that
        # will make the uecmd that used it to raise an error
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

        # Flight mode activation
        self._networking_api.set_flight_mode("on")

        # Phone has to see the cell off!
        self._modem_api.check_cdk_no_registration_bfor_timeout(self._registration_timeout)

        # Connect to equipment
        self._ns.init()

        # Perform a full preset
        self._ns.perform_full_preset()

        # Wait a while for the full preset
        time.sleep(self._wait_btwn_cmd)

        # Set EPC off, not for CMW500 because DAU starting is a long lasting time operation
        if self._ns_model == "AGILENT_E6621A":
            self._ns_data_4g.set_epc_off()

        # Set cell off
        self._ns_cell_4g.set_cell_off()

        self._ns.configure_amplitude_offset_table()

        self._check_ip_version()

        if self._ip_version in ["IPV4V6", "IPV6"]:
            self._ns_dut_ip_Address = self._ns_dut_ipv6_Address

        # Perform setup for a FDD cell
        if self._duplex_type == "FDD":
            # Load Cell Configuration
            self._ns.load_cell_config("COMMON_LTE", self._ns_number)
            self._ns.load_cell_config(self._cell_band, self._ns_number)
            if self._carrier_aggregation:
                self._ns.load_cell_config("CAT6", self._ns_number)
            if self._ims == "on":
                self._ns.load_cell_config("COMMON_IMS", self._ns_number)
                if self._ip_version != "IPV4":
                    self._ns_data_4g.set_ims_ip_version(self._ip_version)

            # Call specific configuration functions
            RegUtil.setup_cell_lte(self._ns,
                                   self._mcc,
                                   self._mnc,
                                   self._ns_dut_ip_Address,
                                   self._signal_mode,
                                   self._cell_id,
                                   self._physical_cell_id,
                                   self._mimo,
                                   self._cell_power_rf1,
                                   self._cell_power_rf2,
                                   self._scenario_path,
                                   self._cell_band,
                                   self._dl_earfcn,
                                   self._apn)

            # Set EPC on for Agilent PXT or DAU for RCW500
            self._ns_data_4g.set_epc_on()

            if self._ims == "on":
                self._ns_cell_4g.set_ims_on()

            time.sleep(self._wait_btwn_cmd)

            # stop scenario to set cell parameters
            self._ns.stop_scenario()
            # set configuration for LTE
            self._ns_cell_4g.set_lte_configuration(self._antennas_number,
                                                   self._bandwidth,
                                                   self._transmission_mode,
                                                   self._type0_bitmap,
                                                   self._ul_grant_mode,
                                                   self._dl_nb_rb,
                                                   self._dl_i_mcs,
                                                   self._ul_nb_rb,
                                                   self._ul_i_mcs,
                                                   self._mimo,
                                                   self._carrier_aggregation)

            # Set configugation on Secondary Component Carrier
            if self._carrier_aggregation:
                self._ns_cell_4g.set_secondary_carrier_configuration(self._scc_band,
                                                                     self._scc_earfcn,
                                                                     self._scc_bandwidth,
                                                                     self._cell_power_rf1,
                                                                     self._scc_dl_nb_rb,
                                                                     self._scc_dl_i_mcs)
            # Start the scenario
            self._ns.start_scenario()
        elif self._duplex_type == "TDD":
            if self._ip_version in ["IPV4V6", "IPV6"]:
                # Setting the IPv6 Prefix and IID on the CallBox
                self._ns_data_4g.set_ip6_network_parameters(self._server_ip_v6_address)
            # Set the DUT IP address
            self._ns_data_4g.set_dut_ip_address(self._ns_dut_ip_Address)
            # Load LTE TDD optimal configuration
            self._ns.load_cell_config("TDD_COMMON", self._ns_number)
            if hasattr(self, "_lte_tdd_config"):
                self._ns.load_cell_config(self._lte_tdd_config, self._ns_number)
            if self._cell_band is not None and not self._switch_tdd_fdd:
                self._ns_cell_4g.set_cell_band(self._cell_band)
                self._ns_cell_4g.set_downlink_earfcn(self._dl_earfcn)

            # Set MIMO mode
            self._ns_cell_4g.set_lte_tdd_configuration(self._antennas_number,
                                                       self._mimo)
        else:
            return Global.FAILURE, "Wrong duplex type for LTE: %s it must be FDD or TDD" % self._duplex_type

        return Global.SUCCESS, "No errors"

# ------------------------------------------------------------------------------
    def tear_down(self):
        """
        End and dispose the test
        """

        # Call UseCaseBase Tear down
        UseCaseBase.tear_down(self)

        try:
            # Avoid exception during tear down
            # For instance on windows platforms set_flight_mode
            # use UI code, it can throw an exception if we are on the wrong window
            # Reset roaming flag
            if self._wanted_reg_state == "roaming":
                self._networking_api.set_roaming_mode("OFF")

            # Set Flight Mode On
            self._networking_api.set_flight_mode("on")
        except:
            pass

        # Disable IMS service if it was activated in the set_up
        if self._ims == "on":
            self._ns_cell_4g.set_ims_off()

        # Set cell off
        self._ns_cell_4g.set_cell_off()

        # Disconnect Equipment
        self._ns.release()

        return Global.SUCCESS, "No errors"

# --------------------------------------------------------------------------------------

    def _check_ip_version(self):
        """
        Check ip protocol version for phone and callbox
        """
        # Check the type of IP address assigned by the NW
        self._nw_ip_version = str(self._ns_data_4g.get_ip_address_type())
        # Check if IP version needed is already set
        if self._nw_ip_version == self._ip_version:
            self._logger.info("IP version is already set to: " + self._nw_ip_version)

        else:
            # Set the NW IP version to the version in XML file
            self._nw_ip_version = self._ip_version
            # Check if the version value is valid
            if self._nw_ip_version in ("IPV4", "IPV6", "IPV4V6"):
                # stop scenario in order to change version
                self._ns.stop_scenario()
                # Set the new IP version assigned by the NW
                self._ns_data_4g.set_ip_address_type(self._nw_ip_version)
                # Start scenario
                self._ns.start_scenario()

            else:
                msg = "IP_VERSION possible values are only IPV4, IPV6, or IPV4V6"
                self._logger.error(msg)
                return Global.FAILURE, msg

# ------------------------------------------------------------------------------
    def _check_data_connection_state(self, state, timeout=None):
        """
        Check Data Connection State

        :type state: str
        :param state: IDLE|CON

        :type timeout: int
        :param timeout: Optional (default is None) timeout in second to reach the state.

        """

        if timeout is None:
            timeout = self._registration_timeout

        if state == "CON":
            self._ns_data_4g.check_data_connection_state("CON",
                                                         timeout,
                                                         blocking=False,
                                                         cell_id=self._cell_id)
        else:  # IDLE case
            if self._ns_model == "RS_CMW500":
                # Check Data Connection State => OFF before timeout
                self._ns_data_4g.check_data_connection_state("OFF",
                                                             timeout,
                                                             blocking=False,
                                                             cell_id=self._cell_id)
            else:  # case AGILENT_E6621A
                # Check Data Connection State => IDLE before timeout
                self._ns_data_4g.check_data_connection_state("IDLE",
                                                             timeout,
                                                             blocking=False,
                                                             cell_id=self._cell_id)
        # Increase cell power if needed
        check_and_set_minimal_cell_power(self._ns_cell_4g, self._modem_api, self._dut_config, self._cell_power_rf1)

# ------------------------------------------------------------------------------
    def _set_lte_throughput_settings(self):
        """
        Compute and sets lte parameters depending of the throughput wanted.
        """
        # Getting the LTE category.
        self._lte_category = \
            str(self._tc_parameters.get_param_value("LTE_CATEGORY", None))
        if self._lte_category in [None, "None"]:
            # Read max lteCategory supported from DeviceCatalog.xml
            self._lte_category = str(self._dut_config.get("maxSupportedLteCategory", "3"))

        # Get antennas number and bandwidth for maximum throughput available
        bandwidth, antennas_number = self._get_lte_max_tput_params(self._lte_category)

        if self._antennas_number in (None, ""):
            self._antennas_number = antennas_number
        if self._mimo in (None, ""):
            if antennas_number > 1:
                self._mimo = "True"
            else:
                self._mimo = "False"
        if self._bandwidth in (None, ""):
            self._bandwidth = bandwidth

        self._logger.info("Set Throughput target for LTE %s Category: %s, Bandwidth: %s MHz, %s and %s antenna(s)"
                          % (self._duplex_type, self._lte_category, self._bandwidth, self._direction, self._antennas_number))

        # Get customized failure Targets
        self._failure_targets = \
            str(self._tc_parameters.get_param_value("FAILURE_TARGETS", ""))

        if self._duplex_type == "FDD":
            # Read the throughput targets for FDD mode
            self._throughput_targets, self._lte_parameters = TelephonyConfigsParser("Throughput_Targets").\
                parse_lte_theoretical_targets(self._lte_category, self._bandwidth, self._antennas_number)

            # Set LTE parameters from _Config/Throughput_Targets.xml, only if they aren't define in Test Case...
            if self._dl_nb_rb in (None, ""):
                self._dl_nb_rb = str(self._lte_parameters.dl_nb_rb)
                self._logger.info("Set dl_rb_size to %s" % self._dl_nb_rb)

            if self._ul_nb_rb in (None, ""):
                self._ul_nb_rb = str(self._lte_parameters.ul_nb_rb)
                self._logger.info("Set ul_rb_size to %s" % self._ul_nb_rb)

            if self._dl_i_mcs in (None, ""):
                self._dl_i_mcs = str(self._lte_parameters.dl_i_mcs)
                self._logger.info("Set dl_i_mcs to %s" % self._dl_i_mcs)

            if self._ul_i_mcs in (None, ""):
                self._ul_i_mcs = str(self._lte_parameters.ul_i_mcs)
                self._logger.info("Set ul_i_mcs to %s" % self._ul_i_mcs)

            # Add Secondary Component Carrier throughput if Carrier Aggregation is activated
            if self._carrier_aggregation:
                self._scc_throughput_targets, self._scc_parameters = TelephonyConfigsParser("Throughput_Targets").\
                    parse_lte_theoretical_targets(self._lte_category, self._scc_bandwidth, self._antennas_number)
                self._throughput_targets.add_secondary_carrier_throughput_targets(self._scc_throughput_targets)

                if self._scc_dl_nb_rb in (None, ""):
                    self._scc_dl_nb_rb = str(self._scc_parameters.dl_nb_rb)
                    self._logger.info("Set scc_dl_rb_size to %s" % self._scc_dl_nb_rb)
                if self._scc_dl_i_mcs in (None, ""):
                    self._scc_dl_i_mcs = str(self._scc_parameters.dl_i_mcs)
                    self._logger.info("Set scc_dl_i_mcs to %s" % self._scc_dl_i_mcs)

            if self._transmission_mode in (None, ""):
                if self._mimo:
                    self._lte_parameters.transmission_mode = "TM3"
                self._transmission_mode = self._lte_parameters.transmission_mode
                self._logger.info("Set transmission_mode to %s" % self._transmission_mode)
        elif self._duplex_type == "TDD":
            # Read the throughput targets for TDD mode
            self._throughput_targets, self._lte_tdd_config = TelephonyConfigsParser("Throughput_Targets").\
                parse_lte_tdd_targets(self._lte_category, self._direction)

# ------------------------------------------------------------------------------
    def _get_lte_max_tput_params(self, lte_category):
        """
        Compute and return lte parameters to reach the phone maximum throughput.
        Read max lteCategory supported from DeviceCatalog.xml
        get antennas number and bandwidth for maximum throughput available for this category

        :type lte_category: str
        :param lte_category: the lte category of the test
        :rtype: str
        :return: bw, the maximum bandwidth supported by phone
        :rtype: str
        :return: antennas_number, the maximum number of antenna supported by phone
        """
        antennas_number = "1"
        bw = "10"
        if int(lte_category) > 1:
            antennas_number = "2"
        if int(lte_category) > 2:
            bw = "20"

        return bw, antennas_number

    def _set_apn_for_lte_and_ims(self):
        # Set the APN
        time.sleep(self._wait_btwn_cmd)
        self._logger.info("Setting APN " + str(self._apn) + "...")

        # Check whether an APN is already set on the phone, and that
        # this APN corresponds to the one defined in BenchConfig.xml
        # For IMS, set an additional APN depending on IP version without removing previous one
        if self._ims == "on":
            if self._apn_ip_version == "IPV4":
                self._logger.info("Setting APN for IMS " + str(self._apn) + " on protocol IPV4 ")
                # APN for AP centric
                self._networking_api.set_apn(self._ssid, self._apn, None, None, "IP", None, "default,supl,ims")
                # APN for BP centric
                self._networking_api.set_apn("ims", "ims", None, None, "IP", None, None, False, False)
                # APN for IMS Emergency call
                self._networking_api.set_apn("emi", "emergency", None, None, "IP", None, "emergency", False, False)
            else:
                self._logger.info("Setting APN for IMS " + str(self._apn) + " on protocol " + str(self._apn_ip_version))
                # APN for AP centric
                self._networking_api.set_apn(self._ssid, self._apn, None, None, self._apn_ip_version, None,
                                             "default,supl,ims")
                # APN for BP centric
                self._networking_api.set_apn("ims", "ims", None, None, self._apn_ip_version, None, None, False, False)
                # APN for IMS Emergency call
                self._networking_api.set_apn("emi", "emergency", None, None, self._apn_ip_version, None, "emergency",
                                             False, False)
        else:
            if self._apn_ip_version == "IPV4":
                self._logger.info("Setting APN " + str(self._apn) + " on protocol IPV4 ")
                self._networking_api.set_apn(self._ssid, self._apn)
            else:
                self._logger.info("Setting APN " + str(self._apn) + " on protocol " + str(self._apn_ip_version))
                self._networking_api.set_apn(self._ssid, self._apn, None, None, self._apn_ip_version)

    def _setup_ftp_for_lte(self):
        """
        Setup FTP for LTE use cases (used in LabLteFtp and LabLteUsbTether classes).

        :rtype: integer
        :return: function execution status

        :rtype: str
        :return: function execution comment
        """
        # configure RRC state after cell attach process (will be used on LabLteBase set_up)
        if "IDLE" in str(self._rrc_state):
            self._ns_cell_4g.keep_rrc_connection(False)
        else:
            self._ns_cell_4g.keep_rrc_connection(True)

        (code, message) = LabLteBase.set_up(self)

        if code == Global.SUCCESS:
            # Set Cell on
            self._ns_cell_4g.set_cell_on(self._mimo)

            # Flight mode deactivation
            self._networking_api.set_flight_mode("off")

            # Check registration state is connected using
            # registrationTimeout from Device_Catalog.xml
            self._logger.info("Check network registration status is %s on DUT" %
                              self._wanted_reg_state)

            self._modem_api.check_cdk_state_bfor_timeout(
                self._wanted_reg_state,
                self._registration_timeout)

            # Set APN for LTE and/or IMS depending on protocol IPv4 or IPv6
            self._set_apn_for_lte_and_ims()

            # Enable Data Usage
            time.sleep(self._wait_btwn_cmd)
            self._networking_api.activate_pdp_context(check=False)

            # Check data connection state is "CON"
            self._check_data_connection_state("CON")

            # Get RAT from Equipment
            network_type = self._ns_data_4g.get_network_type()
            # Check that DUT is registered on the good RAT
            self._modem_api.check_network_type_before_timeout(network_type,
                                                              self._registration_timeout)
            # Get on DUT its IP address
            if self._ip_version == "IPV4":
                self._ns_dut_ip_Address = wait_for_dut_ipv4_address(self._registration_timeout, self._networking_api, self._device.get_cellular_network_interface())

            current_rrc_state = self._ns_cell_4g.get_rrc_state()
            if self._rrc_state == "RRC_IDLE":
                self._networking_api.disable_output_traffic()
                # Force detaching on PXT
                if self._ns_model == "AGILENT_E6621A":
                    self._ns_data_4g.ue_detach()
                self._ns_cell_4g.check_rrc_state_before_timeout("IDLE", 60)
                current_rrc_state = self._ns_cell_4g.get_rrc_state()
                self._networking_api.enable_output_traffic()
                # self._data_4g.release_rrc_connection()
                self._logger.info("The FTP will start from %s state" % current_rrc_state)
            elif self._rrc_state == "RRC_CONNECTED":
                self._logger.info("The FTP will start from %s state" % current_rrc_state)
            # Start FTP service on equipment side
            self._ns_data_4g.start_ftp_service()

            # Selecting the  IPV6 address of the FTP server, according to
            # the TC parameter value.
            if self._ip_version == "IPV6":
                if self._server_ip_v6_address is not None:
                    # If Protocol is IPV6 use IPV6 address.
                    log_msg = "Using IPV6 address to connect to the FTP server."
                    self._logger.info(log_msg)
                    self._ip_address = self._server_ip_v6_address
                else:
                    # If IPV6 address is not present in the BenchConfig.
                    msg = "The IPV6 parameter is missing from the Bench Config!"
                    raise AcsConfigException(AcsConfigException.INVALID_BENCH_CONFIG, msg)
            else:
                self._ip_address = self._server_ip_address

        return (code, message)
