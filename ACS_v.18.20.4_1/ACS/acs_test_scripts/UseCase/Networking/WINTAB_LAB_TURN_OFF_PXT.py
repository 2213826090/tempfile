"""
@copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
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

@organization: INTEL MCG PSI
@summary:  This file is the base Use Case for FTP, PING, IPERF, browsing UC [data transfers] over LTE network
@since: 23/04/2012
@author: Chenghua Yang
@comment: BZ3071 - PING MO over LTE network
"""
import time
from acs_test_scripts.Utilities.CommunicationUtilities import ConfigsParser
import acs_test_scripts.Utilities.RegistrationUtilities as RegUtil
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException


class WINTAB_LAB_TURN_OFF_PXT(UseCaseBase):

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
            self._dut_config.get("registrationTimeout")

        # Wait 15 seconds before UE detach
        self._wait_before_ue_detach = 15

        # Get FTP server parameters
        self._server = \
            global_config.benchConfig.get_parameters("LAB_SERVER") # there is no seperate LAB server for LTE
        self._server_ip_address = self._server.get_param_value("IP")
        self._username = self._server.get_param_value("username")
        self._password = self._server.get_param_value("password")
        if self._server.has_parameter("ftp_path"):
            self._ftp_path = self._server.get_param_value("ftp_path")
        else:
            self._ftp_path = ""
        # Read the the ip _version file name from UseCase xml Parameter
        self._ip_version = self._tc_parameters.get_param_value("IP_VERSION", "IPV4")

        # Get the server ipv6 address from the BenchConfig, if the key
        # is present in the file.
        if self._server.has_parameter("IPV6"):
            self._server_ip_v6_address = self._server.get_param_value("IPV6")

        # Get Network configuration
        self._network = \
            global_config.benchConfig.get_parameters("CELLULAR_NETWORK")
        self._apn = self._network.get_param_value("APN")
        self._ssid = self._network.get_param_value("SSID")

        # Read DUT IP Address from BenchConfig.xml
        # and Network simulator Model
        self._ns_node = \
            global_config.benchConfig.get_parameters("NETWORK_SIMULATOR2")  # changed from NETWORK_SIMULATOR1
        self._ns_dut_ip_Address = \
            self._ns_node.get_param_value("DUT_IP_Address")
        self._ns_model = self._ns_node.get_param_value("Model")

        # Read the xml Template
        # Read MIMO from test case xml file
        self._mimo_temp = self._tc_parameters.get_param_value("MIMO", "FALSE")
        if self._mimo_temp not in (None, ''):
            self._mimo = (str(self._mimo_temp).lower() == "true")
        # Read SIGNAL_MODE from test case xml file
        self._signal_mode = \
            str(self._tc_parameters.get_param_value("SIGNAL_MODE"))

        # Read PHYSICAL_CELL_ID from test case xml file
        self._physical_cell_id = \
            str(self._tc_parameters.get_param_value("PHYSICAL_CELL_ID"))
        # Read CELL_ID from test case xml file
        self._cell_id = \
            str(self._tc_parameters.get_param_value("CELL_ID"))
        # Read CELL_POWER_RFO1 from test case xml file
        self._cell_power_rf1 = \
            str(self._tc_parameters.get_param_value("CELL_POWER_RFO1"))
        # Read CELL_POWER_RFO2 from test case xml file
        self._cell_power_rf2 = \
            str(self._tc_parameters.get_param_value("CELL_POWER_RFO2"))

        # Read ANTENNAS NUMBER from test case xml file
        self._antennas_number = \
            self._tc_parameters.get_param_value("ANTENNAS_NUMBER", "1")
        # Read CH_BANDWIDTH from test case xml file
        self._bandwidth = \
            self._tc_parameters.get_param_value("CELL_CHANNEL_BANDWIDTH")
        # Read TRANSMISSION_MODE from test case xml file
        self._transmission_mode = \
            self._tc_parameters.get_param_value("TRANSMISSION_MODE")
        # Read BANDWIDTH from test case xml file
        self._type0_bitmap = \
            str(self._tc_parameters.get_param_value("TYPE0_BITMAP","63"))
        # Read DL_RB_SIZE from test case xml file
        self._dl_rb_size = \
            self._tc_parameters.get_param_value("DL_RB_SIZE")
        # Read UL_RB_SIZE from test case xml file
        self._ul_rb_size = \
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
            int(self._tc_parameters.get_param_value("MCC"))

        # Read Mobile Network Code (MNC) from test case xml file
        self._mnc = \
            int(self._tc_parameters.get_param_value("MNC"))
        # Read CELL_BAND from test case xml file
        self._cell_band = \
            int(self._tc_parameters.get_param_value("CELL_BAND"))
        # Read DL_EARFCN from test case xml file
        self._dl_earfcn = \
            int(self._tc_parameters.get_param_value("DL_EARFCN"))
        # Read SCENARIO_PATH from test case xml file
        self._scenario_path = \
            str(self._tc_parameters.get_param_value("SCENARIO_PATH"))

        # Get UECmdLayer for Data Use Cases
        self._networking_api = self._device.get_uecmd("Networking")
        self._modem_api = self._device.get_uecmd("Modem")

        # Create cellular network simulator
        self._ns = self._em.get_cellular_network_simulator("NETWORK_SIMULATOR2") # changed from Simulator1
        self._cell_4g = self._ns.get_cell_4g()
        self._data_4g = self._ns.get_cell_4g().get_data()

        # init wanted registration parameters to a value that
        # will make the uecmd that used it to raise an error
        self._wanted_reg_state = "None"

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        UseCaseBase.set_up(self)

        # Connect to equipment
        self._ns.init()

        # Perform a full preset
        self._ns.perform_full_preset()

        # Wait a while for the full preset
        time.sleep(self._wait_btwn_cmd)

        # Set EPC off, not for CMW500 because DAU starting is a long lasting time operation
        if self._ns_model == "AGILENT_E6621A":
            self._data_4g.set_epc_off()

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------
    def tear_down(self):
        """
        End and dispose the test
        """

        # Call UseCaseBase Tear down
        UseCaseBase.tear_down(self)

        # Set cell off
        self._cell_4g.set_cell_off()

        # Disconnect Equipment
        self._ns.release()

        return Global.SUCCESS, "No errors"
