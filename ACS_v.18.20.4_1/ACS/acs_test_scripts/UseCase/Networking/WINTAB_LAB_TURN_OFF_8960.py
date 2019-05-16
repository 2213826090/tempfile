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
of the Materials, either expressly, by implication, inducement, estoppel or nn
otherwise. Any license under such intellectual property rights must be express
and approved by Intel in writing.

@organization: INTEL MCG PSI
@summary: This file implements LAB WCDMA Base UC
@author: cbresoli
@since 30/03/2010
"""

import time
from acs_test_scripts.Utilities.CommunicationUtilities import ConfigsParser
import acs_test_scripts.Utilities.RegistrationUtilities as RegUtil
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException


class WINTAB_LAB_TURN_OFF_8960(UseCaseBase):

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

        # Read NETWORK_SIMULATOR1 from BenchConfig.xml
        self._ns_node = \
            global_config.benchConfig.get_parameters("NETWORK_SIMULATOR1")
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
        self._band_name = self._tc_parameters.get_param_value("CELL_BAND")
        self._dl_uarfcn = int(self._tc_parameters.get_param_value("DL_UARFCN"))
        self._cell_power = \
            int(self._tc_parameters.get_param_value("CELL_POWER"))
        self._cell_service = self._tc_parameters.get_param_value("CELL_SERVICE")

        # Read the DIRECTION value from UseCase xml Parameter
        self._direction = self._tc_parameters.get_param_value("DIRECTION")
        self._protocol = self._tc_parameters.get_param_value("PROTOCOL")

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

        # Read Amplitude_Offset_Table file name from Amplitude_Offset_Table.xml
        self._amploffset_filename = \
            self._ns_node.get_param_value("AmplitudeOffsetTable")
        amplitude_file = ConfigsParser(self._amploffset_filename)
        amplitude_table = amplitude_file.parse_amplitude_offset_table()

        # Read frequency_list and offset_list from Amplitude_Offset_Table.xml
        self._frequency_list = amplitude_table.get("FrequencyList")
        self._offset_list = amplitude_table.get("OffsetList")

        # Get UECmdLayer for Data Use Cases
        self._networking_api = self._device.get_uecmd("Networking")
        self._modem_api = self._device.get_uecmd("Modem")

        # Create cellular network simulator and retrieve 3G data API
        self._ns = self._em.get_cellular_network_simulator("NETWORK_SIMULATOR1")
        self._ns_cell_3g = self._ns.get_cell_3g()
        self._data_3g = self._ns_cell_3g.get_data()

        # init wanted registration parameters
        self._wanted_reg_state = "None"
#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        UseCaseBase.set_up(self)

        # Connect to equipment using GPIBAddress and GPIBBoardId
        self._ns.init()

        # Set the equipment application format WCDMA
        self._ns.switch_app_format("WCDMA")
        # Perform a full preset
        self._ns.perform_full_preset()

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """

        # Call Use case base Tear down
        UseCaseBase.tear_down(self)

        # Set Cell OFF
        self._ns_cell_3g.set_cell_off()

        # Clear UE Info
        self._ns_cell_3g.clear_ue_info()

        # Close equipment connection
        self._ns.release()

        return Global.SUCCESS, "No errors"
