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
:summary: This file implements the loading of a web page while DUT is in voice
call on EGPRS only.
:since: 14/03/2013
:author: jduran4x
"""

import time

from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from acs_test_scripts.Utilities.CommunicationUtilities import ConfigsParser
import acs_test_scripts.Utilities.RegistrationUtilities as RegUtil

from UtilitiesFWK.Utilities import Global
from ErrorHandling.AcsConfigException import AcsConfigException
from acs_test_scripts.Utilities.EquipmentUtilities import get_nw_sim_bench_name


class LabFitEgprsVcWebBrowsing(UseCaseBase):

    """
    Lab Voice Call MO/MR + web browsing.
    LabEgprsBase requires and sets FTP parameters that are of no use here
    So this Use case inherits from UseCaseBase instead of LabEgprsBase
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call LabMobilityBase Init function
        UseCaseBase.__init__(self, tc_name, global_config)
        # Read CELLULAR_NETWORK parameters from BenchConfig.xml
        self._network = \
            global_config.benchConfig.get_parameters("CELLULAR_NETWORK")
        self._apn = self._network.get_param_value("APN")
        self._ssid = self._network.get_param_value("SSID")

        # Retrieve valid bench name for 2G capability
        self._bench_name = get_nw_sim_bench_name("2G", global_config, self._logger)

        # Read NETWORK_SIMULATOR from BenchConfig.xml
        self._ns_node = \
            global_config.benchConfig.get_parameters(self._bench_name)
        self._ns_IP_Lan1 = self._ns_node.get_param_value("IP_Lan1")
        self._ns_IP_Lan2 = \
            self._ns_node.get_param_value("IP_Lan2")
        self._ns_DUT_IP_Address = \
            self._ns_node.get_param_value("DUT_IP_Address")
        self._ns_DNS1 = self._ns_node.get_param_value("DNS1")
        self._ns_DNS2 = self._ns_node.get_param_value("DNS2")
        self._ns_Subnet_Mask = \
            self._ns_node.get_param_value("Subnet_Mask")
        self._ns_Default_Gateway = \
            self._ns_node.get_param_value("Default_Gateway")

        # Read the xml Template
        self._band_name = self._tc_parameters.get_param_value("CELL_BAND")
        self._bch_arfcn = int(self._tc_parameters.get_param_value("BCH_ARFCN"))
        self._pdtch_arfcn = \
            int(self._tc_parameters.get_param_value("PDTCH_ARFCN"))
        self._cell_power = \
            int(self._tc_parameters.get_param_value("CELL_POWER"))
        self._multislot = \
            self._tc_parameters.get_param_value("MULTISLOT_CONFIG")
        self._ul_mcs = self._tc_parameters.get_param_value("UL_MCS")
        self._dl_mcs = self._tc_parameters.get_param_value("DL_MCS")
        self._ps_mcs = self._tc_parameters.get_param_value("PS_MCS")

        # Get UECmdLayer for Data and Voice call Use Cases
        self._networking_api = self._device.get_uecmd("Networking")
        self._modem_api = self._device.get_uecmd("Modem")
        self._voicecall_api = self._device.get_uecmd("VoiceCall")

        # set VOICE CODER RATE to FR_AMR_NB_1220
        self._voice_coder_rate = "FR_AMR_NB_1220"

        # Create cellular network simulator and retrieve 2G data interface
        self._ns = self._em.get_cellular_network_simulator(self._bench_name)

        # Shortcut to get Cell 2G parameters
        self._ns_cell_2g = self._ns.get_cell_2g()
        self._ns_data_2g = self._ns_cell_2g.get_data()
        self._ns_voice_call_2g = self._ns_cell_2g.get_voice_call()

        # Read PHONE_NUMBER from testcase xml parameters
        self._phone_number = self._tc_parameters.get_param_value("PHONE_NUMBER")
        if self._phone_number not in (None, ''):
            if str(self._phone_number).isdigit():
                pass
            elif self._phone_number == "[PHONE_NUMBER]":
                self._phone_number = str(self._device.get_phone_number())
            else:
                self._phone_number = None
        else:
            self._phone_number = None

        self._timeout = int(self._tc_parameters.get_param_value("TIMEOUT"))
        self._call_setup_timeout = int(self._tc_parameters.
                                       get_param_value("CALL_SETUP_TIMEOUT"))
        self._browser_type = self._tc_parameters.\
            get_param_value("BROWSER_TYPE").lower()
        self._web_page = self._tc_parameters.get_param_value("WEBSITE_URL")
        if self._web_page in [None, '']:
            self._web_page = "http://" + str(self._ns_IP_Lan1)
        # Read registrationTimeout from DeviceCatalog.xml
        self._registration_timeout = \
            int(self._dut_config.get("registrationTimeout"))

# ------------------------------------------------------------------------------
    def set_up(self):
        """
        Initialize the test
        """
        UseCaseBase.set_up(self)

        # Clear all data connections
        self._networking_api.clean_all_data_connections()
        time.sleep(self._wait_btwn_cmd)

        # Connect to equipment
        self._ns.init()

        # Set the equipment application format GSM/GPRS
        self._ns.switch_app_format("GSM/GPRS")

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

        # Set the DUT DNS1
        self._ns_data_2g.set_dut_primary_dns(self._ns_DNS1)

        # Set the DUT DNS2
        self._ns_data_2g.set_dut_secondary_dns(self._ns_DNS2)

        # Set Cell Power using CELL_POWER parameter
        self._ns_cell_2g.set_cell_power(self._cell_power)

        # Set VOICE_CODER_RATE
        self._ns_voice_call_2g.set_audio_codec(self._voice_coder_rate)

        # Set connection type to auto
        self._ns_data_2g.set_connection_type("AUTO")

        # Set Cell Band  using CELL_BAND parameter
        self._ns_cell_2g.set_band(self._band_name)

        # Set Broadcast Channel Arfcn using BCH_ARFCN parameter
        self._ns_cell_2g.set_bcch_arfcn(self._bch_arfcn)

        # Set PDTCH Arfcn using PDTCH_ARFCN parameter
        self._ns_cell_2g.set_pdtch_arfcn(self._pdtch_arfcn)

        # Set the multislot configuration
        self._ns_data_2g.set_multislot_config(self._multislot)

        # Set the downlink and uplink Modulation Coding Schema
        # (DL_MCS and UL_MCS)
        self._ns_data_2g.set_pdtch_modulation_coding_scheme(
            self._dl_mcs, self._ul_mcs)

        # Set the Puncturing Modulation Coding Schema (PS_MCS)
        self._ns_data_2g.set_pdtch_puncturing_scheme(self._ps_mcs)

        # Set DTM state to ON
        self._ns_cell_2g.set_dtm_state("ON")

        # Set cell on
        self._ns_cell_2g.set_cell_on()

        # Check Data Connection State => ATTACHED before timeout
        RegUtil.check_dut_data_connection_state_before_timeout("ATTACHED",
                                                               self._ns_cell_2g,
                                                               self._networking_api,
                                                               self._logger,
                                                               self._registration_timeout)

        # Check registration state is connected using
        # registrationTimeout from Device_Catalog.xml (Non blocking
        # for this test if function isn't implemented on CDK)
        self._modem_api.check_cdk_registration_bfor_timeout(
            self._registration_timeout)

        # Get RAT from Equipment
        network_type = self._ns_data_2g.get_network_type()

        # Check that DUT is registered on the good RAT
        self._modem_api.check_network_type_before_timeout(network_type,
                                                          self._registration_timeout)

        # Set the APN
        time.sleep(self._wait_btwn_cmd)
        self._logger.info("Setting APN " + str(self._apn) + "...")
        self._networking_api.set_apn(self._ssid, self._apn)

        # Activate PDP context
        time.sleep(self._wait_btwn_cmd)
        self._logger.info("Active PDP Context...")
        self._networking_api.activate_pdp_context(self._ssid)

        # Check Data Connection State => PDP Active before timeout
        self._ns_data_2g.check_data_connection_state("PDP_ACTIVE",
                                                     self._registration_timeout)

        # Check that operator gave a valid Voice call number
        # Perform MO voice call on active network simulator
        if self._phone_number is None:
            # Raise an error message as no valid phone number has been set by the operator
            self._error.Msg = "Phone number has no valid value"
            self._logger.error(self._error.Msg)
            raise AcsConfigException(AcsConfigException.PROHIBITIVE_BEHAVIOR, self._error.Msg)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------
    def run_test(self):
        """
        Execute the test
        """

        # Call LabFitTelEgprsftpVcBase Run function
        UseCaseBase.run_test(self)

        self._voicecall_api.dial(self._phone_number)

        # Check call state "CONNECTED" before callSetupTimeout seconds
        self._ns_voice_call_2g.check_call_connected(self._call_setup_timeout)

        # load the web page
        (result_code, result_msg) = self._networking_api.\
            open_web_browser(self._web_page, self._browser_type,
                             self._timeout)

        # Check call is still connected
        self._ns_voice_call_2g.is_voice_call_connected()

        # Release the voice call
        self._ns_voice_call_2g.voice_call_network_release()

        return result_code, result_msg
