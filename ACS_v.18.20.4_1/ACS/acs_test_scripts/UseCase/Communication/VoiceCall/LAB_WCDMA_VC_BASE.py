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
:summary: This file implements the LAB WCDMA Voice Call Base UC
:since: 30/03/2010
:author: cbresoli
"""

import time
from acs_test_scripts.Utilities.CommunicationUtilities import ConfigsParser
import acs_test_scripts.Utilities.RegistrationUtilities as RegUtil

from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from acs_test_scripts.Utilities.EquipmentUtilities import get_nw_sim_bench_name


class LabWcdmaVcBase(UseCaseBase):

    """
    Lab Wcdma Voice Call base class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        # Call UseCase base init function
        UseCaseBase.__init__(self, tc_name, global_config)

        # Read registrationTimeout from Device_Catalog.xml
        self._registration_timeout = \
            int(self._dut_config.get("registrationTimeout"))

        # Read CELLULAR_NETWORK parameters from BenchConfig.xml
        self._network = \
            global_config.benchConfig.get_parameters("CELLULAR_NETWORK")
        self._apn = self._network.get_param_value("APN")
        self._ssid = self._network.get_param_value("SSID")

        # Read callSetupTimeout from Device_Catalog.xml
        self._call_setup_time = \
            int(self._dut_config.get("callSetupTimeout"))

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

        # Read CELL_BAND from test case xml file
        self._band = int(self._tc_parameters.get_param_value("CELL_BAND"))

        # NS_CELL_REL
        self._ns_cell_rel = 7

        # Read DL_UARFCN from test case xml file
        self._dl_uarfcn = int(self._tc_parameters.get_param_value("DL_UARFCN"))

        # Read CELL_SERVICE from test case xml file
        self._cell_service = self._tc_parameters.get_param_value("CELL_SERVICE")

        # Read CELL_POWER from test case xml file
        self._cell_power = \
            int(self._tc_parameters.get_param_value("CELL_POWER"))

        # Read VOICE_CODER_RATE from test case xml file
        self._voice_coder_rate = \
            self._tc_parameters.get_param_value("VOICE_CODER_RATE")

        # Read CALL_DURATION from test case xml file
        self._call_duration = \
            int(self._tc_parameters.get_param_value("CALL_DURATION"))

        # Read Mobile Country Code (MCC) from test case xml file
        self._mcc = \
            self._tc_parameters.get_param_value("MCC", 1, int)

        # Read Mobile Network Code (MNC) from test case xml file
        self._mnc = \
            self._tc_parameters.get_param_value("MNC", 1, int)

        # Create cellular network simulator and retrieve 2G voice call interface
        self._ns = self._em.get_cellular_network_simulator(self._bench_name)
        self._ns_cell_3g = self._ns.get_cell_3g()
        self._ns_voice_call_3g = self._ns_cell_3g.get_voice_call()
        self._ns_data_3g = self._ns_cell_3g.get_data()

        # Instantiate generic UECmd for all use cases
        self._modem_api = self._device.get_uecmd("Modem")
        self._voice_call_api = self._device.get_uecmd("VoiceCall")
        self._networking_api = self._device.get_uecmd("Networking")

# ------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """

        # Call UseCase base set_up function
        UseCaseBase.set_up(self)

        # Ensure flight mode off so that GSM sim operator info can be retrieved
        self._networking_api.set_flight_mode("on")
        time.sleep(self._wait_btwn_cmd)

        # Connect to equipment using GPIBBoardId and GPIBAddress parameters
        self._ns.init()

        # Set the equipment application format WCDMA
        self._ns.switch_app_format("WCDMA")

        # Perform a full preset
        self._ns.perform_full_preset()

        # Set Cell off
        self._ns_cell_3g.set_cell_off()

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

        # Set Frequency points using frequency list
        # Set Amplitude Offset correction using offset list
        # Turning amplitude offset state to ON
        self._ns.configure_amplitude_offset_table()

        # Set Cell Band UARFCN (uplink) in auto mode
        self._ns_cell_3g.set_uplink_channel_mode("ON")

        # Set Audio codec using VOICE_CODER_RATE parameter
        self._ns_voice_call_3g.set_audio_codec(self._voice_coder_rate)

        # Set SRB  Configuration Control to auto
        self._ns_cell_3g.set_srb_config_control("ON")

        # Set Cell Power using CELL_POWER parameter
        self._ns_cell_3g.set_cell_power(self._cell_power)

        # Deactivate HSUPA and HSDPA capabilities
        self._ns_data_3g.set_edch_cell_capability("OFF")
        self._ns_data_3g.set_hsdpa_cell_capability("OFF")

        # Set Cell on
        self._ns_cell_3g.set_cell_on()

        # Ensure flight mode off so that GSM sim operator info can be retrieved
        self._networking_api.set_flight_mode("off")

        # Check registration state is connected using
        # registrationTimeout from Device_Catalog.xml
        self._modem_api.check_cdk_registration_bfor_timeout(
            self._registration_timeout)

        # Adapt attachment procedure to CIRCUIT or PACKET
        if "CIRCUIT" not in self._cell_service:
            # Set the APN
            time.sleep(self._wait_btwn_cmd)
            self._logger.info("Setting APN " + str(self._apn) + "...")
            self._networking_api.set_apn(self._ssid, self._apn)

            # Activate PDP context
            time.sleep(self._wait_btwn_cmd)
            self._logger.info("Active PDP Context...")
            self._networking_api.activate_pdp_context(self._ssid, check=False)

            # Check Data Connection State => PDP_ACTIVE before timeout
            RegUtil.check_dut_data_connection_state_before_timeout("PDP_ACTIVE",
                                                                   self._ns_cell_3g,
                                                                   self._networking_api,
                                                                   self._logger,
                                                                   self._registration_timeout,
                                                                   flightmode_cycle=True,
                                                                   blocking=False)
        # Get RAT from Equipment
        network_type = self._ns_data_3g.get_network_type()

        # Check that DUT is registered on the good RAT
        self._modem_api.check_network_type_before_timeout(network_type,
                                                          self._registration_timeout)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """

        # Call UseCase base tear_down function
        UseCaseBase.tear_down(self)

        try:
            # Activate the flight mode.
            self._networking_api.set_flight_mode("on")
        except:
            pass
        # Set cell off
        self._ns_cell_3g.set_cell_off()

        # Clear UE Info
        self._ns_cell_3g.clear_ue_info()

        # Disconnect equipment
        self._ns.release()

        return Global.SUCCESS, "No errors"
