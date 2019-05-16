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
:summary: Use Case GSM Voice Call for SMOKE and BAT tests.
:since: 19/08/2010
:author: dgo
"""

import time
from acs_test_scripts.Utilities.CommunicationUtilities import ConfigsParser
import acs_test_scripts.Utilities.RegistrationUtilities as RegUtil

from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from acs_test_scripts.Utilities.EquipmentUtilities import get_nw_sim_bench_name


class LabGsmVcBase(UseCaseBase):

    """
    Lab Gsm Voice Call base class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        # Call UseCase base Init function
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

        # Retrieve valid bench name for 2G capability
        self._bench_name = get_nw_sim_bench_name("2G", global_config, self._logger)

        # Read NETWORK_SIMULATOR from BenchConfig.xml
        self._ns_node = \
            global_config.benchConfig.get_parameters(self._bench_name)

        self._band_name = self._tc_parameters.get_param_value("CELL_BAND")

        # Read BCH_ARFCN from test case xml file
        self._bch_arfcn = int(self._tc_parameters.get_param_value("BCH_ARFCN"))

        # Read TCH_ARFCN from test case xml file
        self._tch_arfcn = int(self._tc_parameters.get_param_value("TCH_ARFCN"))

        # Read CELL_SERVICE from test case xml file
        self._cell_service = \
            self._tc_parameters.get_param_value("CELL_SERVICE")

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
            int(self._tc_parameters.get_param_value("MCC"))

        # Read Mobile Network Code (MNC) from test case xml file
        self._mnc = \
            int(self._tc_parameters.get_param_value("MNC"))

        # Create cellular network simulator and retrieve 2G voice call interface
        self._ns = self._em.get_cellular_network_simulator(self._bench_name)
        self._ns_cell_2g = self._ns.get_cell_2g()
        self._ns_voice_call_2g = self._ns_cell_2g.get_voice_call()
        self._ns_data_2g = self._ns_cell_2g.get_data()

        # Instantiate generic UECmd for voiceCall Ucs
        self._modem_api = self._device.get_uecmd("Modem")
        self._voicecall_api = self._device.get_uecmd("VoiceCall")
        self._networking_api = self._device.get_uecmd("Networking")

        # init wanted registration parameters to a value that
        # will make the uecmd that used it to raise an error
        self._wanted_reg_state = "None"

# ------------------------------------------------------------------------------
    def set_up(self):
        """
        Set up the test configuration
        """

        # Call use case base Setup function
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
        else:
            self._wanted_reg_state = "registered"

        # Clear all data connections
        self._networking_api.clean_all_data_connections()
        time.sleep(self._wait_btwn_cmd)

        # Enable flight mode
        self._networking_api.set_flight_mode("on")

        # Connect to equipment using GPIBAddress and GPIBBoardId
        self._ns.init()

        # Set the equipment application format GSM/GPRS
        self._ns.switch_app_format("GSM/GPRS")

        # Perform a full preset
        self._ns.perform_full_preset()

        # Set cell off
        self._ns_cell_2g.set_cell_off()

        # Set Frequency points using frequency list
        # Set Amplitude Offset correction using offset list
        # Turning amplitude offset state to ON
        self._ns.configure_amplitude_offset_table()

        # Set Cell Band  using CELL_BAND parameter
        self._ns_cell_2g.set_band(self._band_name)

        # Set Broadcast Channel Arfcn using BCH_ARFCN parameter
        self._ns_cell_2g.set_bcch_arfcn(self._bch_arfcn)

        # Set Traffic Channel Arfcn using TCH_ARFCN parameter
        self._ns_cell_2g.set_tch_arfcn(self._tch_arfcn)

        # Set Mobile Country Code (MCC)
        self._ns_cell_2g.set_mcc(self._mcc)

        # Set Mobile Network Code (MNC)
        self._ns_cell_2g.set_mnc(self._mnc)

        # Set cell service using CELL_SERVICE parameter
        self._ns_cell_2g.set_cell_service(self._cell_service)

        # Set Cell Power using CELL_POWER parameter
        self._ns_cell_2g.set_cell_power(self._cell_power)

        # Set VOICE_CODER_RATE
        self._ns_voice_call_2g.set_audio_codec(self._voice_coder_rate)

        # Set DTM OFF
        self._logger.info("Setting DTM OFF")
        self._ns_cell_2g.set_dtm_state("OFF")

        # Set cell on
        self._ns_cell_2g.set_cell_on()

        # Disable airplane mode
        self._networking_api.set_flight_mode("off")

        # Adapt attachment procedure to CIRCUIT or PACKET
        if "GSM" not in self._cell_service:
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
                                                                   self._ns_cell_2g,
                                                                   self._networking_api,
                                                                   self._logger,
                                                                   self._registration_timeout,
                                                                   flightmode_cycle=True,
                                                                   blocking=False)

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

        # Get RAT from Equipment
        network_type = self._ns_data_2g.get_network_type()

        # Check that DUT is registered on the good RAT
        self._modem_api.check_network_type_before_timeout(network_type,
                                                          self._registration_timeout)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """

        # Call power measurement base tear_down function
        UseCaseBase.tear_down(self)

        try:
            # Activate the flight mode.
            self._networking_api.set_flight_mode("on")
        except:
            pass

        # Set cell off
        self._ns_cell_2g.set_cell_off()

        # DisConnect from equipment
        self._ns.release()

        return Global.SUCCESS, "No errors"
