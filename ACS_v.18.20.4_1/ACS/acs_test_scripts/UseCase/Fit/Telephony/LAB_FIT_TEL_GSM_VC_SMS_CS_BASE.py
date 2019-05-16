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
:summary: Use Case GSM Voice Call and SMS for FIT testing.
:since: 20/03/2012
:author: lvacheyx
"""
import time

from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from acs_test_scripts.Utilities.SmsUtilities import DataCodingScheme
from acs_test_scripts.Utilities.EquipmentUtilities import get_nw_sim_bench_name
from ErrorHandling.AcsConfigException import AcsConfigException
import acs_test_scripts.Utilities.RegistrationUtilities as RegUtil


class LabFitTelGsmVcSmsCsBase(UseCaseBase):

    """
    Lab Gsm FIT (Voice call + SMS) base class.
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
        self.ns_cell_tech = "2G"

        # Retrieve valid bench name for 2G capability
        self._bench_name = get_nw_sim_bench_name(self.ns_cell_tech, global_config, self._logger)
        # bench_name should be either NETWORK_SIMULATOR1 or NETWORK_SIMULATOR2
        # In both cases, we need to retrieve the NS number (position in the bench config, either 1 or 2)
        self._ns_number = int(self._bench_name[-1])

        # Read NETWORK_SIMULATOR from BenchConfig.xml
        self._ns_node = \
            global_config.benchConfig.get_parameters(self._bench_name)

        self._band_name = self._tc_parameters.get_param_value("CELL_BAND")

        # Read BCH_ARFCN from test case xml file
        self._bch_arfcn = int(self._tc_parameters.get_param_value("BCH_ARFCN"))

        # Read TCH_ARFCN from test case xml file
        self._tch_arfcn = int(self._tc_parameters.get_param_value("TCH_ARFCN"))

        # Set CELL SERVICE to GSM are we are in CS mode
        self._cell_service = "GSM"

        # Set CELL POWER to -60 dBm
        self._cell_power = int(-60)

        # set VOICE CODER RATE to FR_AMR_NB_1220
        self._voice_coder_rate = "FR_AMR_NB_1220"

        # Read SMS_TEXT from xml UseCase parameter file
        self._sms_text = self._tc_parameters.get_param_value("SMS_TEXT")

        # Set data coding sheme
        self._data_coding_sheme = str(00)

        # Read SMS_TRANSFER_TIMEOUT from xml UseCase parameter file
        self._sms_transfer_timeout = \
            int(self._tc_parameters.get_param_value("SMS_TRANSFER_TIMEOUT"))

        dcs = DataCodingScheme(self._data_coding_sheme)
        dcs.decode()

        # Create cellular network simulator and retrieve 2G voice call interface
        self._ns = self._em.get_cellular_network_simulator(self._bench_name)

        # Shortcut to get Cell 2G parameters
        self._ns_cell_2g = self._ns.get_cell_2g()
        self._ns_voice_call_2g = self._ns_cell_2g.get_voice_call()
        self._ns_data_2g = self._ns_cell_2g.get_data()

        # retrieve 2G messaging interface
        self._messaging_2g = self._ns_cell_2g.get_messaging()

        # Get number of bits per character setted in DCS
        self._nb_bits_per_char = dcs.compute_character_size()

        character_set = dcs.get_character_set()

        if character_set == "7BITS":
            self._content_type = "CTEX"
        else:
            self._content_type = "CDAT"

        # Instantiate Messaging UECmd for SMS
        self._messaging_api = self._device.get_uecmd("SmsMessaging")

        # Instantiate Modem UECmd for checking phone registration
        self._modem_api = self._device.get_uecmd("Modem")

        # Instantiate generic UECmd for voiceCall
        self._voicecall_api = self._device.get_uecmd("VoiceCall")

        # Instantiate generic UECmd for camp
        self._networking_api = self._device.get_uecmd("Networking")

        # NS_CELL_REL
        self._ns_cell_rel = 7

        self._ns_model = self._ns_node.get_param_value("Model")

# ------------------------------------------------------------------------------
    def set_up(self):
        """
        Set up the test configuration
        """

        # Call use case base Setup function
        UseCaseBase.set_up(self)

        # Ensure flight mode off so that GSM sim operator info can be retrieved
        self._networking_api.set_flight_mode("on")
        time.sleep(self._wait_btwn_cmd)

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

        # Call specific configuration functions
        # Some parameters are defined by function
        RegUtil.setup_cell(self._ns_number,
                           self._ns_model,
                           self.ns_cell_tech,
                           self._band_name,
                           self._ns_cell_rel,
                           self._logger)

        # Set cell service using CELL_SERVICE parameter
        self._ns_cell_2g.set_cell_service(self._cell_service)

        # Set Cell Power using CELL_POWER parameter
        self._ns_cell_2g.set_cell_power(self._cell_power)

        # Set VOICE_CODER_RATE
        self._ns_voice_call_2g.set_audio_codec(self._voice_coder_rate)

        # Set cell on
        self._ns_cell_2g.set_cell_on()

        # Ensure flight mode off so that GSM sim operator info can be retrieved
        self._networking_api.set_flight_mode("off")
        time.sleep(self._wait_btwn_cmd)

        # Set the APN
        time.sleep(self._wait_btwn_cmd)
        self._logger.info("Setting APN " + str(self._apn) + "...")
        self._networking_api.set_apn(self._ssid, self._apn)

        # Activate PDP context
        time.sleep(self._wait_btwn_cmd)
        self._logger.info("Active PDP Context...")
        self._networking_api.activate_pdp_context(self._ssid, check=False)

        # Check registration state is connected using
        # registrationTimeout from Device_Catalog.xml
        self._modem_api.check_cdk_registration_bfor_timeout(
            self._registration_timeout)

        return Global.SUCCESS, "No errors"
# ------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """

        # Call use case base Setup function
        UseCaseBase.run_test(self)

        # Clear old SMS (Non blocking for this test if function isn't
        # implemented on CDK)
        time.sleep(self._wait_btwn_cmd)
        self._messaging_api.delete_all_sms()

        # Clear old SMS on 8960
        self._messaging_2g.clear_message_data()

        # Release any previous call (Robustness)
        self._voicecall_api.release()

        # Check that operator gave a valid Voice call number
        # Perform MO voice call on active network simulator
        if self._is_phone_number_checked:
            if self._phone_number is None:
                self._error.Msg = "Operator Phone Number cannot be used to perform a voice call" \
                                  "due to invalid test parameter value (Phone Number %s)" \
                    % (str(self._tc_parameters.get_param_value("PHONE_NUMBER")))
                self._logger.warning(self._error.Msg)
                raise AcsConfigException(AcsConfigException.PROHIBITIVE_BEHAVIOR, self._error.Msg)

            else:
                self._logger.info("Perform MO voice call to : " + self._phone_number)
                self._voicecall_api.dial(self._phone_number)
        else:
            # Raise an error message as no valid phone number has been set by the operator
            self._error.Msg = "Phone number has no valid value (%s) so voice call can not be performed" % \
                              self._phone_number
            self._logger.error(self._error.Msg)
            raise AcsConfigException(AcsConfigException.PROHIBITIVE_BEHAVIOR, self._error.Msg)

        # Check call state "CONNECTED" before callSetupTimeout seconds
        self._ns_voice_call_2g.check_call_connected(self._call_setup_time)

        # Wait for state "active" before callSetupTimeout seconds
        self._voicecall_api.wait_for_state(
            self._uecmd_types.VOICE_CALL_STATE.ACTIVE,  # pylint: disable=E1101
            self._call_setup_time)

        return Global.SUCCESS, "No errors"
# ------------------------------------------------------------------------------

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

        # Clear old SMS (Non blocking for this test if function isn't
        # implemented on CDK)
        time.sleep(self._wait_btwn_cmd)
        self._messaging_api.delete_all_sms()

        return Global.SUCCESS, "No errors"
