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
:summary: Use Case WCDMA Voice Call and SMS for FIT testing.
:since: 08/10/2012
:author: lvacheyx
"""
import time
import acs_test_scripts.Utilities.RegistrationUtilities as RegUtil

from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from acs_test_scripts.Utilities.SmsUtilities import DataCodingScheme
from acs_test_scripts.Utilities.EquipmentUtilities import get_nw_sim_bench_name
from ErrorHandling.AcsConfigException import AcsConfigException


class LabFitTelWcdmaVcSmsCsBase(UseCaseBase):

    """
    Lab Wcdma FIT (Voice call + SMS) base class.
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

        # Retrieve valid bench name for 3G capability
        self._bench_name = get_nw_sim_bench_name("3G", global_config, self._logger)

        # Read NETWORK_SIMULATOR from BenchConfig.xml
        self._ns_node = \
            global_config.benchConfig.get_parameters(self._bench_name)

        # Read CELL_BAND from test case xml file
        self._band = int(self._tc_parameters.get_param_value("CELL_BAND"))

        # Read DL_UARFCN from test case xml file
        self._dl_uarfcn = int(self._tc_parameters.get_param_value("DL_UARFCN"))

        # Read CELL_SERVICE from test case xml file
        self._cell_service = self._tc_parameters.get_param_value("CELL_SERVICE", "CIRCUIT")

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

        # Create cellular network simulator and retrieve 3G voice call interface
        self._ns = self._em.get_cellular_network_simulator(self._bench_name)

        # Shortcut to get Cell 3G parameters
        self._ns_cell_3g = self._ns.get_cell_3g()
        self._ns_voice_call_3g = self._ns_cell_3g.get_voice_call()
        self._ns_data_3g = self._ns_cell_3g.get_data()

        # retrieve 3g messaging interface
        self._messaging_3g = self._ns_cell_3g.get_messaging()

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

        # Set the equipment Application Format = "WCDMA"
        self._ns.switch_app_format("WCDMA")

        # Perform a full preset
        self._ns.perform_full_preset()

        # Set cell off
        self._ns_cell_3g.set_cell_off()

        # Set Frequency points using frequency list
        # Set Amplitude Offset correction using offset list
        # Turning amplitude offset state to ON
        self._ns.configure_amplitude_offset_table()

        # "Set Cell Service in function of the CELL_SERVICE value :
        # ""CIRCUIT"" : paging service is AMR
        #                PS domain is ABSENT
        # ""PACKET"" : paging service is GPRS
        #                PS domain is PRESENT
        # ""CIRCUIT_PACKET"": paging service is AMR
        #                PS domain is PRESENT
        # ""RBTEST"" : paging service is RBT"
        self._ns_cell_3g.set_cell_service(self._cell_service)

        # Set Cell Band UARFCN (downlink) using Band and DlUarfcn
        self._ns_cell_3g.set_band_and_dl_arfcn(
            "BAND" + str(self._band), self._dl_uarfcn)

        # Set Cell Band UARFCN (uplink) in auto mode
        self._ns_cell_3g.set_uplink_channel_mode("ON")

        # Set Audio codec using VOICE_CODER_RATE parameter
        self._ns_voice_call_3g.set_audio_codec(self._voice_coder_rate)

        # Set SRB  Configuration Control to auto
        self._ns_cell_3g.set_srb_config_control("ON")

        # Set Cell Power using CELL_POWER parameter
        self._ns_cell_3g.set_cell_power(self._cell_power)

        # Set Cell on
        self._ns_cell_3g.set_cell_on()

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

        if self._cell_service not in "CIRCUIT":
            # Adapt attachment procedure to CIRCUIT
            dut_imsi = self._modem_api.get_imsi(self._registration_timeout)

            RegUtil.check_dut_registration_before_timeout(self._ns_cell_3g,
                                                          self._networking_api,
                                                          self._logger,
                                                          dut_imsi,
                                                          self._registration_timeout)
            # Get RAT from Equipment
            network_type = self._ns_data_3g.get_network_type()

            # Check that DUT is registered on the good RAT
            self._modem_api.check_network_type_before_timeout(network_type,
                                                              self._registration_timeout)

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
        self._messaging_3g.clear_message_data()

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
        self._ns_voice_call_3g.check_call_connected(self._call_setup_time)

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
        self._ns_cell_3g.set_cell_off()

        # DisConnect from equipment
        self._ns.release()

        # Clear old SMS (Non blocking for this test if function isn't
        # implemented on CDK)
        time.sleep(self._wait_btwn_cmd)
        self._messaging_api.delete_all_sms()

        return Global.SUCCESS, "No errors"
