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
:summary: TEL - this UseCase Base prepare testing all Uecmd used by all TEL UseCases
:author: lvacheyx
:since: 19/03/2013
"""

from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from acs_test_scripts.Utilities.CommunicationUtilities import ConfigsParser
from acs_test_scripts.Utilities.SmsUtilities import DataCodingScheme
from UtilitiesFWK.Utilities import Global

import acs_test_scripts.Utilities.RegistrationUtilities as RegUtil
import time


class LabTelDebugBase(UseCaseBase):

    """
    Lab Telephony Debug Base class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call UseCase base Init function
        UseCaseBase.__init__(self, tc_name, global_config)

        self._modem_api = self._device.get_uecmd("Modem")
        self._networking_api = self._device.get_uecmd("Networking")
        self._messaging_api = self._device.get_uecmd("SmsMessaging")
        self._voicecall_api = self._device.get_uecmd("VoiceCall")

        # Read registrationTimeout from Device_Catalog.xml
        self._registration_timeout = \
            int(self._dut_config.get("registrationTimeout"))

        # Read CELLULAR_NETWORK parameters from BenchConfig.xml
        self._network = \
            global_config.benchConfig.get_parameters("CELLULAR_NETWORK")
        self._apn = self._network.get_param_value("APN")
        self._ssid = self._network.get_param_value("SSID")

        # Read NETWORK_SIMULATOR1 from BenchConfig.xml
        self._ns_node = \
            global_config.benchConfig.get_parameters("NETWORK_SIMULATOR1")

        # Set an artificial BAND Value
        self._band_name = 1

        # Set an artificial Cell Service Value
        self._cell_service = "CIRCUIT"

        # Set an artificial CELL_POWER Value
        self._cell_power = -60

        # Set an artificial DL_UARFCN Value
        self._dl_uarfcn = 10700

        # Set an artificial Data Coding Scheme Value
        self._data_coding_sheme = "0"

        # Set an artificial SMS text Value
        self._sms_text = "TEST OF THE MESSAGING UEcmds"

        # Set an artificial SMS TRANSFER TIMEOUT Value
        self._sms_transfer_timeout = 30

        # Read UECMD_TYPE_LIST from TC parameters
        self.__uecmd_type = self._tc_parameters.get_param_value("UECMD_TYPE_LIST").upper()

        # Set an artificial voice coder rate Value
        self._voice_coder_rate = "FR_AMR_NB_1220"

        # Set an artificial phone number Value
        self._destination_number = "0620202020"

        dcs = DataCodingScheme(self._data_coding_sheme)
        dcs.decode()

        # Get number of bits per character set in DCS
        self._nb_bits_per_char = dcs.compute_character_size()

        character_set = dcs.get_character_set()
        if character_set == "7BITS":
            self._content_type = "CTEX"
        else:
            self._content_type = "CDAT"

        # Create cellular network simulator and retrieve 3G voice call and data interfaces
        self._ns = self._em.get_cellular_network_simulator("NETWORK_SIMULATOR1")
        self._ns_cell_3g = self._ns.get_cell_3g()
        self._ns_voice_call_3g = self._ns_cell_3g.get_voice_call()
        self._ns_data_3g = self._ns_cell_3g.get_data()
        self._ns_messaging_3g = self._ns_cell_3g.get_messaging()

# ------------------------------------------------------------------------------
    def set_up(self):
        """
        Set up the test configuration
        """
        # Call use case base Setup function
        UseCaseBase.set_up(self)

        # Connect to equipment
        self._ns.init()

        # Set the equipment Application Format = "WCDMA"
        self._ns.switch_app_format("WCDMA")

        # Perform full preset
        self._ns.perform_full_preset()

        # Set cell off
        self._ns_cell_3g.set_cell_off()

        # Deactivate HSUPA and HSDPA capabilities
        self._ns_data_3g.set_edch_cell_capability("OFF")
        self._ns_data_3g.set_hsdpa_cell_capability("OFF")

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

        # Set Cell Band UARFCN (uplink) in auto mode
        self._ns_cell_3g.set_uplink_channel_mode("ON")

        # Set Audio codec using VOICE_CODER_RATE parameter
        self._ns_voice_call_3g.set_audio_codec(self._voice_coder_rate)

        # Set cell power using CELL_POWER value
        self._ns_cell_3g.set_cell_power(self._cell_power)

        # Set downlink UARFCN using DL_UARFCN value
        self._ns_cell_3g.set_band_and_dl_arfcn(
            "BAND" + str(self._band_name), self._dl_uarfcn)

        # Set SRB  Configuration Control to auto
        self._ns_cell_3g.set_srb_config_control("ON")

        # Set cell on
        self._ns_cell_3g.set_cell_on()

        # Check registration status before time out using
        # registrationTimeout value from Device_Catalog.xml
        time.sleep(self._wait_btwn_cmd)
        dut_imsi = self._modem_api.get_imsi(self._registration_timeout)

        RegUtil.check_dut_registration_before_timeout(self._ns_cell_3g,
                                                      self._networking_api,
                                                      self._logger,
                                                      dut_imsi,
                                                      self._registration_timeout)

        # Check registration state is connected using
        # registrationTimeout from Device_Catalog.xml (Non blocking
        # for this test if function isn't implemented on CDK)
        time.sleep(self._wait_btwn_cmd)
        self._modem_api.check_cdk_registration_bfor_timeout(
            self._registration_timeout)

        # Set the preferred connection type to CS Domain ("CSD")
        self._ns_messaging_3g.select_sms_transportation("CSD")

        # Set the Data coding scheme using DATA_CODING_SCHEME value
        self._ns_messaging_3g.set_sms_data_coding_scheme(
            int(self._data_coding_sheme, 16))

        # Set sender address using DESTINATION_NUMBER on Network simulator
        self._ns_messaging_3g.set_sms_sender_address(self._destination_number)

        return Global.SUCCESS, "No errors"
