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
:summary: Use Case GSM Voice Call for Ringtone tests.
:since: 01/08/2011
:author: shuang
"""

import time
from acs_test_scripts.Utilities.CommunicationUtilities import ConfigsParser
import acs_test_scripts.Utilities.RegistrationUtilities as RegUtil

from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.Multimedia.LIVE_MUM_BASE import LiveMuMBase
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from acs_test_scripts.Utilities.EquipmentUtilities import get_nw_sim_bench_name


class LabGsmVcRingtoneMtmr(LiveMuMBase):

    """
    Lab Gsm Voice Call Ringtone test class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        # Call LiveMuMBase base Init function
        LiveMuMBase.__init__(self, tc_name, global_config)

        # Read registrationTimeout from Device_Catalog.xml
        self._registration_timeout = \
            int(self._dut_config.get("registrationTimeout"))

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

        # Read RT_TIME from test case xml file
        self._rt_time = int(self._tc_parameters.get_param_value("RT_TIME"))

        # Read RT_NAME from test case xml file
        self._rt_name = self._tc_parameters.get_param_value("RT_NAME")

        # Read AUDIO_OUTPUT from test case xml file
        self._audio_output = str(self._tc_parameters.get_param_value("AUDIO_OUTPUT")).lower()

        # Read VIBRA_MODE from test case xml file
        self._vibra_mode = str(self._tc_parameters.get_param_value("VIBRA_MODE")).lower()

        # Read SILENT_MODE from test case xml file
        self._silent_enable = str(self._tc_parameters.get_param_value("SILENT_MODE")).lower()

        # store original silent mode and vibrate mode and audio output
        self._original_silent_vibrate = None
        self._origianl_audio_output = None

        # Create cellular network simulator and retrieve 2G voice call interface
        self._ns = self._em.get_cellular_network_simulator(self._bench_name)
        self._ns_cell_2g = self._ns.get_cell_2g()
        self._ns_voice_call_2g = self._ns_cell_2g.get_voice_call()
        self._ns_data_2g = self._ns_cell_2g.get_data()

        # Instantiate generic UECmd for voiceCall Ucs
        self._modem_api = self._device.get_uecmd("Modem")
        self._voicecall_api = self._device.get_uecmd("VoiceCall")
        self._audio_recorder_api = self._device.get_uecmd("AudioRecorderHost")
        self._phonesystem_api = self._device.get_uecmd("PhoneSystem")
        self._networking_api = self._device.get_uecmd("Networking")

# ------------------------------------------------------------------------------
    def set_up(self):
        """
        Set up the test configuration
        """

        # Call use case base Setup function
        UseCaseBase.set_up(self)

        # get original audio output
        self._origianl_audio_output = \
            self._phonesystem_api.get_audio_output()

        # get original silent mode and vibrate mode
        self._original_silent_vibrate = \
            self._phonesystem_api.get_silent_vibrate()

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

        # Set cell service using CELL_SERVICE parameter
        self._ns_cell_2g.set_cell_service(self._cell_service)

        # Set Cell Power using CELL_POWER parameter
        self._ns_cell_2g.set_cell_power(self._cell_power)

        # Set VOICE_CODER_RATE
        self._ns_voice_call_2g.set_audio_codec(self._voice_coder_rate)

        # Set ringtone time using RT_TIME parameter
        self._ns_voice_call_2g.set_mt_originate_call_timeout(self._rt_time)

        # Set cell on
        self._ns_cell_2g.set_cell_on()

        # Adapt attachment procedure to CIRCUIT or PACKET
        if self._cell_service == "GSM":
            # Check phone registration on equipment before given CAMP_TIMEOUT
            # (in loop compare every second the IMSI returned by the simulator
            # and IMSI read on CDK until comparison is true or timeout expired)
            dut_imsi = self._modem_api.get_imsi(self._registration_timeout)

            RegUtil.check_dut_registration_before_timeout(self._ns_cell_2g,
                                                          self._networking_api,
                                                          self._logger,
                                                          dut_imsi,
                                                          self._registration_timeout)
        else:
            # Check Data Connection State => ATTACHED before timeout
            RegUtil.check_dut_data_connection_state_before_timeout("ATTACHED",
                                                                   self._ns_cell_2g,
                                                                   self._networking_api,
                                                                   self._logger,
                                                                   self._registration_timeout,
                                                                   flightmode_cycle=True,
                                                                   blocking=False)

        # Check registration state is connected using
        # registrationTimeout from Device_Catalog.xml
        self._modem_api.check_cdk_registration_bfor_timeout(self._registration_timeout)

        # Get RAT from Equipment
        network_type = self._ns_data_2g.get_network_type()

        # Check that DUT is registered on the good RAT
        self._modem_api.check_network_type_before_timeout(network_type,
                                                          self._registration_timeout)

        return Global.SUCCESS, "No errors"
#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        LiveMuMBase.run_test(self)
        # Release any previous call (Robustness)
        self._voicecall_api.release()

        # set audio output
        if self._audio_output not in (None, "none"):
            self._phonesystem_api.switch_audio_output(self._audio_output)

        # set silent mode and vibrate mode
        if (self._silent_enable not in (None, "none")) and \
                (self._vibra_mode not in (None, "none")):
            self._phonesystem_api.set_silent_vibrate(self._silent_enable,
                                                     self._vibra_mode)

        # Set Ringtone
        self._voicecall_api.set_ringtone(self._rt_name)

        time.sleep(self._wait_btwn_cmd)
        self._logger.info("Start recording on pc...")
        # Generate the audio record name for each playback
        self._host_record_file = self.generate_audio_record_name()
        self._audio_recorder_api.pc_audio_record_start(self._host_record_file)

        # Mobile Terminated originate call
        self._ns_voice_call_2g.mt_originate_call()

        # pylint: disable=E1101
        # Wait for state "incoming" before callSetupTimeout seconds
        self._voicecall_api.wait_for_state(
            self._uecmd_types.VOICE_CALL_STATE.INCOMING,
            self._call_setup_time)

        time.sleep(int(self._rt_time))
        self._logger.info("Stop recording on pc...")
        self._audio_recorder_api.pc_audio_record_stop()

        # Mobile Release call
        self._voicecall_api.release()

        # Check voice call state is "IDLE" (8960)
        # self._ns_voice_call_2g.check_call_state("IDLE", self._call_setup_time)
        # Check call is released (NS)
        self._ns_voice_call_2g.check_call_idle(self._registration_timeout,
                                               blocking=False)

        # Check voice call state is "no_call" (CDK)
        self._voicecall_api.wait_for_state(
            self._uecmd_types.VOICE_CALL_STATE.NOCALL,
            self._call_setup_time)

        # pylint: enable=E1101
        result_msg = "ringtone record saved at " + self._host_record_file
        return Global.SUCCESS, result_msg

#------------------------------------------------------------------------------
    def tear_down(self):
        """
        End and dispose the test
        """

        # Call power measurement base tear_down function
        UseCaseBase.tear_down(self)

        # Call pc_audio_record_stop in tear_down, to make sure UC release system resource
        time.sleep(float(self._wait_btwn_cmd))
        self._logger.info("Stop recording on pc...")
        self._audio_recorder_api.pc_audio_record_stop()

        # Set cell off
        self._ns_cell_2g.set_cell_off()

        # DisConnect from equipment
        self._ns.release()

        # set the original silent mode and vibrate mode and audio output
        if (self._silent_enable not in (None, "none")) and \
                (self._vibra_mode not in (None, "none")):
            self._phonesystem_api.set_silent_vibrate(self._original_silent_vibrate.split('+')[0],
                                                     self._original_silent_vibrate.split('+')[1])
        if self._audio_output not in (None, "none"):
            self._phonesystem_api.switch_audio_output(self._origianl_audio_output)

        return Global.SUCCESS, "No errors"
