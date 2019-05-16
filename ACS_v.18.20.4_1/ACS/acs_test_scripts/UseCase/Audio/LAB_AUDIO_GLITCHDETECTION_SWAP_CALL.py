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

property right is granted to or conferred upon you by disclosure or delivery
of the Materials, either expressly, by implication, inducement, estoppel or
otherwise. Any license under such intellectual property rights must be express
and approved by Intel in writing.

:organization: INTEL MCG PSI
:summary: This file implements the AudioComms Swap calls (Check audio
quality).
:since: 02/07/2013
:author: nprecigx
"""

import time
from UtilitiesFWK.Utilities import Global, str_to_bool
from Device.DeviceManager import DeviceManager
from ErrorHandling.TestEquipmentException import TestEquipmentException
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from acs_test_scripts.Device.UECmd.UECmdTypes import PreferredNetwork
# pylint: disable=E1101


class LabAudioGlitchDetectionSwapCall(UseCaseBase):
    """
    AudioComms Audio Swap Call class.
    """
    def __compute_test_verdict__(self, audio_analyzer_result):

        if audio_analyzer_result == 0 and self._audio_analyzer_result_save == 0:
            self._result_verdict = Global.SUCCESS
        elif audio_analyzer_result == 1:
            # Update audio_analyser_result => error code 1 lower priority than error code 2
            if self._audio_analyzer_result_save == 0 or self._audio_analyzer_result_save == 3:
                self._audio_analyzer_result_save = 1
            # Save failed transition
            self._failed_transition_audio_pb = self._failed_transition_audio_pb + ", " + \
                self._previous_call_type_save + " => " + self._call_type_save
            self._result_verdict = Global.FAILURE
        elif audio_analyzer_result == 2:
            self._audio_analyzer_result_save = 2
            self._failed_transition_no_audio = self._failed_transition_no_audio + ", " + \
                self._previous_call_type_save + " => " + self._call_type_save
            self._result_verdict = Global.FAILURE
        elif audio_analyzer_result == 3:
            self._audio_analyzer_result_save = 3
            self._result_verdict = Global.FAILURE
            self._failed_transition_no_audio = self._failed_transition_no_audio + ", " + \
                self._previous_call_type_save + " => " + self._call_type_save
        else:
            if self._audio_analyzer_result_save == 0:
                self._audio_analyzer_result_save = 4
            self._result_verdict = Global.FAILURE

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        # Call UseCaseBase Init function
        UseCaseBase.__init__(self, tc_name, global_config)

        # Create Audio Analyser
        self._audio_analyzer = self._em.get_audio_analyzer("AudioAnalyzer")

        # Check if IO card is used
        self._use_io_card = str_to_bool(global_config.campaignConfig.get("isIoCardUsed"))

        # Create Whs
        if self._use_io_card:
            self._wired_headset = self._em.get_wired_headset("WiredHeadset")

        # Initialization of the test type
        self._test_type = "glitchdetectiononswap"

        # Initialization of test result
        self._result_verdict = Global.FAILURE

        # Initialization of the verdict comment
        self._verdict_comment = "Audio Quality test fail"

        # Initialization of _dut_bt_address
        self._dut_bt_address = "None"

        # Initialization of audio_analyser_result saved
        self._audio_analyzer_result_save = 0
        self._audio_analyzer_result_ul = 0
        self._audio_analyzer_result_dl = 0

        # delta volume for voip
        self._delta_voip_volume = 10

        # Initialization of failed transition saved
        self._failed_transition_audio_pb = ""
        self._failed_transition_no_audio = ""
        self._previous_call_type_save = ""
        self._call_type_save = ""

        self._phone_calling_list = []
        self._phone_receiving_list = []
        self._phone_releasing_list = []
        self._ui_api_phone_list = []
        self._calling_phone_number = []
        # List Saving
        self._saved_call_list = []
        self._saved_call_end_type_list = []
        self._saved_call_origin_type_list = []
        self._call_type_list_audio_analyser_run = []

        # Instantiate the instances for phone caller, receiver and releaser
        self._phone_calling = None
        self._phone_receiving = None

        # Read registrationTimeout from Device_Catalog.xml
        self._registration_timeout = \
            int(self._dut_config.get("registrationTimeout"))

        # Read callSetupTimeout from Device_Catalog.xml
        self._call_setup_time = \
            int(self._dut_config.get("callSetupTimeout"))

        # Read wifi parameters from bench config xml file (int)
        self._wifirouter = global_config.benchConfig.\
            get_parameters("WIFI_ACCESS_POINT")
        self._ssid = self._wifirouter.get_param_value("SSID")

        # Read accessories type from test case xml file (str)
        self._acc_type = \
            str(self._tc_parameters.get_param_value("ACC_TYPE"))

        # Read call type list from test case xml file (str)
        self._call_seq = str(self._tc_parameters.get_param_value("CALL_SEQUENCE"))
        # Split the call type list, call separated by ','
        self._call_seq = self._call_seq.strip('[] ')
        self._call_list = self._call_seq.split(',')
        self._call_list.reverse()

        # Read call origin list from test case xml file (str)
        self._call_origin_type = \
            str(self._tc_parameters.get_param_value("CALL_ORIGIN_TYPE_SEQUENCE"))
        # Split the call origin type list, call origin type separated by ','
        self._call_origin_type = self._call_origin_type.strip('[] ')
        self._call_origin_type_list = self._call_origin_type.split(',')
        self._call_origin_type_list.reverse()

        # Read call end type list from test case xml file (str)
        self._call_end_type = \
            str(self._tc_parameters.get_param_value("CALL_END_TYPE_SEQUENCE"))
        # Split the call end type list, call end type separated by ','
        self._call_end_type = self._call_end_type.strip('[] ')
        self._call_end_type_list = self._call_end_type.split(',')
        self._call_end_type_list.reverse()

        # Read call type from test case xml file (str)
        self._call_type = str(self._tc_parameters.get_param_value("CALL_TYPE"))

        # Call Stream Volume in percent
        self._call_stream_volume_dut = \
            int(self._tc_parameters.get_param_value("CALL_VOLUME_DUT"))

        # Call Stream Volume in percent
        self._call_stream_volume_ref = \
            int(self._tc_parameters.get_param_value("CALL_VOLUME_REF"))

        # Read duration type from test case xml file (int)
        if self._tc_parameters.get_param_value("DURATION"):
            self._call_duration = int(self._tc_parameters.get_param_value("DURATION"))
        else:
            # Call duration by default
            self._call_duration = 5

        # Read number of swap from test case xml file (int)
        self._number_of_swap = int(self._tc_parameters.get_param_value("SWAP_REPETITION"))

        # Read dut sip address from test case xml file (str)
        self._dut_sip_address = \
            str(self._tc_parameters.get_param_value("DUT_SIP_ADDRESS"))

        # Read ref phone sip address from test case xml file (str)
        self._peer_sip_address = \
            str(self._tc_parameters.get_param_value("PEER_SIP_ADDRESS"))

        # Read dut sip profile name from test case xml file (str)
        self._dut_sip_profile = \
            str(self._tc_parameters.get_param_value("DUT_SIP_PROFILE"))

        # Read ref phone sip profile name from test case xml file (str)
        self._peer_sip_profile = \
            str(self._tc_parameters.get_param_value("PEER_SIP_PROFILE"))

        # Read test call direction type from test case xml file (str)
        self._signal_tested_direction = \
            str(self._tc_parameters.get_param_value("SIGNAL_TESTED_DIRECTION"))

        # Read Audio Analyzer parameters from bench config xml file (str)
        self._audio_analyser = global_config.benchConfig.\
            get_parameters("AudioAnalyzer")
        self._bt_remote_addr = self._audio_analyser.get_param_value("Bt_mac_addr")

        # Instantiate generic UECmd for voiceCall Ucs
        self._voice_call_api = self._device.get_uecmd("VoiceCall")
        self._phonesystem_api = self._device.get_uecmd("PhoneSystem")
        self._sip_call_api = self._device.get_uecmd("SipCall")
        self._phonemodem_api = self._device.get_uecmd("Modem")
        self._system_api = self._device.get_uecmd("System")
        self._networking_api = self._device.get_uecmd("Networking")
        self._ui_api = self._device.get_uecmd("Ui")
        self._ui_api.set_global_config(global_config)

        # Read Audio Analyzer parameters from bench config xml file (str)
        self._audio_analyser = global_config.benchConfig.get_parameters("AudioAnalyzer")

        if self._acc_type.find("BLUETOOTH") != -1:
            # Get UECmdLayer
            self._bt_api = self._device.get_uecmd("LocalConnectivity")
            self._bt_remote_addr = self._audio_analyser.get_param_value("Bt_mac_addr")

        # Load instance of the PHONE2
        self._phone2 = DeviceManager().get_device("PHONE2")
        # Instantiate system UECmd for Phone2
        self._system_api2 = None

        # Initialize UI object
        self._ui_api.init()

        # Connect the board
        self._device.connect_board()

        if self._phone2 is not None:
            if not self._phone2.is_available():
                self._phone2.switch_on(simple_switch_mode=True)
            self._voice_call_api2 = self._phone2.get_uecmd("VoiceCall")
            self._sip_call_api2 = self._phone2.get_uecmd("SipCall")
            self._ui_api2 = self._phone2.get_uecmd("Ui")
            self._ui_api2.set_global_config(global_config)
            self._system_api2 = self._phone2.get_uecmd("System")
            self._networking_api2 = self._phone2.get_uecmd("Networking")
            self._phonesystem_api2 = self._phone2.get_uecmd("PhoneSystem")
            self._ui_api2.init()
        else:
            error_msg = \
                "This test case requires two phones to be executed !"
            self._logger.error(error_msg)
            raise AcsConfigException(AcsConfigException.INVALID_BENCH_CONFIG, error_msg)

    def set_up(self):
        """
        Set up the test configuration
        """
        # Call UseCaseBase Setup function
        UseCaseBase.set_up(self)

        # Check if we have the second phone available
        if self._phone2 is None:
            # We are using this multi UC with only one phone
            error_msg = \
                "This test case requires two phones to be executed !"
            self._logger.error(error_msg)
            raise AcsConfigException(AcsConfigException.INVALID_BENCH_CONFIG, error_msg)

        self._phonesystem_api.wake_screen()
        # Swap 2G 3G or 4G.
        if self._call_type == "2G":
            self._networking_api.set_preferred_network_type(PreferredNetwork.GSM_ONLY)
        elif self._call_type == "3G":
            self._networking_api.set_preferred_network_type(PreferredNetwork.WCDMA_ONLY)
        elif self._call_type == "4G":
            self._networking_api.set_preferred_network_type(PreferredNetwork.LTE_ONLY)
        else:
            if self._call_type not in "VOIP":
                msg = "wrong value of parameter CALL TYPE"
                self._logger.error(msg)
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        # List Saving
        self._saved_call_list = list(self._call_list)
        self._saved_call_end_type_list = list(self._call_end_type_list)
        self._saved_call_origin_type_list = list(self._call_origin_type_list)

        # make list for call test type for audio_analyser
        l_temp_call_list = list(self._call_list)
        l_temp_call_list.reverse()
        self._call_type_list_audio_analyser_run = list(l_temp_call_list)
        num_of_swap_unpair = self._number_of_swap % 2
        i = self._number_of_swap
        while i > 1:
            self._call_type_list_audio_analyser_run += list(l_temp_call_list)
            i -= 1
        if num_of_swap_unpair > 0:
            # Grep last element of call list and insert at the top
            self._call_type_list_audio_analyser_run.insert(0, l_temp_call_list.pop())
        i = 1
        # Initialization of list for calling, receiving and releasing calls
        while len(self._saved_call_list) > 0:
            call_type = self._saved_call_list.pop()
            call_origin = self._saved_call_origin_type_list.pop()
            call_end = self._saved_call_end_type_list.pop()
            if call_origin == "MO":
                self._phone_calling_list.insert(i, self._voice_call_api)
                self._phone_receiving_list.insert(i, self._voice_call_api2)
                self._ui_api_phone_list.insert(i, self._ui_api2)
                if call_type == "2G" or call_type == "3G":
                    self._calling_phone_number.insert(i, self._phone2.get_phone_number())
                elif call_type == "VOIP":
                    self._calling_phone_number.insert(i, self._peer_sip_address)
                else:
                    error_msg = "This test case requires call type to be executed !"
                    self._logger.error(error_msg)
                    raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)
            elif call_origin == "MT":
                self._phone_calling_list.insert(i, self._voice_call_api2)
                self._phone_receiving_list.insert(i, self._voice_call_api)
                self._ui_api_phone_list.insert(i, self._ui_api)
                if call_type == "2G" or call_type == "3G":
                    self._calling_phone_number.insert(i, self._device.get_phone_number())
                elif call_type == "VOIP":
                    self._calling_phone_number.insert(i, self._dut_sip_address)
                else:
                    error_msg = "This test case requires call type to be executed !"
                    self._logger.error(error_msg)
                    raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)
            else:
                error_msg = \
                    "This test case requires call originated type to be executed !"
                self._logger.error(error_msg)
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)

            if call_end == "NR":
                self._phone_releasing_list.insert(i, self._voice_call_api2)
            elif call_end == "MR":
                self._phone_releasing_list.insert(i, self._voice_call_api)
            else:
                error_msg = \
                    "This test case requires call end type to be executed !"
                self._logger.error(error_msg)
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)
            i += 1

        self._activate_wifi = False
        while len(self._saved_call_list) > 0:
            call_type = self._saved_call_list.pop()
            if call_type == "VOIP":
                self._activate_wifi = True
        if self._activate_wifi == True :
            if self._networking_api.get_wifi_power_status() == 0:
                # Turn Wifi interface ON
                self._networking_api.set_wifi_power("off")
                self._networking_api.set_wifi_power("on")
                # Configure the DUT
                self._networking_api.clean_all_data_connections()
                time.sleep(self._wait_btwn_cmd)
                # Connect to wifi using SSID parameter value for DUT
                self._networking_api.wifi_connect(self._ssid)
                time.sleep(self._wait_btwn_cmd)
            if self._networking_api2.get_wifi_power_status() == 0:
                self._networking_api2.set_wifi_power("off")
                self._networking_api2.set_wifi_power("on")
                self._networking_api2.clean_all_data_connections()
                time.sleep(self._wait_btwn_cmd)
                # Connect to wifi using SSID parameter value for Ref phone
                self._networking_api2.wifi_connect(self._ssid)
                time.sleep(self._wait_btwn_cmd)

        self._phonesystem_api.set_screen_timeout(900)
        self._phonesystem_api2.set_screen_timeout(900)

        # Audio Analyser Initialization
        if self._audio_analyzer.initialization(self._device.get_phone_model(), self._test_type) != 0:
            error_msg = "Audio Analyser Initialization Failed !"
            self._logger.error(error_msg)
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, error_msg)

        if self._acc_type.find("BLUETOOTH") != -1:
            self._dut_bt_address = self._bt_api.get_bt_adapter_address()

        self._sip_call_api.set_profile_sip_phone_app(self._dut_sip_profile)
        self._sip_call_api2.set_profile_sip_phone_app(self._peer_sip_profile)

        self._phone_calling_list.reverse()
        self._phone_receiving_list.reverse()
        self._phone_releasing_list.reverse()
        # self._calling_phone_number_list.reverse()
        self._ui_api_phone_list.reverse()
        self._calling_phone_number.reverse()

        # Release any previous call (Robustness)
        self._voice_call_api.release()
        self._voice_call_api2.release()

        return Global.SUCCESS, "No errors"

    def run_test(self):
        """
        Execute the test
        """
        # Call UseCaseBase run_test function
        UseCaseBase.run_test(self)

        if self._acc_type == "HEADSET":
            if self._use_io_card:
                self._wired_headset.unplug_headphone()
                self._wired_headset.plug_whs()
        elif self._acc_type == "HEADPHONE":
            if self._use_io_card:
                self._wired_headset.unplug_whs()
                self._wired_headset.plug_headphone()

        # Connect Bluetooth device
        if self._acc_type.find("BLUETOOTH") != -1:
            if self._audio_analyzer.connect_bt(self._acc_type,
                                                  self._dut_bt_address) != 0:
                error_msg = \
                    "Connect Bluetooth failed !"
                self._logger.error(error_msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, error_msg)

        self._phonesystem_api.display_off()
        self._phonesystem_api2.display_off()
        self._phonesystem_api.display_on()
        self._phonesystem_api2.display_on()
        # Unlock phones
        self._ui_api2.run_operation_set("phone_unlock")
        self._ui_api.run_operation_set("phone_unlock")

        l_audio_analyzer_result = 3
        l_call_type = self._call_list.pop()
        l_phone_calling = self._phone_calling_list.pop()
        l_phone_receiving = self._phone_receiving_list.pop()
        l_phone_ui = self._ui_api_phone_list.pop()
        self._call_type_save = l_call_type

        # Dial using PHONE_NUMBER parameter
        l_phone_calling.dial(self._calling_phone_number.pop(), False)
        time.sleep(self._wait_btwn_cmd)

        if l_call_type == "2G" or l_call_type == "3G":
            # Wait for state "active" before callSetupTimeout seconds
            l_phone_receiving.wait_for_state(
                self._uecmd_types.VOICE_CALL_STATE.INCOMING,
                self._call_setup_time)
            # Answer call
            l_phone_receiving.answer()

            # Phone1 & 2 : Check voice call is active
            l_phone_calling.wait_for_state(
                self._uecmd_types.VOICE_CALL_STATE.ACTIVE,
                self._call_setup_time)
            l_phone_receiving.wait_for_state(
                self._uecmd_types.VOICE_CALL_STATE.ACTIVE,
                self._call_setup_time)

        elif l_call_type == "VOIP":
            # Wait for state "active" before callSetupTimeout seconds
            l_phone_receiving.wait_for_audio_state(
                self._uecmd_types.AUDIO_STATE.RINGTONE,
                self._call_setup_time)
            # Answer call
            l_phone_ui.run_operation_set("call_answer")

            # Phone1 & 2 : Check voice call is active
            l_phone_calling.wait_for_audio_state(
                self._uecmd_types.AUDIO_STATE.IN_COMMUNICATION,
                self._call_setup_time)
            l_phone_receiving.wait_for_audio_state(
                self._uecmd_types.AUDIO_STATE.IN_COMMUNICATION,
                self._call_setup_time)

        # Configure Audio output to acc_type given with the test_case
        if self._acc_type == "EARPIECE":
            if self._use_io_card:
                self._wired_headset.unplug_headphone()
                self._wired_headset.unplug_whs()
        elif self._acc_type == "SPEAKER":
            # Switch to Speaker, nothing to do in Earpiece
            # Bluetooth already done with connect Bluetooth function above
            self._phonesystem_api.switch_audio_output(self._acc_type.lower())
            time.sleep(self._wait_btwn_cmd)

        # Voice call volume settings
        if self._acc_type.find("BLUETOOTH") == -1:
            # Set Voice Call Volume
            self._system_api.adjust_specified_stream_volume("VoiceCall",
                                                            self._call_stream_volume_dut)
        else:
            # Set Voice Call Volume
            self._system_api.adjust_specified_stream_volume("Bluetooth",
                                                            self._call_stream_volume_dut)

        self._system_api2.adjust_specified_stream_volume("VoiceCall",
                                                         self._call_stream_volume_ref)

        # WAIT FOR CALL DURATION
        self._logger.info(
            "Wait for call duration: %s s..." % str(self._call_duration))

        l_call_type = self._call_list.pop()
        l_phone_calling = self._phone_calling_list.pop()
        l_phone_receiving = self._phone_receiving_list.pop()
        l_phone_ui = self._ui_api_phone_list.pop()
        self._previous_call_type_save = self._call_type_save
        self._call_type_save = l_call_type
        # Dial using PHONE_NUMBER parameter
        l_phone_calling.dial(self._calling_phone_number.pop(), False)
        time.sleep(self._wait_btwn_cmd)

        if l_call_type == "2G" or l_call_type == "3G":
            # Wait for state "active" before callSetupTimeout seconds
            l_phone_receiving.wait_for_state(
                self._uecmd_types.VOICE_CALL_STATE.INCOMING,
                self._call_setup_time)
            # Answer call
            l_phone_receiving.answer()
            # Phone1 & 2 : Check voice call is active
            l_phone_calling.wait_for_state(
                self._uecmd_types.VOICE_CALL_STATE.ACTIVE,
                self._call_setup_time)
            l_phone_receiving.wait_for_state(
                self._uecmd_types.VOICE_CALL_STATE.ACTIVE,
                self._call_setup_time)
        else:
            # Answer call
            time.sleep(5)
            l_phone_ui.run_operation_set("call_answer")

            # Phone1 & 2 : Check voice call is active
            l_phone_calling.wait_for_audio_state(
                self._uecmd_types.AUDIO_STATE.IN_COMMUNICATION,
                self._call_setup_time)
            l_phone_receiving.wait_for_audio_state(
                self._uecmd_types.AUDIO_STATE.IN_COMMUNICATION,
                self._call_setup_time)

        self._call_type_list_audio_analyser_run.pop()

        while self._number_of_swap > 0:

            self._phonesystem_api2.wake_screen()
            time.sleep(self._wait_btwn_cmd)
            self._ui_api2.run_operation_set("swap_call")

            call_type_audio_analyzer = self._call_type_list_audio_analyser_run.pop()
            if call_type_audio_analyzer == "VOIP":
                self._system_api2.adjust_specified_stream_volume("VoiceCall",
                                                                (self._call_stream_volume_ref +
                                                                 self._delta_voip_volume))
            else:
                self._system_api2.adjust_specified_stream_volume("VoiceCall",
                                                                 self._call_stream_volume_ref)

            self._phonesystem_api.wake_screen()
            time.sleep(self._wait_btwn_cmd)

            # Launch audio_quality test
            l_audio_analyzer_ready, l_audio_analyzer_process = self._audio_analyzer.glitch_detection_before_switch(
                self._test_type,
                self._device.get_phone_model(),
                call_type_audio_analyzer,
                self._acc_type,
                self._signal_tested_direction)

            if l_audio_analyzer_ready == 0:
                self._ui_api.run_operation_set("swap_call")

                l_audio_analyzer_result = self._audio_analyzer.glitch_detection_after_switch(l_audio_analyzer_process)

            # Compute test verdict
            self.__compute_test_verdict__(l_audio_analyzer_result)

            self._previous_call_type_save = self._call_type_save
            self._call_type_save = call_type_audio_analyzer
            self._number_of_swap -= 1

        l_phone_releasing = self._phone_releasing_list.pop()
        # Hang up call
        l_phone_releasing.release(False)
        time.sleep(self._wait_btwn_cmd)

        l_phone_releasing = self._phone_releasing_list.pop()
        # Hang up call
        l_phone_releasing.release()

        # Phone1 & 2 : Check call is idle
        l_phone_calling.wait_for_state(
            self._uecmd_types.VOICE_CALL_STATE.NOCALL,
            self._call_setup_time)
        l_phone_receiving.wait_for_state(
            self._uecmd_types.VOICE_CALL_STATE.NOCALL,
            self._call_setup_time)

        if self._audio_analyzer_result_save == 0:
            self._verdict_comment = ("Audio Glitch detection test success with %s call sequence"
                                     % (str(self._tc_parameters.get_param_value("CALL_SEQUENCE"))))
        elif self._audio_analyzer_result_save == 1:
            self._verdict_comment = ("Audio Glitch detection test fail: Glitch "
                                     "problems detected in %s with "
                                     "%s call sequence"
                                     % (self._signal_tested_direction,
                                        self._failed_transition_audio_pb))
        elif self._audio_analyzer_result_save == 2:
            self._verdict_comment = ("Audio Glitch detection test fail: No audio signal in %s with "
                                     "%s call sequence"
                                     % (self._signal_tested_direction,
                                        self._failed_transition_no_audio))
        elif self._audio_analyzer_result_save == 3:
            self._verdict_comment = ("Audio Glitch detection test fail: Exception in executable "
                                     "in %s with %s call sequence"
                                     % (self._signal_tested_direction,
                                        self._failed_transition_no_audio))
        else:
            self._verdict_comment = ("Audio Glitch detection test fail: Unknown problems detected "
                                     "with a %s call sequence"
                                     % (str(self._tc_parameters.get_param_value("CALL_SEQUENCE"))))

        return self._result_verdict, self._verdict_comment

    def tear_down(self):
        """
        End and dispose the test
        """

        # Call UseCaseBase tear_down function
        UseCaseBase.tear_down(self)

        if self._acc_type == "HEADSET":
            if self._use_io_card:
                self._wired_headset.unplug_whs()
        elif self._acc_type == "HEADPHONE":
            if self._use_io_card:
                self._wired_headset.unplug_headphone()

        # Release any previous call (Robustness)
        if self._voice_call_api is not None:
            self._voice_call_api.release(False)
        if self._voice_call_api2 is not None:
            self._voice_call_api2.release(False)

        self._sip_call_api.delete_profile_sip_phone_app()
        self._sip_call_api2.delete_profile_sip_phone_app()

        # Reset Voice Call Volume at 70%
        if self._system_api is not None:
            self._system_api.adjust_specified_stream_volume("VoiceCall", 70)
        if self._system_api2 is not None:
            self._system_api2.adjust_specified_stream_volume("VoiceCall", 70)

        # Disable Wifi connections
        if self._result_verdict != Global.SUCCESS:
            if self._networking_api is not None:
                self._networking_api.clean_all_data_connections()
                time.sleep(self._wait_btwn_cmd)
                self._networking_api.set_wifi_power("off")
            if self._networking_api2 is not None:
                self._networking_api2.clean_all_data_connections()
                time.sleep(self._wait_btwn_cmd)
                self._networking_api2.set_wifi_power("off")

        return Global.SUCCESS, "No errors"
