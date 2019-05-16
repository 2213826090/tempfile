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
:summary: This file implements the AudioComms CSV call (Check DTMF).
:since: 20/01/2014
:author: nprecigx
"""

import os
import time
from UtilitiesFWK.Utilities import Global
import acs_test_scripts.Utilities.AudioUtilities as AudioUtil
from acs_test_scripts.UseCase.Audio.LAB_AUDIO_QUALITY_BASE import LabAudioQualityBase
from ErrorHandling.AcsConfigException import AcsConfigException
from acs_test_scripts.Device.UECmd.UECmdTypes import PreferredNetwork
# pylint: disable=E1101


class LabAudioDTMFCSV(LabAudioQualityBase):

    """
    AudioComms DTMF detection class
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        # Call LabAudioQualityBase Init function
        LabAudioQualityBase.__init__(self, tc_name, global_config)

        # Initialization of the test type
        self._test_type = "dtmf"

        # Instantiate generic UECmd for voiceCall Ucs
        self._keyevent_api = self._device.get_uecmd("KeyEvent")
        self._ui_api = self._device.get_uecmd("Ui")
        self._ui_api.set_global_config(global_config)

        # Initialize UI object for DUT
        self._ui_api.init()

        if self._phone2 is not None:
            self._voice_call_api2 = self._phone2.get_uecmd("VoiceCall")
            self._ui_api2 = self._phone2.get_uecmd("Ui")
            self._ui_api2.set_global_config(global_config)

            # Initialize UI object for reference phone
            self._ui_api2.init()
        else:
            error_msg = \
                "This test case requires two phones to be executed !"
            self._logger.error(error_msg)
            raise AcsConfigException(AcsConfigException.INVALID_BENCH_CONFIG, error_msg)

        # Instantiate the instances for phone caller, receiver and releaser
        self._phone_calling = None
        self._phone_receiving = None
        self._phone_releasing = None
        # Initialization of phone_number to call
        self._calling_phone_number = None
        self._phone_ui = None

        if self._signal_tested_direction == "UL":
            self._phone_ui = self._ui_api
        else:
            self._phone_ui = self._ui_api2

        self._dtmf_list = str(self._tc_parameters.get_param_value("DTMF_LIST"))
        self._dtmf_list = self._dtmf_list.strip('[]').split(',')

        if self._audio_analyzer_node.get_param_value("Model") in "RS_UPV":
            self._dtmf_not_detect_dut = []
            self._dtmf_not_detect_ref = []

            self._dial_type = self._tc_parameters.get_param_value("DIAL_TYPE")

            # Read ConfPath from AUDIO_ANALYZER BenchConfig.xml
            self._aa_conf_path = os.path.join(str(self._audio_analyzer_node.get_param_value("ConfPath")))

            # Read DestPath from AUDIO_ANALYZER BenchConfig.xml
            self._aa_dest_path = os.path.join(str(self._audio_analyzer_node.get_param_value("DestPath")))

            # Construct trace file path on bench computer
            self._host_trace_file_path = os.path.join(self._aa_conf_path,
                                                      "dtmf_trace.trc")

            # Construct trace file path on audio analyzer
            self._aa_trace_file_path = os.path.join(self._aa_dest_path,
                                                    "dtmf_trace.trc")

            if self._signal_tested_direction in "DL":
                self._dtmf_check_order = ["REF", "DUT"]
            else:
                self._dtmf_check_order = ["DUT", "REF"]

    def set_up(self):
        """
        Set up the test configuration
        """
        # Call LabAudioQualityBase Init function
        LabAudioQualityBase.set_up(self)

        self._phonesystem_api2 = self._phone2.get_uecmd("PhoneSystem")

        if self._call_type == "2G":
            self._networking_api.set_preferred_network_type(PreferredNetwork.GSM_ONLY)
        elif self._call_type == "3G":
            self._networking_api.set_preferred_network_type(PreferredNetwork.WCDMA_ONLY)
        elif self._call_type == "4G":
            self._networking_api.set_preferred_network_type(PreferredNetwork.LTE_ONLY)
        elif self._call_type == "WCDMA_preferred":
            self._networking_api.set_preferred_network_type(PreferredNetwork.WCDMA_PREF)
        else:
            msg = "wrong value of parameter CALL TYPE"
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        if self._audio_analyzer_node.get_param_value("Model") in "RS_UPV":

            # RS UPV Initialization
            self._audio_analyzer.init()

            # Load a configuration on the RS UPV
            self._audio_analyzer.load_configuration(self._name,
                                                    None,
                                                    None,
                                                    None,
                                                    [self._dial_type, self._signal_tested_direction])

        # Setup phone type ( calling, receiving, releasing phone)
        if self._call_origin_type == "MO":
            self._phone_calling = self._voice_call_api
            self._phone_receiving = self._voice_call_api2
            self._calling_phone_number = self._phone2.get_phone_number()
        elif self._call_origin_type == "MT":
            self._phone_calling = self._voice_call_api2
            self._phone_receiving = self._voice_call_api
            self._calling_phone_number = self._device.get_phone_number()
        else:
            error_msg = \
                "This test case requires call originated type to be executed !"
            self._logger.error(error_msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)

        if self._call_end_type == "NR":
            self._phone_releasing = self._voice_call_api2
        elif self._call_end_type == "MR":
            self._phone_releasing = self._voice_call_api
        else:
            error_msg = \
                "This test case requires call end type to be executed !"
            self._logger.error(error_msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)

        # Release any previous call (Robustness)
        self._voice_call_api.release()
        self._voice_call_api2.release()

        return Global.SUCCESS, "No errors"

    def run_test(self):
        """
        Execute the test
        """
        # Call LabAudioQualityBase run_test function
        LabAudioQualityBase.run_test(self)

        if self._acc_type == "HEADSET":
            if self._use_io_card:
                self._wired_headset.unplug_headphone()
                self._wired_headset.plug_whs()
        elif self._acc_type == "HEADPHONE":
            if self._use_io_card:
                self._wired_headset.unplug_whs()
                self._wired_headset.plug_headphone()

        # Dial using PHONE_NUMBER parameter
        self._phone_calling.dial(self._calling_phone_number)
        time.sleep(self._wait_btwn_cmd)

        # Wait for state "active" before callSetupTimeout seconds
        self._phone_receiving.wait_for_state(
            self._uecmd_types.VOICE_CALL_STATE.INCOMING,
            self._call_setup_time)

        # Answer call
        self._phone_receiving.answer()

        # Phone1 & 2 : Check voice call is active
        self._phone_calling.wait_for_state(
            self._uecmd_types.VOICE_CALL_STATE.ACTIVE,
            self._call_setup_time)
        self._phone_receiving.wait_for_state(
            self._uecmd_types.VOICE_CALL_STATE.ACTIVE,
            self._call_setup_time)

        if self._audio_analyzer_node.get_param_value("Model") in "APx585":
            # Configure Audio output to acc_type given with the test_case
            if self._acc_type == "EARPIECE":
                if self._use_io_card:
                    self._wired_headset.unplug_whs()
                    self._wired_headset.unplug_headphone()
            elif self._acc_type == "SPEAKER":
                # Switch to Speaker, nothing to do in Earpiece
                # Bluetooth already done with connect Bluetooth function above
                self._phonesystem_api.switch_audio_output(self._acc_type.lower())
                time.sleep(self._wait_btwn_cmd)

            self._system_api2.adjust_specified_stream_volume("VoiceCall",
                                                             self._call_stream_volume_ref)

            self._phonesystem_api.display_off()
            self._phonesystem_api2.display_off()
            self._phonesystem_api.display_on()
            self._phonesystem_api2.display_on()
            self._dtmf_touch = self._dtmf_list.pop()
            self._phone_ui.run_operation_set("open_dial_pad_incall")

            # Launch audio_quality test
            audio_analyzer_result, l_dtmf_find = self._audio_analyzer.get_dtmf(self._acc_type,
                                                                    self._signal_tested_direction,
                                                                    self._dtmf_touch)
            self._logger.info(
                "DTMF touch find : %s " % str(l_dtmf_find))

            if audio_analyzer_result == 0:
                if str(l_dtmf_find) == self._dtmf_touch:
                    self._result_verdict = 0
                else:
                    self._result_verdict = 1
            elif audio_analyzer_result == 1:
                self._result_verdict = 1
            else:
                self._result_verdict = 3
            # Compute test verdict and comment verdict
            self.__compute_test_verdict__(self._result_verdict)

        # Start measurement on Audio Analyzer
        elif self._audio_analyzer_node.get_param_value("Model") in "RS_UPV":
            self._audio_analyzer.start_continuous_measurement()

            # Open DUT dial pad
            self._logger.info("Open the DUT dial pad")
            self._ui_api.run_operation_set("open_dial_pad_incall")

            # Open remote phone dial pad
            self._logger.info("Open the REF phone dial pad")
            self._ui_api2.run_operation_set("open_dial_pad_incall")

            # Start dialing and analyzing the DTMF tones for call_duration s
            for i in range(1, self._call_duration, 60):

                # Start dialing on the DUT dial pad
                self._logger.info("")
                self._logger.info("Start dialing on the DUT dial pad")
                self._logger.info("")

                # Select on the Audio Analyzer which input channel will trigger the measurement (CH1 = DUT, CH2 = REF)
                self._audio_analyzer.switch_trigger_source(self._dtmf_check_order[0])

                for dial_key_current in self._dtmf_list:

                    # Dial a single key on the DUT dial pad
                    self._dial_incall_dtmf(dial_key_current, self._dial_type, "DUT")

                    # Wait for the UPV to analyze the DTMF tone
                    time.sleep(1)

                    # Analyze the tones emitted on DUT and reference phone
                    self._audio_analyzer.store_data_list(self._aa_trace_file_path, self._host_trace_file_path, "FFT")

                    if (AudioUtil.detect_dtmf_tone(self._host_trace_file_path,
                                                   dial_key_current,
                                                   self._dtmf_check_order[0],
                                                   self._logger)):
                        self._logger.info("DTMF tone detected on %s party when dialing %s from DUT" %
                                          (self._signal_tested_direction, dial_key_current))
                    else:
                        self._logger.warning("Dial tone for key %s not detected by %s user when dialing from DUT" % (
                            dial_key_current,
                            self._signal_tested_direction))
                        self._dtmf_not_detect_dut.append(dial_key_current)

                    # Wait 1 second between successive dial
                    time.sleep(1)

                if self._signal_tested_direction == "DL":
                    # Start dialing on the reference phone dial pad
                    self._logger.info("")
                    self._logger.info("Start dialing on the reference phone dial pad")
                    self._logger.info("")

                    # Select on the Audio Analyzer which input channel will trigger the measurement
                    # (CH1 = DUT, CH2 = REF)
                    self._audio_analyzer.switch_trigger_source(self._dtmf_check_order[1])

                    for dial_key_current in self._dtmf_list:

                        # Dial a single key on the REF phone dial pad
                        self._dial_incall_dtmf(dial_key_current, self._dial_type, "REF")

                        time.sleep(1)

                        # Analyze the tones emitted on DUT and reference phone
                        self._audio_analyzer.store_data_list(self._aa_trace_file_path, self._host_trace_file_path, "FFT")

                        if (AudioUtil.detect_dtmf_tone(self._host_trace_file_path,
                                                       dial_key_current,
                                                       self._dtmf_check_order[1],
                                                       self._logger)):
                            self._logger.info("DTMF tone detected on %s party when dialing %s from reference phone" %
                                              (self._signal_tested_direction, dial_key_current))
                        else:
                            self._logger.warning("Dial tone for key %s not detected by %s" \
                                                 " user when dialing from reference phone" % (
                                                     dial_key_current,
                                                     self._signal_tested_direction))
                            self._dtmf_not_detect_ref.append(dial_key_current)

                        # Wait 1 second between successive dial
                        time.sleep(1)

            dut_detect_rate = 1 - len(self._dtmf_not_detect_dut) / float(
                len(self._dtmf_list) * len(range(1, self._call_duration, 60)))
            ref_detect_rate = 1 - len(self._dtmf_not_detect_ref) / float(
                len(self._dtmf_list) * len(range(1, self._call_duration, 60)))

            self._verdict_comment = ("DTMF detection rate target: %.3f%% - "
                                     "DTMF detection rate obtained when dialing on the DUT: %.3f%% - "
                                     "DTMF detection rate obtained when dialing on the REF phone: %.3f%%"
                                     % (75, dut_detect_rate * 100, ref_detect_rate * 100))

            if dut_detect_rate >= 0.75 and ref_detect_rate >= 0.75:
                self._result_verdict = Global.SUCCESS
            else:
                self._result_verdict = Global.FAILURE

            self._dtmf_not_detect_dut = []
            self._dtmf_not_detect_ref = []

            # Stop measurement on Audio Analyzer
            self._audio_analyzer.stop_measurement()

        # RELEASE THE CALL
        # Phone1 & 2 : Check call is still active
        self._phone_calling.check_state(
            self._uecmd_types.VOICE_CALL_STATE.ACTIVE)
        self._phone_receiving.check_state(
            self._uecmd_types.VOICE_CALL_STATE.ACTIVE)

        # Hang up call
        self._phone_releasing.release()

        # Phone1 & 2 : Check call is idle
        self._phone_calling.wait_for_state(
            self._uecmd_types.VOICE_CALL_STATE.NOCALL,
            self._call_setup_time)
        self._phone_receiving.wait_for_state(
            self._uecmd_types.VOICE_CALL_STATE.NOCALL,
            self._call_setup_time)

        return self._result_verdict, self._verdict_comment

    def tear_down(self):
        """
        End and dispose the test
        """

        # Call LabAudioQualityBase tear_down function
        LabAudioQualityBase.tear_down(self)

        if self._acc_type == "HEADSET":
            if self._use_io_card:
                self._wired_headset.plug_whs()
        elif self._acc_type == "HEADPHONE":
            if self._use_io_card:
                self._wired_headset.unplug_headphone()

        # Release any previous call (Robustness)
        if self._voice_call_api is not None:
            self._voice_call_api.release()
        if self._voice_call_api2 is not None:
            self._voice_call_api2.release()

        if self._audio_analyzer_node.get_param_value("Model") in "RS_UPV":
            # Stop measurement on Audio Analyzer (Robustness)
            self._audio_analyzer.stop_measurement()

        return Global.SUCCESS, "No errors"

    def __compute_test_verdict__(self, audio_analyzer_result):
        """
        Compute the result verdict and the verdict comment

        :type Audio audio_analyzer: int
        :param Audio audio_analyzer: the Audio Analyzer return code return by run command
        """

        if audio_analyzer_result == 0:
            self._result_verdict = Global.SUCCESS
            self._verdict_comment = ("Audio DTMF test success: with "
                                     "a %s call and %s accessories"
                                     % (self._call_type,
                                        self._acc_type))
        elif audio_analyzer_result == 1:
            self._result_verdict = Global.FAILURE
            self._verdict_comment = ("Audio DTMF test fail: with "
                                     "a %s call and %s accessories"
                                     % (self._call_type,
                                        self._acc_type))
        elif audio_analyzer_result == 3:
            self._result_verdict = Global.BLOCKED
            self._verdict_comment = "Audio DTMF test fail: Audio analyzer executable exception occurred"
        else:
            self._result_verdict = Global.BLOCKED
            self._verdict_comment = ("Audio DTMF test fail: Audio analyzer problems detected "
                                     "with a %s call and %s accessories"
                                     % (self._call_type,
                                        self._acc_type))

    def _dial_incall_dtmf(self, dial_key, dial_type, dialing_party):
        """

        :type dial_key: basestring
        :param dial_key: Key to dial while in call
        :type dial_type: basestring
        :param dial_type: Type of keypress (long/short) to use
        :type dialing_party: basestring
        :param dialing_party: Dialing party (DUT/REF)

        """

        if dial_type in "long":
            self._logger.info("Dial with a long keypress from %s", dialing_party)
            self._ui_api.run_operation_set("dial_long")
        elif dial_type in "short":
            if dialing_party in "DUT":
            # Dial a single key on the DUT dial pad
                self._phone_calling.dial(dial_key, check_state=False, single_dial=True)

            elif dialing_party in "REF":
            # Dial a single key on the REF phone dial pad
                self._phone_receiving.dial(dial_key, check_state=False, single_dial=True)
            else:
                raise AcsConfigException(AcsConfigException.INVALID_BENCH_CONFIG, "Invalid dialing party type")
        else:
            raise AcsConfigException(AcsConfigException.INVALID_BENCH_CONFIG, "Invalid dialing type")
