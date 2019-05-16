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
:summary: This file implements the AudioComms CSV call (Check audio
quality after mute call).
:since: 04/02/2014
:author: nprecigx
"""

import time
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.Audio.LAB_AUDIO_QUALITY_BASE import LabAudioQualityBase
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.AcsBaseException import AcsBaseException
# pylint: disable=E1101


class LabAudioQualityCSVMute(LabAudioQualityBase):

    """
    AudioComms Audio CSV Mute Call class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        # Call LabAudioQualityBase Init function
        LabAudioQualityBase.__init__(self, tc_name, global_config)

        # Instantiate generic UECmd for voiceCall Ucs
        self._voice_call_api = self._device.get_uecmd("VoiceCall")
        self._phonesystem_api = self._device.get_uecmd("PhoneSystem")
        self._networking_api = self._device.get_uecmd("Networking")

        self._voice_call_api2 = None

        # Read Audio Analyzer parameters from bench config xml file (str)
        # self._audio_analyzer_node = global_config.benchConfig.\
        #    get_parameters("AudioAnalyzer")

        if self._audio_analyzer_node.get_param_value("Model") in "APx585":
            self._test_type = "audioactivity"

        # Instantiate the instances for phone caller, receiver and releaser
        self._phone_calling = None
        self._phone_receiving = None
        self._phone_releasing = None
        # Initialization of phone_number to call
        self._calling_phone_number = None

    def set_up(self):
        """
        Set up the test configuration
        """
        # Call LabAudioQualityBase Init function
        LabAudioQualityBase.set_up(self)

        if self._phone2 is not None:
            self._voice_call_api2 = self._phone2.get_uecmd("VoiceCall")
        else:
            error_msg = \
                "This test case requires two phones to be executed !"
            self._logger.error(error_msg)
            raise AcsConfigException(AcsConfigException.INVALID_BENCH_CONFIG, error_msg)

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

        return Global.SUCCESS, "No errors"

    def run_test(self):
        """
        Execute the test
        """
        # Call LabAudioQualityBase run_test function
        LabAudioQualityBase.run_test(self)

        self._voice_call_api.set_mute(1)

        # Check no audio on uplink after mute call. Check audio routing in downlink still after mute call
        audio_analyzer_result_ul = self._audio_analyzer.run(self._call_type, self._acc_type, "UL")
        audio_analyzer_result_dl = self._audio_analyzer.run(self._call_type, self._acc_type, "DL")

        # unmute call
        self._voice_call_api.set_mute(0)

        if audio_analyzer_result_ul == 2 and audio_analyzer_result_dl == 0:
            # Check audio on uplink is back after unmute
            audio_analyzer_result_ul = self._audio_analyzer.run(self._call_type, self._acc_type, "UL")
            audio_analyzer_result_dl = self._audio_analyzer.run(self._call_type, self._acc_type, "DL")

            if audio_analyzer_result_dl == 0 and audio_analyzer_result_ul == 0:
                audio_analyzer_result = 0
            else:
                audio_analyzer_result = 4
        else:
            audio_analyzer_result = 4

        # Compute test verdict and comment verdict
        self.__compute_test_verdict__(audio_analyzer_result)

        return self._result_verdict, self._verdict_comment

    def tear_down(self):
        """
        End and dispose the test
        """
        try:
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

        except AcsBaseException as acs_exception:
            self._logger.warning("Call release fail:" + str(acs_exception))

        # Call LabAudioQualityBase tear_down function
        LabAudioQualityBase.tear_down(self)

        if self._acc_type == "HEADSET":
            if self._use_io_card:
                self._wired_headset.unplug_whs()
        elif self._acc_type == "HEADPHONE":
            if self._use_io_card:
                self._wired_headset.unplug_headphone()

        # Release any previous call (Robustness)
        if self._voice_call_api is not None:
            self._voice_call_api.release()
        if self._voice_call_api2 is not None:
            self._voice_call_api2.release()

        return Global.SUCCESS, "No errors"

    def __compute_test_verdict__(self, audio_analyzer_result):
        """
        Compute the result verdict and the verdict comment

        :type Audio audio_analyzer: int
        :param Audio audio_analyzer: the Audio Analyzer return code return by run command
        """

        if audio_analyzer_result == 0:
            self._result_verdict = Global.SUCCESS
            self._verdict_comment = ("Audio Mute call test success: with "
                                     "a %s call and %s accessories"
                                     % (self._call_type,
                                        self._acc_type))
        elif audio_analyzer_result == 1 or audio_analyzer_result == 2:
            self._result_verdict = Global.FAILURE
            self._verdict_comment = ("Audio Mute call test fail: with "
                                     "a %s call and %s accessories"
                                     % (self._call_type,
                                        self._acc_type))
        elif audio_analyzer_result == 3:
            self._result_verdict = Global.BLOCKED
            self._verdict_comment = "Audio Mute call test fail: Audio analyzer executable exception occurred"
        elif audio_analyzer_result == 4:
            self._result_verdict = Global.FAILURE
            self._verdict_comment = ("Audio Mute call test fail: with "
                                     "a %s call and %s accessories: Call not muted"
                                     % (self._call_type,
                                        self._acc_type))
        else:
            self._result_verdict = Global.BLOCKED
            self._verdict_comment = ("Audio Mute call test fail: Audio analyzer problems detected "
                                     "with a %s call and %s accessories"
                                     % (self._call_type,
                                        self._acc_type))
