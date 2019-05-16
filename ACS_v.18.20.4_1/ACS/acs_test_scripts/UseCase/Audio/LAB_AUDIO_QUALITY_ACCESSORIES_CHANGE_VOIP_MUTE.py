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
:summary: This file implements the AudioComms accessories change interaction with mute function during VOIP call class.
:since: 27/11/2014
:author: fbelvezx
"""

import time
from UtilitiesFWK.Utilities import Global
from ErrorHandling.TestEquipmentException import TestEquipmentException
from acs_test_scripts.UseCase.Audio.LAB_AUDIO_QUALITY_ACCESSORIES_CHANGE_BASE \
    import LabAudioQualityAccessoriesChangeBase
from acs_test_scripts.UseCase.Audio.LAB_AUDIO_QUALITY_ACCESSORIES_CHANGE_VOIP \
    import LabAudioQualityAccessoriesChangeVOIP


class LabAudioQualityAccessoriesChangeVOIPMute(LabAudioQualityAccessoriesChangeVOIP):

    """
    AudioComms accessories change interaction with mute function during VOIP call class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        # Call LabAudioQualityAccessoriesChangeBase Init function
        LabAudioQualityAccessoriesChangeVOIP.__init__(self, tc_name, global_config)

        self._test_type = "audioactivity"

    def run_test(self):
        """
        Execute the test
        """
        # Call LabAudioQualityAccessoriesChangeBase run_test function
        LabAudioQualityAccessoriesChangeBase.run_test(self)

        # Save accessories list in temporary copy
        self._temp_acc_list_split = [acc for acc in self._acc_list_split]
        self._temp_call_volume_dut_list_split = [vol_dut for vol_dut in self._call_volume_dut_list_split]
        self._temp_call_volume_ref_list_split = [vol_ref for vol_ref in self._call_volume_ref_list_split]
        self._temp_acc_active_list_split = [acc_act for acc_act in self._acc_active_list_split]

        self._sip_call_api.toogle_mute()

        # Check no audio on uplink after mute call
        audio_analyzer_result_ul = self._audio_analyzer.run(self._call_type, self._temp_acc_list_split[-1], "UL")
        audio_analyzer_result_dl = self._audio_analyzer.run(self._call_type, self._temp_acc_list_split[-1], "DL")

        # Update audio test type for accessory change
        self._test_type = "audioquality"

        # Switch to audio quality setup on audio analyzer
        if self._audio_analyzer.initialization(self._device.get_phone_model(), self._test_type) != 0:
            error_msg = \
                "Audio Analyzer Initialization Failed !"
            self._logger.error(error_msg)
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, error_msg)

        if audio_analyzer_result_ul == 2 and audio_analyzer_result_dl == 0:

            # Change Accessories
            while len(self._temp_acc_list_split) > 0:
                self._previous_acc_type = self._acc_type
                self._acc_type = self._temp_acc_list_split.pop()
                self._logger.info(
                    "Accessories transition: " + self._previous_acc_type + " => " + self._acc_type)
                # Configure Audio output to acc_type given with the test_case
                if "EARPIECE" in self._acc_type:
                    self._sip_call_api.switch_to_earpiece()
                    time.sleep(self._wait_btwn_cmd)
                elif "SPEAKER" in self._acc_type:
                    self._sip_call_api.switch_to_speaker()
                    time.sleep(self._wait_btwn_cmd)
                elif self._acc_type == "HEADSET":
                    self._sip_call_api.switch_to_earpiece()
                    time.sleep(self._wait_btwn_cmd)
                    if self._use_io_card:
                        self._wired_headset.unplug_headphone()
                        self._wired_headset.plug_whs()
                elif self._acc_type == "HEADPHONE":
                    self._sip_call_api.switch_to_earpiece()
                    time.sleep(self._wait_btwn_cmd)
                    if self._use_io_card:
                        self._wired_headset.unplug_whs()
                        self._wired_headset.plug_headphone()
                elif "BLUETOOTHHSP" in self._acc_type:
                    self._sip_call_api.switch_to_bluetooth()
                elif "BLUETOOTHHFP" in self._acc_type:
                    self._sip_call_api.switch_to_bluetooth()
                elif "BLUETOOTHA2DP" in self._acc_type:
                    self._sip_call_api.switch_to_bluetooth_a2dp()
                else:
                    self._logger.error("Unknown accessories => Change accessories failed")
                time.sleep(self._wait_btwn_cmd)

                # Set Stream Voice Call Volume
                l_volume_dut = self._temp_call_volume_dut_list_split.pop()
                l_volume_ref = self._temp_call_volume_ref_list_split.pop()
                l_acc_active = int(self._temp_acc_active_list_split.pop())

                if self._acc_type.find("BLUETOOTH") == -1:
                    # Set Voice Call Volume
                    self._system_api.adjust_specified_stream_volume("VoiceCall",
                                                                    int(l_volume_dut))
                else:
                    # Set Voice Call Volume
                    self._system_api.adjust_specified_stream_volume("Bluetooth",
                                                                    int(l_volume_dut))

                self._system_api2.adjust_specified_stream_volume("VoiceCall",
                                                                 int(l_volume_ref))

                # WAIT FOR CALL DURATION
                self._logger.info(
                    "Wait for call duration: " + str(self._call_duration) + "s...")
                time.sleep(self._call_duration)

                if l_acc_active == 1:
                    # No need to launch audio quality test in uplink since the call is muted
                    l_audio_analyzer_result_ul = 0

                    # Launch audio_quality test
                    l_audio_analyzer_result_dl = int(self._audio_analyzer.run(self._call_type, self._acc_type, "dl"))

                    # Compute test verdict
                    self._compute_test_verdict_(l_audio_analyzer_result_ul, l_audio_analyzer_result_dl)
                else:
                    self._logger.info("accessory: " + self._acc_type + " not active => not tested")

            #unmute call
            self._sip_call_api.toogle_mute()

            self._test_type = "audioactivity"

            # Switch to audio routing setup on audio analyzer, for mute test
            if self._audio_analyzer.initialization(self._device.get_phone_model(), self._test_type) != 0:
                error_msg = \
                    "Audio Analyzer Initialization Failed !"
                self._logger.error(error_msg)
                raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, error_msg)

            # Check audio on uplink is back after unmute
            audio_analyzer_result_ul = self._audio_analyzer.run(self._call_type, self._acc_type, "UL")
            audio_analyzer_result_dl = self._audio_analyzer.run(self._call_type, self._acc_type, "DL")
            if audio_analyzer_result_dl == 0 and audio_analyzer_result_ul == 0:
                audio_analyzer_result = 0
            else:
                audio_analyzer_result = 4
        elif audio_analyzer_result_ul == 0 and audio_analyzer_result_dl == 0:
            audio_analyzer_result = 4
            self._verdict_comment = "Call not muted in uplink"
            self._logger.error(self._verdict_comment + "!")
            self._result_verdict = Global.FAILURE
        else:
            audio_analyzer_result = 4

        if self._result_verdict is Global.SUCCESS:
            # Compute verdict for mute test in case accessory change was successful
            self.__compute_test_verdict_mute(audio_analyzer_result)

        return self._result_verdict, self._verdict_comment

    def __compute_test_verdict_mute(self, audio_analyzer_result):
        """
        Compute the result verdict and the verdict comment, for mute test

        :type audio_analyzer_result: int
        :param audio_analyzer_result: the Audio Analyzer return code return by run command for
        mute test
        """

        if audio_analyzer_result == 0:
            self._result_verdict = Global.SUCCESS
            self._verdict_comment += ("\nAudio Mute call test success: with "
                                      "a %s call and %s accessories"
                                      % (self._call_type,
                                         self._acc_type))
        elif audio_analyzer_result == 1 or audio_analyzer_result == 2:
            self._result_verdict = Global.FAILURE
            self._verdict_comment += ("\nAudio Mute call test fail: with "
                                      "a %s call and %s accessories"
                                      % (self._call_type,
                                         self._acc_type))
        elif audio_analyzer_result == 3:
            self._result_verdict = Global.BLOCKED
            self._verdict_comment += "\nAudio Mute call test fail: Audio analyzer executable exception occurred"
        elif audio_analyzer_result == 4:
            self._result_verdict = Global.FAILURE
            self._verdict_comment += ("\nAudio Mute call test fail: with "
                                      "a %s call and %s accessories: Call not muted"
                                      % (self._call_type,
                                         self._acc_type))
        else:
            self._result_verdict = Global.BLOCKED
            self._verdict_comment += ("\nAudio Mute call test fail: Audio analyzer problems detected "
                                      "with a %s call and %s accessories"
                                      % (self._call_type,
                                         self._acc_type))