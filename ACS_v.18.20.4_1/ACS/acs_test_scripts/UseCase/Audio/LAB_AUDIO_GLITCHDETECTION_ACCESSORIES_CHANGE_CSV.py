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
:summary: This file implements the AudioCommsCSV call (Check Glitch Detection between change accessories)
with accessories change during call.
:since: 02/02/2015
:author: hgarantx
"""

import time
from UtilitiesFWK.Utilities import Global
from ErrorHandling.AcsConfigException import AcsConfigException
from acs_test_scripts.UseCase.Audio.LAB_AUDIO_QUALITY_ACCESSORIES_CHANGE_BASE \
    import LabAudioQualityAccessoriesChangeBase
from acs_test_scripts.Device.UECmd.UECmdTypes import PreferredNetwork

# pylint: disable=E1101


class LabAudioGlitchDetectionAccessoriesChangeCSV(LabAudioQualityAccessoriesChangeBase):

    """
    AudioComms Audio Glitch Detection CSV Call Accessories Change class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        # Call LabAudioQualityAccessoriesChangeBase Init function
        LabAudioQualityAccessoriesChangeBase.__init__(self, tc_name, global_config)

        # Initialization of the test type
        self._test_type = "glitchdetectiononswitch"

        # Instantiate generic UECmd for voiceCall Ucs
        self._voice_call_api = self._device.get_uecmd("VoiceCall")
        self._phonesystem_api = self._device.get_uecmd("PhoneSystem")
        self._phonemodem_api = self._device.get_uecmd("Modem")
        self._networking_api = self._device.get_uecmd("Networking")

        self._voice_call_api2 = None
        # Instantiate the instances for phone caller, receiver and releaser
        self._phone_calling = None
        self._phone_receiving = None
        self._phone_releasing = None
        # Initialization of phone_number to call
        self._calling_phone_number = None

        # Copy of accessories list
        self._temp_acc_list_splited = None

    def set_up(self):
        """
        Set up the test configuration
        """
        # Call LabAudioQualityAccessoriesChangeBase Setup function
        LabAudioQualityAccessoriesChangeBase.set_up(self)

        self._phonesystem_api.wake_screen()

        self._voice_call_api2 = self._phone2.get_uecmd("VoiceCall")

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
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     error_msg)

        if self._call_end_type == "NR":
            self._phone_releasing = self._voice_call_api2
        elif self._call_end_type == "MR":
            self._phone_releasing = self._voice_call_api
        else:
            error_msg = \
                "This test case requires call end type to be executed !"
            self._logger.error(error_msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     error_msg)

        # Release any previous call (Robustness)
        self._voice_call_api.release()
        self._voice_call_api2.release()

        return Global.SUCCESS, "No errors"

    def run_test(self):
        """
        Execute the test
        """
        # Call LabAudioQualityAccessoriesChangeBase run_test function
        LabAudioQualityAccessoriesChangeBase.run_test(self)

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

        # Save accessories list in temporary copy
        self._temp_acc_list_splited = self._acc_list_split

        # Change Accessories
        while len(self._temp_acc_list_splited) > 0:
            self._previous_acc_type = self._acc_type
            self._acc_type = self._temp_acc_list_splited.pop()
            self._logger.info(
                "Accessories transition: " + self._previous_acc_type + " => " + self._acc_type)
            # Phone1 & 2 : Check voice call is active
            self._phone_calling.wait_for_state(
                self._uecmd_types.VOICE_CALL_STATE.ACTIVE,
                self._call_setup_time)
            self._phone_receiving.wait_for_state(
                self._uecmd_types.VOICE_CALL_STATE.ACTIVE,
                self._call_setup_time)

            # Set Stream Voice Call Volume
            l_volume_dut = self._call_volume_dut_list_split.pop()
            l_volume_ref = self._call_volume_ref_list_split.pop()
            l_acc_active = int(self._acc_active_list_split.pop())

            if l_acc_active == 1:
                # Need to test accessories
                # Launch audio_quality test
                l_audio_analyzer_ready, l_audio_analyzer_process = self._audio_analyzer.glitch_detection_before_switch(
                    self._test_type,
                    self._device.get_phone_model(),
                    self._call_type,
                    self._acc_type,
                    self._signal_tested_direction)

                # Ready for switch
                if l_audio_analyzer_ready == 0:
                    if self._acc_type.find("BLUETOOTH") == -1:
                        # Set Voice Call Volume
                        self._system_api.adjust_specified_stream_volume("VoiceCall", int(l_volume_dut))
                    else:
                        # Set Voice Call Volume
                        self._system_api.adjust_specified_stream_volume("Bluetooth", int(l_volume_dut))

                    self._system_api2.adjust_specified_stream_volume("VoiceCall", int(l_volume_ref))

                    # Configure Audio output to acc_type given with the test_case
                    # except in bluetooth test case
                    if "SPEAKER" in self._acc_type:
                        self._phonesystem_api.switch_audio_output(self._acc_type.lower())
                        time.sleep(self._wait_btwn_cmd)
                    elif "EARPIECE" in self._acc_type:
                        self._phonesystem_api.switch_audio_output("headset")
                        if self._use_io_card:
                            self._wired_headset.unplug_headphone()
                            self._wired_headset.unplug_whs()
                        time.sleep(self._wait_btwn_cmd)
                    elif "HEADSET" in self._acc_type:
                        if self._use_io_card:
                            self._wired_headset.unplug_headphone()
                            self._wired_headset.plug_whs()
                    elif self._acc_type == "HEADPHONE":
                        if self._use_io_card:
                            self._wired_headset.unplug_whs()
                            self._wired_headset.plug_headphone()
                    elif self._acc_type.find("BLUETOOTH") != -1:
                        self._phonesystem_api.switch_audio_output("bluetooth")
                    else:
                        self._logger.error("Unknown accessories => Change accessories failed")

                    l_audio_analyzer_result = self._audio_analyzer.glitch_detection_after_switch(
                        l_audio_analyzer_process)

                    # Compute test verdict
                    self._compute_test_verdict_(l_audio_analyzer_result, 0)
                else:
                    self._logger.info("Glitch Detection initialization problem")
                    self._audio_analyzer_result_save == 3
            else:
                self._logger.info("accessory: " + self._acc_type + " not active => not tested")

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

        # Call LabAudioQualityAccessoriesChangeBase tear_down function
        LabAudioQualityAccessoriesChangeBase.tear_down(self)

        if self._acc_type == "HEADSET":
            if self._use_io_card:
                self._wired_headset.unplug_whs()
        elif self._acc_type == "HEADPHONE":
            if self._use_io_card:
                self._wired_headset.plug_headphone()

        # Release any previous call (Robustness)
        if self._voice_call_api is not None:
            self._voice_call_api.release()
        if self._voice_call_api2 is not None:
            self._voice_call_api2.release()

        return Global.SUCCESS, "No errors"
