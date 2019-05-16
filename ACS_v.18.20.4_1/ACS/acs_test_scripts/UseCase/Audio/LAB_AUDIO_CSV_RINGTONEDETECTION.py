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
ringtone).
:since: 05/05/2014
:author: nprecigx
"""

import time
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.Audio.LAB_AUDIO_QUALITY_BASE import LabAudioQualityBase
from ErrorHandling.AcsConfigException import AcsConfigException
# pylint: disable=E1101


class LabAudioCSVRingtoneDetection(LabAudioQualityBase):

    """
    AudioComms LabAudioCSVRingtoneDetection class.
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

        # Setup phone type for this test Mobile terminated is mandatory
        self._phone_calling = self._voice_call_api2
        self._phone_receiving = self._voice_call_api
        self._calling_phone_number = self._device.get_phone_number()
        self._phone_releasing = self._voice_call_api2

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

        self._system_api.adjust_specified_stream_volume("Ringtone",
                                                        self._call_stream_volume_dut)

        # Dial using PHONE_NUMBER parameter
        self._phone_calling.dial(self._calling_phone_number)
        time.sleep(self._wait_btwn_cmd)

        # Wait for state "active" before callSetupTimeout seconds
        self._phone_receiving.wait_for_state(
            self._uecmd_types.VOICE_CALL_STATE.INCOMING,
            self._call_setup_time)

        # Check ringtone
        audio_analyzer_result = self._audio_analyzer.ringtone_detection(self._acc_type)

        # Answer for all accessories except Bluetooth( auto answer by audio_analyzer for bt)
        if self._acc_type.find("BLUETOOTH") == -1:
            # Answer call
            self._phone_receiving.answer()

        # Phone1 & 2 : Check voice call is active
        self._phone_calling.wait_for_state(
            self._uecmd_types.VOICE_CALL_STATE.ACTIVE,
            self._call_setup_time)
        self._phone_receiving.wait_for_state(
            self._uecmd_types.VOICE_CALL_STATE.ACTIVE,
            self._call_setup_time)

        # Compute test verdict and comment verdict
        LabAudioQualityBase.__compute_test_verdict__(self, audio_analyzer_result)

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

    def __compute_test_verdict__(self, apx_result):
        """
        Compute the result verdict and the verdict comment

        @type apx_result: int
        @param apx_result: the apx585 return code return by run command
        """

        if apx_result == 0:
            self._result_verdict = Global.SUCCESS
            self._verdict_comment = ("Ringtone present with "
                                     "a %s call and %s accessories"
                                     % (self._call_type,
                                        self._acc_type))
        elif apx_result == 1:
            self._result_verdict = Global.FAILURE
            self._verdict_comment = ("Ringtone not present with a %s call and "
                                     "%s accessories"
                                     % (self._call_type,
                                        self._acc_type))
        elif apx_result == 3:
            self._result_verdict = Global.BLOCKED
            self._verdict_comment = "Ringtone detection fail: Apx585 executable exception occurred"
        else:
            self._result_verdict = Global.BLOCKED
            self._verdict_comment = ("Ringtone detection fail: APx585 problems detected "
                                     "with a %s call and %s accessories"
                                     % (self._call_type,
                                        self._acc_type))
