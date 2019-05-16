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
:summary: This file implements the check for the presence of recording tone during a voice call
on simulated network in 2G
:since: 07/08/2013
:author: fbelvezx
"""

import os
from UtilitiesFWK.Utilities import Global
import acs_test_scripts.Utilities.AudioUtilities as AudioUtil
from acs_test_scripts.UseCase.Audio.LAB_AUDIO_GSM_VC_BASE import LabAudioGsmVcBase
from ErrorHandling.AcsBaseException import AcsBaseException


class LabAudioGsmVcRecordtones(LabAudioGsmVcBase):

    """
    Lab Audio Gsm Voice Call Recording tone class
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        LabAudioGsmVcBase.__init__(self, tc_name, global_config)

        # Read PHONE_NUMBER from testcase xml parameters
        if (self._tc_parameters.get_param_value("PHONE_NUMBER") not in (None, '')) \
                and str(self._tc_parameters.get_param_value("PHONE_NUMBER")).isdigit():
            self._phone_number = self._tc_parameters.get_param_value("PHONE_NUMBER")
        else:
            self._phone_number = str(self._device.get_phone_number())

        # Get PSI Recorder API
        self._psi_recorder = self._device.get_uecmd("PsiRecorder")

        # Read source input for PSI recorder from testcase xml
        self._rec_source = self._tc_parameters.get_param_value("SOURCE_INPUT")

        # Read container for PSI recorder from testcase xml
        self._rec_container = self._tc_parameters.get_param_value("CONTAINER")

        # Read audio codec for PSI recorder from testcase xml
        self._rec_codec = self._tc_parameters.get_param_value("REC_CODEC")

        # Read audio bitrate for PSI recorder from testcase xml
        self._rec_bitrate = self._tc_parameters.get_param_value("REC_BITRATE")

        # Read sample rate for PSI recorder from testcase xml
        self._rec_samplerate = self._tc_parameters.get_param_value("SAMPLERATE")

        # Read the number of channel for PSI recorder from testcase xml
        self._rec_channelnum = self._tc_parameters.get_param_value("CHANNELNUM")

        # Read AUDIO_FILE_SPEECH from test case xml file
        self._ref_speech_file = str(self._tc_parameters.get_param_value("AUDIO_FILE_SPEECH"))

        # Construct reference audio file path on RS UPV
        self._aa_speech_file_path = os.path.join(self._aa_dest_path, self._ref_speech_file)

        # Read time between 2 record tones from testcase xml
        self._time_between_tones = int(self._tc_parameters.get_param_value("TONE_PERIODICITY"))

        # Read record tone expected frequency
        self._tone_frequency = float(self._tc_parameters.get_param_value("TONE_FREQUENCY"))

        # DTX enabling status
        self._dtx_enabled = str(self._tc_parameters.get_param_value("DTX"))

        self._verdict_comment = "Recording tone not detected"

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Setup
        """
        LabAudioGsmVcBase.set_up(self)

        self._result_verdict = Global.FAILURE

        # Configure Audio output to headset (jack)
        self._phonesystem_api.switch_audio_output("headset")

        # Launch PSI recorder with the corresponding settings
        self._psi_recorder.preset(self._rec_codec,
                                           self._rec_samplerate,
                                           self._rec_channelnum,
                                           self._rec_bitrate,
                                           self._rec_container)
        self._logger.info("Launch PSI recorder with the corresponding settings")

        # Configure Mobile DTX state
        if self._dtx_enabled in "ON":
            self._ns_2g.set_mobile_dtx("ON")
        else:
            self._ns_2g.set_mobile_dtx("OFF")

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------
    def run_test(self):
        """
        Execute the test
        """
        # Call LabAudioVcBase Run Test function
        LabAudioGsmVcBase.run_test(self)

        # Initiate a MO voice call
        if self._vc_type == "MO":

            # Dial using phone number
            self._logger.info("Calling %s ..." % self._phone_number)
            self._voicecall_api.dial(self._phone_number)

            # Check call status before callSetupTimeout (NS)
            self._vc_2g.check_call_connected(self._call_setup_time,
                                             blocking=False)

            # Check call status before callSetupTimeout (DUT)
            self._voicecall_api.wait_for_state(self._uecmd_types.VOICE_CALL_STATE.ACTIVE,  # pylint: disable=E1101
                                               self._call_setup_time)

        else:
            # Initiate a MT voice call
            self._vc_2g.mt_originate_call()
            # pylint: disable=E1101
            # Check call status is incoming before callSetupTimeout
            self._voicecall_api.wait_for_state(self._uecmd_types.VOICE_CALL_STATE.INCOMING,
                                               self._call_setup_time)

            # Answer incoming call
            self._voicecall_api.answer()

            # Check call status before callSetupTimeout (NS)
            self._vc_2g.check_call_connected(self._call_setup_time,
                                             blocking=False)

            # Check call status before callSetupTimeout (DUT)
            self._voicecall_api.wait_for_state(self._uecmd_types.VOICE_CALL_STATE.ACTIVE,
                                               self._call_setup_time)

        # Set Voice Call Volume at 100%
        self._system_api.adjust_specified_stream_volume("VoiceCall", 100)

        # Set source input for the recording
        self._psi_recorder.set_source_input(self._rec_source)

        # Start recording the voice call
        self._psi_recorder.start()

        # Step 1: Start detecting recording tone with no speech in UPV uplink audio path
        self._logger.info("Step 1: Start detecting recording tone with no speech in UPV uplink audio path")

        if(AudioUtil.detect_recording_tone(self._em.get_audio_analyzer("AUDIO_ANALYZER"),
                                                               self._time_between_tones,
                                                               self._tone_frequency,
                                                               self._logger)):
            self._logger.info("Recording tone detected on the UL signal on the UPV without any speech")

            # Send an audio signal which contains speech
            self._audio_analyzer.load_waveform(self._aa_speech_file_path)

            # Step 2: Start detecting recording tone with speech in UPV uplink audio path
            self._logger.info("Step 2: Start detecting recording tone with speech in UPV uplink audio path")

            if(AudioUtil.detect_recording_tone(self._em.get_audio_analyzer("AUDIO_ANALYZER"),
                                                                           self._time_between_tones,
                                                                           self._tone_frequency,
                                                                           self._logger)):
                self._logger.info("Recording tone detected on the UL signal on the UPV with speech")

                # Send an audio signal which contains silence only
                self._audio_analyzer.load_waveform(self._aa_ref_file_path)

                # Step 3: Start detecting recording tone with no speech in UPV uplink audio path
                self._logger.info("Step 3: Start detecting recording tone with no speech in UPV uplink audio path")

                if(AudioUtil.detect_recording_tone(self._em.get_audio_analyzer("AUDIO_ANALYZER"),
                                                               self._time_between_tones,
                                                               self._tone_frequency,
                                                               self._logger)):
                    self._logger.info("Recording tone detected on the UL signal on the UPV without any speech")

                    # Stop recording the voice call
                    self._psi_recorder.stop()

                    # Step 4: Check that no recording tones are generated in UL
                    self._logger.info("Step 4: Check that no recording tones are generated in UL")
                    if(not AudioUtil.detect_recording_tone(self._em.get_audio_analyzer("AUDIO_ANALYZER"),
                                                               self._time_between_tones,
                                                               self._tone_frequency,
                                                               self._logger)):
                        self._logger.info("No recording tone detected while recorder is stopped")
                        self._verdict_comment = "Recording tone successfully detected during voice call"
                        self._result_verdict = Global.SUCCESS
                    else:
                        self._logger.error("Recording tone detected while recorder is stopped")
                else:
                    self._logger.error("Recording tone not detected in the UL signal")
                    # Stop recording the voice call
                    self._psi_recorder.stop()
            else:
                self._logger.error("Recording tone not detected in the UL signal")
                # Stop recording the voice call
                self._psi_recorder.stop()
        else:
            self._logger.error("Recording tone not detected in the UL signal")
            # Stop recording the voice call
            self._psi_recorder.stop()

        # Release the call
        self._vc_2g.voice_call_network_release()

        try:
            # Check call is released (NS)
            self._vc_2g.check_call_idle(self._registration_timeout,
                                        blocking=False)

            # Check call is released (DUT)
            self._voicecall_api.wait_for_state(self._uecmd_types.VOICE_CALL_STATE.NOCALL,  # pylint: disable=E1101
                                               self._call_setup_time)

        except AcsBaseException as acs_exception:
            self._logger.error("Call release fail:" + str(acs_exception))

        return self._result_verdict, self._verdict_comment

#------------------------------------------------------------------------------
    def tear_down(self):

        # Call LabAudioVcBase Teardown function
        LabAudioGsmVcBase.tear_down(self)

        return Global.SUCCESS, "No errors"
