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
:summary: This file is the process of LiveAudioPlayback
:since: 08/04/2011
:author: fhu2
"""

import time
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.Multimedia.LIVE_MUM_BASE import LiveMuMBase


class LiveAudioPlayback(LiveMuMBase):

    """
    Class Lab Audio playback.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call UseCase base Init function
        LiveMuMBase.__init__(self, tc_name, global_config)
        # Get path to multimedia files
        self._multimedia_path = self._device.multimedia_path
        # Get TC Parameters
        self._audio_file = self._tc_parameters.get_param_value("AUDIO_FILE")
        self._volume = int(self._tc_parameters.get_param_value("VOLUME"))
        self._length = int(self._tc_parameters.get_param_value("LENGTH"))
        self._deviation_rate = int(self._tc_parameters.get_param_value("DEVIATION_RATE"))
        self._audio_output = str(self._tc_parameters.get_param_value("AUDIO_OUTPUT")).lower()

        self._original_ringtone_mode = None
        self._original_vibrate_mode = None
        self._silent_mode = None
        if self._tc_parameters.get_param_value("SILENT_MODE") not in (None, ''):
            self._silent_mode = str(self._tc_parameters.get_param_value("SILENT_MODE")).lower()

        self._host_record = False
        self._host_record_file = None
        self._host_record_device = None
        if self._tc_parameters.get_param_value("RECORD") not in (None, ''):
            self._host_record = ((self._tc_parameters.get_param_value("RECORD")).lower() == "true")

        if self._host_record:
            # Generate the audio record name for each playback, self._host_record_file
            self._host_record_file = self.generate_audio_record_name()
            if self._tc_parameters.get_param_value("HOST_RECORD_DEVICE") not in (None, '', 'default'):
                self._host_record_device = int(self._tc_parameters.get_param_value("HOST_RECORD_DEVICE"))

        self._deviation = int(self._length) * int(self._deviation_rate) / 100
        self._origianl_audio_output = None

        # Get UECmdLayer
        self._audio_api = self._device.get_uecmd("Audio")
        self._audio_recorder_api = self._device.get_uecmd("AudioRecorderHost")
        self._phone_system_api = self._device.get_uecmd("PhoneSystem")
        self._system_api = self._device.get_uecmd("System")

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        # Call UseCase base Setup function
        LiveMuMBase.set_up(self)

        if self._silent_mode not in (None, "none"):
            # get original silent mode and vibrate mode
            original_silent_vibrate = self._phone_system_api.get_silent_vibrate()
            self._original_ringtone_mode = original_silent_vibrate.split('+')[0]
            self._original_vibrate_mode = original_silent_vibrate.split('+')[1]
            self._phone_system_api.set_silent_vibrate(self._silent_mode,
                                                      self._original_vibrate_mode)

        if self._audio_output not in (None, "none"):
            self._logger.info("Switching the audio output")
            # get original audio output
            self._origianl_audio_output = self._phone_system_api.get_audio_output()
            self._phone_system_api.switch_audio_output(self._audio_output)

        time.sleep(self._wait_btwn_cmd)

        return self._error.Code, "No errors"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        # Call UseCase base Run function
        LiveMuMBase.run_test(self)

        time.sleep(self._wait_btwn_cmd)

        self._system_api.adjust_specified_stream_volume("Media", self._volume)

        if self._host_record:
            self._audio_recorder_api.pc_audio_record_start(self._host_record_file, self._host_record_device)

        time.sleep(self._wait_btwn_cmd)
        self._audio_api.play(self._multimedia_path + self._audio_file)

        self._logger.info("Waiting for the audio to complete...")
        time.sleep(int(self._length / 1000) + self._wait_btwn_cmd)

        if self._host_record:
            self._audio_recorder_api.pc_audio_record_stop()
            self._logger.info("audio record saved at " + self._host_record_file)

        time.sleep(self._wait_btwn_cmd)
        self._logger.info("Check if audio is completed...")
        result = self._audio_api.is_complete(self._length, self._deviation)

        self._audio_api.stop()

        result_msg = result

        return Global.SUCCESS, result_msg

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """
        # Call use case base tear_down function
        LiveMuMBase.tear_down(self)

        time.sleep(self._wait_btwn_cmd)

        self._audio_api.stop()

        if self._host_record:
            # Call pc_audio_record_stop in tear_down, to make sure UC release system resource
            self._audio_recorder_api.pc_audio_record_stop()

        if self._audio_output not in (None, "none"):
            self._logger.info("Switching the audio output back")
            self._phone_system_api.switch_audio_output(self._origianl_audio_output)

        if self._silent_mode not in (None, "none"):
            self._phone_system_api.set_silent_vibrate(self._original_ringtone_mode,
                                                      self._original_vibrate_mode)

        return self._error.Code, "No errors"
