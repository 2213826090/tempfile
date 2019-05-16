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
:summary: This file is the process of LiveAudioRecordMic
:since: 28/07/2011
:author: szhen11
"""

import time
import os.path
from acs_test_scripts.UseCase.Multimedia.LIVE_AUDIO_RECORD_BASE import LiveAudioRecordBase
from UtilitiesFWK.Utilities import Global
from ErrorHandling.DeviceException import DeviceException


class LiveAudioRecordMic(LiveAudioRecordBase):

    """
    Class Live Audio Record with mic source.
    """

    def __init__(self, tc_conf, global_config):
        """
        Constructor
        """
        # Call base Init function
        LiveAudioRecordBase.__init__(self, tc_conf, global_config)

        self._source = "mic"

        self._silent_mode = None
        self._original_ringtone_mode = None
        self._original_vibrate_mode = None
        # if there is not such parameter, get_param_value() will return None,
        # if there is such parameter, but not given a value, get_param_value() will return ''
        if self._tc_parameters.get_param_value("SILENT_MODE") not in (None, ''):
            self._silent_mode = str(self._tc_parameters.get_param_value("SILENT_MODE")).lower()

        self._host_play = False
        self._host_play_file = None
        self._host_play_device = None
        if self._tc_parameters.get_param_value("HOST_PLAY") not in (None, ''):
            self._host_play = ((self._tc_parameters.get_param_value("HOST_PLAY")).lower() == "true")

        if self._host_play:
            self._host_play_file = self._tc_parameters.get_param_value("HOST_PLAY_FILE")
            if not os.path.isabs(self._host_play_file):
                # if not a asb path, use the file in _Embedded/MEDIA
                self._host_play_file = os.path.join(self._device._acs_embedded_path,  # pylint: disable=W0212
                                                    "MEDIA",
                                                    self._host_play_file)

            if self._tc_parameters.get_param_value("HOST_PLAY_DEVICE") not in (None, '', 'default'):
                self._host_play_device = int(self._tc_parameters.get_param_value("HOST_PLAY_DEVICE"))

        self._audio_file = None
        self._record_directory = None

        # Get UECmdLayer
        self._phone_system_api = self._device.get_uecmd("PhoneSystem")

        self._audio_host_api = self._device.get_uecmd("AudioHost")
        self._audio_api = self._device.get_uecmd("Audio")
        self._audio_recorder_api = self._device.get_uecmd("AudioRecorder")
        self._audio_recorder_host_api = self._device.get_uecmd("AudioRecorderHost")
        self._system_api = self._device.get_uecmd("System")

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        # Call base Setup function
        LiveAudioRecordBase.set_up(self)

        # Create folder for pull record file to phone
        directory_name = os.path.join("DUTRec", self._name)
        self._record_directory = self._phone_system_api.create_subfolder(directory_name)

        if self._silent_mode not in (None, "none"):
            # get original silent mode and vibrate mode
            original_silent_vibrate = self._phone_system_api.get_silent_vibrate()
            self._original_ringtone_mode = original_silent_vibrate.split('+')[0]
            self._original_vibrate_mode = original_silent_vibrate.split('+')[1]

            self._phone_system_api.set_silent_vibrate(self._silent_mode,
                                                      self._original_vibrate_mode)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        # Call base Run function
        LiveAudioRecordBase.run_test(self)

        # start DUT audio recording
        self._audio_file = self._audio_recorder_api.start_audio_record(self._source,
                                                                       self._codec,
                                                                       self._container,
                                                                       self._bitrate,
                                                                       self._samplerate,
                                                                       self._channelnum,
                                                                       self._multimedia_path,
                                                                       self._duration)
        # host play a sound for DUT recording.
        if self._host_play:
            self._audio_host_api.pc_audio_playback_start(self._host_play_file, self._host_play_device)

        self._logger.info("Waiting for record stop...")
        time.sleep(self._duration / 1000 + self._wait_btwn_cmd)

        # host stop playing sound for DUT recording.
        if self._host_play:
            self._audio_host_api.pc_audio_playback_stop()

        # stop DUT recording, and check record time
        actual_duration = self._audio_recorder_api.stop_audio_record()
        self._logger.info("actual record Duration is %d, %d, %d" % (actual_duration, self._duration, self._deviation))

        # check if file exist
        self._phone_system_api.check_file_exist(self._audio_file)

        # Pull record file to phone
        (_dut_rcf_root, dut_rcf_ext) = os.path.splitext(self._audio_file)
        dut_rcf_host_path = os.path.join(self._record_directory, time.strftime("%d_%Hh%M.%S") + dut_rcf_ext)
        self._logger.info("pull record file to: " + dut_rcf_host_path)
        # the timeout value isn't accurate. use record duration value,
        # just because the transmission time increase by the duration
        self._phone_system_api.pull(self._audio_file,
                                    dut_rcf_host_path,
                                    self._duration / 1000 + 10)

        # check the duration after pull, otherwise record file won't be stored on host if duration mismatch
        if actual_duration < self._duration - self._deviation    \
           or actual_duration > self._duration + self._deviation:
            raise DeviceException(DeviceException.DEFAULT_ERROR_CODE,
                                  "actual record Duration mismatch %d" % actual_duration)

        if self._play:
            # play the recorded audio file, and check if it is played completed
            self._system_api.adjust_specified_stream_volume("Media", self._volume)
            self._audio_api.play(self._audio_file)
            time.sleep(self._duration / 1000 + self._wait_btwn_cmd)

            self._logger.info("Check if audio is completed...")
            # use actual Duration got from agent to check complete
            result = self._audio_api.is_complete(actual_duration, self._deviation)

            self._audio_api.stop()
        else:
            result = "Do not play the record file to check record quality"

        result = "record time:%dms, actual record time:%dms, record file:%s" % \
                 (self._duration, actual_duration, self._audio_file) + result
        return Global.SUCCESS, result

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """
        # Call use case base tear_down function
        LiveAudioRecordBase.tear_down(self)

        # quit DUT recorder
        self._audio_recorder_api.quit_audio_record()

        if self._host_play:
            self._audio_host_api.pc_audio_playback_stop()

        if self._play:
            self._audio_api.stop()

        if self._silent_mode not in (None, "none"):
            self._phone_system_api.set_silent_vibrate(self._original_ringtone_mode, self._original_vibrate_mode)

        return Global.SUCCESS, "No errors"
