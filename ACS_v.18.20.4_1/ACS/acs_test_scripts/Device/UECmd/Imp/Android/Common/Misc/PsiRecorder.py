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
:summary: This file implements the Miscellaneous UEcmd for Android phone
:since: 05/08/2013
:author: fbelvezx
"""

from acs_test_scripts.Device.UECmd.Imp.Android.Common.BaseV2 import BaseV2


class PsiRecorder(BaseV2):

    """
    Class that handle audio operations regarding PSI recorder
    """

    def __init__(self, device):
        """
        Constructor
        """
        BaseV2.__init__(self, device)

        # related to PSI recorder
        self.psi_recorder_audio_component = "com.intel.psi_recorder/com.intel.psi_recorder.PSI_Recorder"
        self.psi_recorder_command_receiver = "com.intel.psi_recorder.PSI_CommandReceiver"

    def preset(self, codec, sampling_rate, channel, bit_rate, container):
        """
        Performs a predefined setting for PSI_Recorder

        :type codec: str
        :param codec: audio codec to be used for the recording

        :type sampling_rate: str
        :param sampling_rate: sampling rate to be used for the recording

        :type channel: str
        :param channel: Type of channel to be use for the recording (Mono/Stereo)

        :type bit_rate: str
        :param bit_rate: bitrate to be used for the recording

        :type container: str
        :param container: container to be used for the recording (3gp/3ga/aac atc)
        """

        # Launch PSI Recorder
        cmd = "adb shell am start -n %s" % self.psi_recorder_audio_component
        self._logger.info("Launch PSI Recorder")
        self._exec(cmd)

        # Set recorder audio codec
        cmd = "adb shell am broadcast -a %s.CHANGE_SETTINGS --es CHANGE_SETTINGS.option codec --es CHANGE_SETTINGS.value %s" \
                % (self.psi_recorder_command_receiver, codec)
        self._exec(cmd)

        # Set recorder sampling_rate
        cmd = "adb shell am broadcast -a %s.CHANGE_SETTINGS --es CHANGE_SETTINGS.option sampling_rate --es CHANGE_SETTINGS.value %s" \
                % (self.psi_recorder_command_receiver, sampling_rate)
        self._exec(cmd)

        # Set recorder channel
        cmd = "adb shell am broadcast -a %s.CHANGE_SETTINGS --es CHANGE_SETTINGS.option channel --es CHANGE_SETTINGS.value %s" \
                % (self.psi_recorder_command_receiver, channel)
        self._exec(cmd)

        # Set recorder bit_rate
        cmd = "adb shell am broadcast -a %s.CHANGE_SETTINGS --es CHANGE_SETTINGS.option bit_rate --es CHANGE_SETTINGS.value %s" \
                % (self.psi_recorder_command_receiver, bit_rate)
        self._exec(cmd)

        # Set recorder container
        cmd = "adb shell am broadcast -a %s.CHANGE_SETTINGS --es CHANGE_SETTINGS.option container --es CHANGE_SETTINGS.value %s" \
                % (self.psi_recorder_command_receiver, container)
        self._exec(cmd)

    def set_source_input(self, source_input):
        """
        Sets the source input for the recording

        :type source_input: str
        :param source_input: source input for the recording

        """
        # Set recorder source_input
        cmd = "adb shell am broadcast -a %s.CHANGE_SETTINGS --es CHANGE_SETTINGS.option src --es CHANGE_SETTINGS.value %s" \
                % (self.psi_recorder_command_receiver, source_input)
        self._exec(cmd)

    def start(self):
        """
        Start a recording

        """

        cmd = "adb shell am broadcast -a %s.START_RECORD" % self.psi_recorder_command_receiver
        self._logger.info("Start Recording")
        self._exec(cmd)

    def stop(self):
        """
        Stops a recording

        """
        cmd = "adb shell am broadcast -a %s.STOP_RECORD" % self.psi_recorder_command_receiver
        self._logger.info("Stop Recording")
        self._exec(cmd)

    def play(self):
        """
        Playback of a recording

        """
        cmd = "adb shell am broadcast -a %s.PLAY_RECORD" % self.psi_recorder_command_receiver
        self._exec(cmd)
