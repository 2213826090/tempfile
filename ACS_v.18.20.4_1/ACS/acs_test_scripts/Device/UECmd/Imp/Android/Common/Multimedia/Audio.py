"""
:copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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
:summary: This file implements UECmds for audio playback
:since: 08/10/2014
:author: vdechefd
"""
from acs_test_scripts.Device.UECmd.Interface.Multimedia.IAudio import IAudio
from acs_test_scripts.Device.UECmd.Imp.Android.Common.BaseV2 import BaseV2
from UtilitiesFWK.Utilities import Global
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException
import os


class Audio(BaseV2, IAudio):

    """
    Class that handle all audio operations
    """

    def __init__(self, device):
        """
        Constructor
        """
        BaseV2.__init__(self, device)
        IAudio.__init__(self, device)

        self._target_audio_class = 'acscmd.multimedia.AudioModule'
        self._target_recorder_class = 'acscmd.audio.AudioRecorderModule'
        self._multimedia_path = device.multimedia_path

    def play(self, audio_file_path, loop=None):
        """
        Starts or resumes a playback.

        :type audio_file_path: str
        :param audio_file_path: can be an absolute path to a file on the device (such as /sdcard/myfile.mp3), or a
                                url to a distant file, in http or rtsp protocol (such as
                                http://myserver.com/myfile.mp3). If it is a distant file, the call will be blocking
                                until data is ready. When using rtsp, loop parameter won't have any effect
                                (e.g. playback won't loop)
        :type loop: bool
        :param loop: If True, audio playback will loop until stop() is called.
                     If False, playback will stop once file's end is reached.
        :rtype: str
        :return: output log
        """
        self._logger.info("Start audio playback on %s", str(audio_file_path))

        # Check if file is a full path
        if os.path.dirname(audio_file_path) == "":
            audio_file_path = "%s%s" % (self._multimedia_path, str(audio_file_path))

        # Build the adb command to launch video playback
        target_method = "startAudioPlayback"
        args = " --es filename \"%s\"" % str(audio_file_path)
        args += " --ez loop %s" % ("true" if loop else "false")

        # execute command on device
        output = self._internal_exec_v2(self._target_audio_class, target_method, args)

        return output[Audio.OUTPUT_MARKER]

    def stop(self):
        """
        Stop the audio file playback.
        This is not meant to work with play_native().

        :rtype: float
        :return: playback duration in seconds.
        """
        self._logger.info("Stop audio playback")

        # Build the adb command to stop audio playback
        target_method = "stopAudioPlayback"

        # execute command on device
        output = self._internal_exec_v2(self._target_audio_class, target_method)

        duration = int(output["duration"]) / 1000.0
        return duration

    def is_playing(self):
        """
        Check if the audio playback is running.
        This is not meant to work with play_native().

        :rtype: tuple
        :return: A tuple containing audio playback state as a bool, and playback duration in milliseconds.
                 If audio is playing, it is the current playback's duration. If not, it is the duration of last
                 playback (or 0 if no last playback).
        """
        self._logger.info("Check audio playback state")

        # Build the adb command to check audio playback
        target_method = "isPlaying"

        # execute command on device
        output = self._internal_exec_v2(self._target_audio_class, target_method)

        is_playing = (output["is_playing"] == "true")
        duration = int(output["duration"])

        return is_playing, duration

    def is_complete(self, length, deviation):
        """
        Check if the playback is complete.

        :type length: int
        :param length: length of file which is played, in milliseconds

        :type deviation: int
        :param deviation: error between expected time and actual time, in milliseconds

        :rtype: str
        :return: function execution output
        """
        try:
            length = int(length)
            deviation = int(deviation)
        except ValueError:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "Duration and/or deviation isn't integer! "
                                     "duration={0} deviation={1}".format(str(length), str(deviation)))

        playing, duration = self.is_playing()

        if playing:
            # playback did not stop
            raise DeviceException(DeviceException.OPERATION_FAILED, "Audio is still playing: it should have stop.")
        elif duration == 0:
            # playback did not start
            raise DeviceException(DeviceException.OPERATION_FAILED, "No audio playback has been launched.")
        elif abs(length - duration) > deviation:
            # playback stopped, but playback duration is wrong
            raise DeviceException(DeviceException.OPERATION_FAILED,
                                  "Play duration mismatch, total playtime is  %s ms" % duration)
        else:
            # playback went fine
            return "Total playtime is %s ms" % duration

    def play_native(self, audio_file_path):
        """
        Starts an audio playback, using platform native player.

        :type audio_file_path: str
        :param audio_file_path: an absolute path to a file on the device (such as /sdcard/myfile.mp3)

        :rtype: str
        :return: output log
        """

        #Play the audio:
        cmd = "adb shell am start -a android.intent.action.VIEW -d \"file://%s\" -t \"audio/*\"" % audio_file_path
        self._exec(cmd, 5)

        return int(Global.SUCCESS)

    def stop_native(self):
        """
        Stop the audio file playback.
        This is not meant to work with play_native().

        :rtype: float
        :return: playback duration in seconds.
        """
        return self._exec("adb shell am force-stop com.google.android.music")
