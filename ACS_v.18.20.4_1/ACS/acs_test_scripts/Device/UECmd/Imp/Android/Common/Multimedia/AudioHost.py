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
:summary: This file implements UECmds for audio playback on host side
:since: 09/10/2014
:author: vdechefd
DEPRECATED: Please discontinue its use.  The code is copied to 
Utilities/AudioUtilities.py, which is a better home since it is not operating 
on a device under test.
"""
from acs_test_scripts.Device.UECmd.Interface.Multimedia.IAudioHost import IAudioHost
from acs_test_scripts.Device.UECmd.Imp.Android.Common.BaseV2 import BaseV2
import threading
import pyaudio
import wave
import os.path
from ErrorHandling.AcsConfigException import AcsConfigException


class AudioHost(BaseV2, IAudioHost):

    """
    Class that handle audio playback on host side.
    """

    def __init__(self, device):
        """
        Constructor
        """
        BaseV2.__init__(self, device)
        IAudioHost.__init__(self, device)

        self.pc_hostplayer = None

    def pc_audio_playback_start(self, input_file, output_device_index=None):
        """
        Start playing the audio on PC

        :type input_file: str
        :param input_file: which file used to play sound
        :type output_device_index: int
        :param output_device_index: which device used to play sound

        :return: None
        """
        if self.pc_hostplayer is not None:
            self._logger.info("PC playback already in progress. Stop previous playback on pc...")
            self.pc_hostplayer.stop_hostplay()
            self.pc_hostplayer = None

        self._logger.info("Start playback on PC ...")
        if not os.path.exists(input_file):
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "PC playback file doesn't exist. %s" % input_file)
        # todo, check output_device_index valid
        self._logger.info("Host play file: %s" % input_file)
        if output_device_index is not None:
            self._logger.info("Host play devices: %d" % output_device_index)
        else:
            self._logger.info("Host play devices: default")
        self.pc_hostplayer = HostPlayer(input_file, output_device_index)
        self.pc_hostplayer.start_hostplay()

    def pc_audio_playback_stop(self):
        """
        Stop playing the audio on PC

        :return: None
        """
        if self.pc_hostplayer is None:
            self._logger.info("PC playback already stopped")
        else:
            self._logger.info("Stop playback on PC ...")
            self.pc_hostplayer.stop_hostplay()
            self.pc_hostplayer = None


class HostPlayer():

    """
    HostPlay, play audio on pc
    """

    def __init__(self, input_file_name, output_device_index=None):
        """
        Constructor
        """
        self._hostplay_event = threading.Event()
        self._input_file_name = input_file_name
        self._output_device_index = output_device_index

    def start_hostplay(self):
        self._hostplay_event.set()

        playback_handler = HostPlayHandler(self._hostplay_event,
                                           self._input_file_name,
                                           self._output_device_index)
        playback_handler.start()

    def stop_hostplay(self):
        self._hostplay_event.clear()


class HostPlayHandler(threading.Thread):

    """
    HostPlayHandler, handles the audio playback on pc
    """

    def __init__(self, hostplay_event, input_file_name, output_device_index=None):
        threading.Thread.__init__(self,)

        self._hostplay_event = hostplay_event
        self._input_file_name = input_file_name
        self._output_device_index = output_device_index
        self._chunk_size = 1024

    def run(self):
        # global playback_event
        pa = pyaudio.PyAudio()
        wave_file = wave.open(self._input_file_name, 'rb')

        # prepare the out stream
        pa_stream = pa.open(format=pa.get_format_from_width(wave_file.getsampwidth()),
                            channels=wave_file.getnchannels(),
                            rate=wave_file.getframerate(),
                            output=True,
                            output_device_index=self._output_device_index)

        # start playback
        data = wave_file.readframes(self._chunk_size)
        while self._hostplay_event.isSet() and data != '':
            pa_stream.write(data)
            data = wave_file.readframes(self._chunk_size)

        # close the input stream
        wave_file.close()

        # close the output stream
        pa_stream.stop_stream()
        pa_stream.close()
        pa.terminate()
