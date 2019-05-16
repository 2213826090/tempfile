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
:summary: This file implements UECmds for audio recording on host
:since: 09/10/2014
:author: vdechefd
"""
from acs_test_scripts.Device.UECmd.Interface.Multimedia.IAudioRecorderHost import IAudioRecorderHost
from acs_test_scripts.Device.UECmd.Imp.Android.Common.BaseV2 import BaseV2
import pyaudio
import wave
import threading


class AudioRecorderHost(BaseV2, IAudioRecorderHost):

    """
    Class that handle all audio recording operations
    """

    def __init__(self, device):
        """
        Constructor
        """
        BaseV2.__init__(self, device)
        IAudioRecorderHost.__init__(self, device)

        self.pc_recorder = None

    def pc_audio_record_start(self, output_file, input_device_index=None):
        """
        Start recording the audio on PC

        :type output_file: str
        :param output_file: which file used to save the recorded sound
        :type input_device_index: int
        :param input_device_index: which device used to record sound

        :return: None
        """
        if self.pc_recorder is not None:
            self._logger.warning("Recording already in progress. Stop previous recording on pc!")
            self.pc_recorder.stop_record()
            self.pc_recorder = None

        self._logger.info("Start recording on PC ...")
        if input_device_index is not None:
            self._logger.info("Host record devices: %d" % input_device_index)
        else:
            self._logger.info("Host record devices: default")
        self.pc_recorder = Recorder(output_file, pyaudio.paInt16, 1, 44100, input_device_index)
        self.pc_recorder.start_record()

    def pc_audio_record_stop(self):
        """
        Stop recording the audio on PC

        :return: None
        """
        if self.pc_recorder is None:
            self._logger.info("PC recorder already stopped")
        else:
            self._logger.info("Stop recording on PC ...")
            self.pc_recorder.stop_record()
            self.pc_recorder = None


class Recorder():

    """
    Recorder, record audio on pc
    """

    def __init__(self, output_file_name, file_format, channels, rate, input_device_index=None):
        """
        Constructor
        """
        self._recording_event = threading.Event()
        self._output_file_name = output_file_name
        self._format = file_format
        self._channels = channels
        self._rate = rate
        self._input_device_index = input_device_index

    def start_record(self):
        self._recording_event.set()

        record_handler = RecordHandler(self._recording_event,
                                       self._output_file_name,
                                       self._format,
                                       self._channels,
                                       self._rate,
                                       self._input_device_index)
        record_handler.start()

    def stop_record(self):
        self._recording_event.clear()


class RecordHandler(threading.Thread):

    """
    RecordHandler, handles the audio recording on pc
    """

    def __init__(self, recording_event, output_file_name,
                 file_format, channels, rate, input_device_index=None):
        threading.Thread.__init__(self,)

        self._recording_event = recording_event
        self._output_file_name = output_file_name
        self._format = file_format
        self._channels = channels
        self._rate = rate
        self._input_device_index = input_device_index
        self._chunk_size = 1024

    def run(self):
        # global recording_event
        pa = pyaudio.PyAudio()

        # prepare the input stream
        pa_stream = pa.open(format=self._format,
                            channels=self._channels,
                            rate=self._rate,
                            input=True,
                            input_device_index=self._input_device_index,
                            frames_per_buffer=self._chunk_size)

        # prepare the output stream
        output = wave.open(self._output_file_name, 'wb')
        output.setnchannels(self._channels)
        output.setsampwidth(pa.get_sample_size(self._format))
        output.setframerate(self._rate)

        # start recording
        while self._recording_event.isSet():
            data = pa_stream.read(self._chunk_size)
            output.writeframes(data)

        # close the input stream
        pa_stream.close()
        pa.terminate()

        # close the output stream
        output.close()
