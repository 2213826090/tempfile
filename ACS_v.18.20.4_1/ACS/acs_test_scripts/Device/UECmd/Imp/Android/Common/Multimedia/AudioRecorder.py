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
:summary: This file implements UECmds for audio recording on DUT
:since: 08/10/2014
:author: vdechefd
"""
from acs_test_scripts.Device.UECmd.Interface.Multimedia.IAudioRecorder import IAudioRecorder
from acs_test_scripts.Device.UECmd.Imp.Android.Common.BaseV2 import BaseV2
import time


class AudioRecorder(BaseV2, IAudioRecorder):

    """
    Class that handle all audio recording operations
    """

    def __init__(self, device):
        """
        Constructor
        """
        BaseV2.__init__(self, device)
        IAudioRecorder.__init__(self, device)

        self._target_recorder_class = 'acscmd.audio.AudioRecorderModule'
        self._multimedia_path = device.multimedia_path

    def start_audio_record(self, source, codec, container, bitrate, samplerate, channelnum, recordpath, duration):
        """
        Start recording audio.

        :type source: str
        :param source: source of the audio
        :type codec: str
        :param codec: recorded audio codec
        :type container: str
        :param container: recorded audio container
        :type bitrate: int
        :param bitrate: recorded audio bitrate, bps
        :type samplerate: int
        :param samplerate: recorded audio samplerate, hz
        :type channelnum: int
        :param channelnum: recorded audio channelnum
        :type recordpath: str
        :param recordpath: path to store record audio file
        :type duration: int
        :param duration: record audio duration

        :rtype: str
        :return: recorded audio file name
        """
        self._logger.info("Start to record audio...")
        target_method = "startAudioRecord"
        timestamp = time.strftime("%d_%H_%M_%S")
        cmd = '--es source %s --es codec %s --es container %s --es bitrate %s --es samplerate %s ' % (source,
                                                                                                      codec,
                                                                                                      container,
                                                                                                      bitrate,
                                                                                                      samplerate)

        cmd += '--es channelnum %s --es record_file_path %s --es timestamp %s --es duration %s' % (channelnum,
                                                                                                   recordpath,
                                                                                                   timestamp,
                                                                                                   duration)
        result = self._internal_exec_v2(self._target_recorder_class, target_method, cmd)
        recorded_filename = result.get("recordFilePath")
        self._logger.info("record file, %r", recorded_filename)
        return recorded_filename

    def stop_audio_record(self):
        """
        Stop recording audio

        :rtype: int
        :return: actual record duration
        """
        self._logger.info("Stop audio recording...")
        target_method = "stopAudioRecord"
        result = self._internal_exec_v2(self._target_recorder_class, target_method)
        return int(result["actualDuration"])

    def quit_audio_record(self):
        """
        Quit audio record

        :return: None
        """
        self._logger.info("Quit audio record")
        target_method = "quitAudioRecord"
        result = self._internal_exec_v2(self._target_recorder_class, target_method)
        self._logger.info('Agent Response({status}): {output}'.format(**result))
        return result['status']
