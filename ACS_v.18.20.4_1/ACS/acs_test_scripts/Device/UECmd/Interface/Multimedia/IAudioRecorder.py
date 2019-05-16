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
from ErrorHandling.DeviceException import DeviceException

# pylint: disable=W0613


class IAudioRecorder():

    """
    Abstract class that defines the interface to be implemented
    by multimedia audio handling sub classes.

    All method that shall be redefined in sub-classes raise a
    I{DeviceException} error.
    """

    def __init__(self, device):
        """
        Initializes this instance.

        Nothing to be done in abstract class.
        """
        pass

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
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def stop_audio_record(self):
        """
        Stop recording audio

        :rtype: int
        :return: actual record duration
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def quit_audio_record(self):
        """
        Quit audio record

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)
