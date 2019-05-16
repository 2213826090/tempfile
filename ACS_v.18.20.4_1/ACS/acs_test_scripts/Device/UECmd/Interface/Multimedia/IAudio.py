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
:summary: This file implements Audio UECmds
:since: 08/10/2014
:author: vdechefd
"""
from ErrorHandling.DeviceException import DeviceException

# pylint: disable=W0613


class IAudio():

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
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def stop(self):
        """
        Stop the audio file playback.
        This is not meant to work with play_native().

        :rtype: float
        :return: playback duration in seconds.
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def is_playing(self):
        """
        Check if the audio playback is running.
        This is not meant to work with play_native().

        :rtype: tuple
        :return: A tuple containing audio playback state as a bool, and playback duration in milliseconds.
                 If audio is playing, it is the current playback's duration. If not, it is the duration of last
                 playback (or 0 if no last playback).
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

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
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def play_native(self, audio_file_path):
        """
        Starts an audio playback, using platform native player.

        :type audio_file_path: str
        :param audio_file_path: an absolute path to a file on the device (such as /sdcard/myfile.mp3)

        :rtype: str
        :return: output log
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def stop_native(self):
        """
        Stop the audio file playback with native player
        This is not meant to work with play().

        :rtype: float
        :return: playback duration in seconds.
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)
