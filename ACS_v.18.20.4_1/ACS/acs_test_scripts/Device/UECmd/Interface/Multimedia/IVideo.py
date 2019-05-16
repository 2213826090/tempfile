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
:summary: This file implements Video UECmds
:since: 08/10/2014
:author: vdechefd
"""
from ErrorHandling.DeviceException import DeviceException

# pylint: disable=W0613


class IVideo():

    """
    Abstract class that defines the interface to be implemented by video handling sub classes.

    All method that shall be redefined in sub-classes raise a
    I{DeviceException} error.
    """

    def __init__(self, device):
        """
        Initializes this instance.

        Nothing to be done in abstract class.
        """
        pass

    def play(self, filename, loop, screen_orientation=None):
        """
        Launch the video file playback.

        :type filename: str
        :param filename: can be an absolute path to a file on the device (such as /sdcard/myfile.avi), or a url to a
                         distant file, in http or rtsp protocol (such as http://myserver.com/myfile.mp4).
                         If it is a distant file, the call will be blocking until data is ready.
                         When using rtsp, loop parameter won't have any effect (e.g. playback won't loop)

        :type loop: bool
        :param loop: If True, video playback will loop until stop() is called.
                       If False, playback will stop once file's end is reached.

        :type screen_orientation: str
        :param screen_orientation: Possible values are portrait, landscape, reverse_portrait, reverse_landscape.
                                   If none, default device screen orientation will be used.

        :rtype: str
        :return: output log
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def stop(self):
        """
        Stop the video file playback.
        This is not meant to work with play_native().

        :rtype: float
        :return: playback duration in seconds.
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def is_playing(self):
        """
        Check if the video playback is running.
        This is not meant to work with play_native().

        :rtype: tuple
        :return: A tuple containing video playback state as a bool, and playback duration in seconds as a float.
                 If video is playing, it is the current playback's duration. If not, it is the duration of last
                 playback (or 0 if no last playback).
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def play_native(self, filename, delay_time_s=0):
        """
        Launch the video file playback. This

        :type filename: str
        :param filename: can be an absolute path to a file on the device (such as /sdcard/myfile.avi), or a url to a
                         distant file, in http or rtsp protocol (such as http://myserver.com/myfile.mp4).
                         If it is a distant file, the call will be blocking until data is ready.

        :rtype: str
        :return: output log
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def enable_post_processing_video_effect(self, wait_btwn_cmd, library_image_path, on_off_image):
        """
        Enable post processing video effect
        :type wait_btwn_cmd: int
        :param wait_btwn_cmd: time between command
        :type library_image_path: str
        :param library_image_path: path to the image library on host.
        :type on_off_image: dict
        :param on_off_image: Dictionary with two Key (ON : name of On image, OFF : nome of Off image)
        :rtype: None
        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def disable_post_processing_video_effect(self, wait_btwn_cmd, library_image_path, on_off_image):
        """
        Disable post processing video effect
        :type wait_btwn_cmd: int
        :param wait_btwn_cmd: time between command
        :type library_image_path: str
        :param library_image_path: path to the image library on host.
        :type on_off_image: dict
        :param on_off_image: Dictionary with two Key (ON : name of On image, OFF : nome of Off image)
        :rtype: None
        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)
