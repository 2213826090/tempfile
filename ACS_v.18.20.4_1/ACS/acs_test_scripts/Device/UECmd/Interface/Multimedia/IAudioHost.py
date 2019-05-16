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
:summary: This file implements Audio UECmds on host
:since: 09/10/2014
:author: vdechefd
"""
from ErrorHandling.DeviceException import DeviceException

# pylint: disable=W0613


class IAudioHost():

    """
    Abstract class that defines the interface to be implemented
    by sub classes handling audio playback on host side.

    All method that shall be redefined in sub-classes raise a
    I{DeviceException} error.
    """

    def __init__(self, device):
        """
        Initializes this instance.

        Nothing to be done in abstract class.
        """
        pass

    def pc_audio_playback_start(self, input_file, output_device_index=None):
        """
        Start playing the audio on PC

        :type input_file: str
        :param input_file: which file used to play sound
        :type output_device_index: int
        :param output_device_index: which device used to play sound

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def pc_audio_playback_stop(self):
        """
        Stop playing the audio on PC

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)
