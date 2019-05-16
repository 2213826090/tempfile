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
:summary: This script implements the interface of system uecmd.
:since: 08/24/2010
:author: wchen61
"""
from ErrorHandling.DeviceException import DeviceException


class ISleepMode:
    """
    SleepMode interface
    """

    def __init__(self, device):
        """
        Constructor
        """
        pass

    def init(self, _mode, _settle_time=0, _audio_file=None,
             _audio_volume=10, _music_player=None):
        """
        Sleep mode initialization on the device

        :type mode: str
        :param mode: Requested mode to enter

        :type audio_file: str
        :param audio_file: The path of the audio file on the board

        :type settle_time: int
        :param settle_time: Time to wait before sleep mode
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def clear(self):
        """
        Clear sleep mode on the device
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def clear_residencies(self):
        """
        Clear residencies on the device
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_residencies(self):
        """
        Get the residencies

        :rtype: list
        :return: a list of dictionary of residency values
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def device_has_residencies(self):
        """
        To know if the device has a residency file

        :rtype: bool
        :return: true if the device has a residency file, else false
        """
        return True
