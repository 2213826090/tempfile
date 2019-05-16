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
:summary: This file implements the System UEcmd for Android device
:since: 19/06/2013
:author: pbluniex
"""
from ErrorHandling.DeviceException import DeviceException

class IKeyEvent():

    """
    Abstract class that defines the interface to be implemented
    by device system operations handling sub classes.

    All method that shall be redefined in sub-classes raise a
    I{DeviceException} error.
    """

    def __init__(self, device):
        """
        Initializes this instance.

        Nothing to be done in abstract class.
        """
        pass

    def tap_on_screen(self, loc_x, loc_y):
        """
        Tap on screen
        :type loc_x: int
        :param loc_x: coordinate on the x axis
        :type loc_y: int
        :param loc_y: coordinate on the x axis
        :rtype: None
        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def swipe(self, local_x, local_y, timeout, local_swipe_x=0, local_swipe_y=0):
        """
        Do a swipe on screen.
        :type local_x: int
        :param local_x: x coordinate in pixel
        :type local_y: int
        :param local_y: y coordinate in pixel
        :type timeout: int
        :param timeout: timeout
        :type local_swipe_x: int
        :param local_swipe_x: coordinate in pixel of target x
        :type local_swipe_y: int
        :param local_swipe_y: coordinate in pixel of target y
        :rtype: None
        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)
