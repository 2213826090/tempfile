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
:summary: This script is the interface of display uecmd.
:author: pbluniex
"""
from ErrorHandling.DeviceException import DeviceException


class IDisplay:

    """
    Abstract class that defines the interface to be implemented
    by device display operations handling sub classes.

    All method that shall be redefined in sub-classes raise a
    I{DeviceException} error.
    """

    def __init__(self, device):
        """
        Initializes this instance.

        Nothing to be done in abstract class.

        :type device: DeviceBase
        :param device: The DUT
        """
        pass

    def set_display_orientation(self, _orientation):
        """
        Set screen orientation

        :type orientation: str
        :param orientation: Orientation to force.
                            Can be "portrait" or "landscape"
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def is_hdmi_connected(self):
        """
        Check whether HDMI is connected or not

        :rtype: boolean
        :return: return True if HDMI exists.
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_user_orientation(self, orientation):
        """
        Set screen orientation.  This method is an alternative to set_display_orientation, and uses system setting instead of Android API.

        :type orientation: str
        :param orientation: Orientation to force.
                            Can be "portrait", "landscape", "reverse_portrait" or "reverse_landscape, other value
                            sets auto rotate screen
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_auto_orientation_mode(self, enabled):
        """
        Set screen orientation

        :type enabled: bool
        :param enabled: True to enable auto rotation
                        False to enable user rotation
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)