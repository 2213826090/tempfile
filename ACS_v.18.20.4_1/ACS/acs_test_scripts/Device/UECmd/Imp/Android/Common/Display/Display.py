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
:summary: This script implements the interface of display uecmd.
:author: pbluniex
"""
import re
import time
from acs_test_scripts.Device.UECmd.Imp.Android.Common.BaseV2 import BaseV2
from acs_test_scripts.Device.UECmd.Interface.Display.IDisplay import IDisplay
import UtilitiesFWK.Utilities as Utils
from ErrorHandling.AcsToolException import AcsToolException
from ErrorHandling.DeviceException import DeviceException

class Display(BaseV2, IDisplay):

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
        BaseV2.__init__(self, device)
        IDisplay.__init__(self, device)

        self.__rotation_module = "acscmd.display.RotationModule"
        self.__display_module = "acscmd.display.DisplayModule"

    def set_auto_orientation_mode(self, enabled):
        """
        Set screen orientation

        :type enabled: bool
        :param enabled: True to enable auto rotation
                        False to enable user rotation
        """
        if enabled==True:
            self._logger.debug("Enable auto orientation and disable user orientation")
            cmd="adb shell content insert --uri content://settings/system --bind name:s:accelerometer_rotation --bind value:i:1"
            self._exec(cmd)
        else:
            self._logger.debug("Disable auto orientation and disable user orientation")
            cmd="adb shell content insert --uri content://settings/system --bind name:s:accelerometer_rotation --bind value:i:0"
            self._exec(cmd)

    def set_display_orientation(self, orientation):
        """
        Set screen orientation

        :type orientation: str
        :param orientation: Orientation to force.
                            Can be "portrait", "landscape", "reverse_portrait" or "reverse_landscape, other value
                            sets auto rotate screen
        """

        self._logger.debug("Setting orientation to %s " % orientation)
        if orientation.lower() == "portrait":
            self._internal_exec_v2(self.__rotation_module, "forcePortrait")
        elif orientation.lower() == "landscape":
            self._internal_exec_v2(self.__rotation_module, "forceLandscape")
        elif orientation.lower() == "reverse_landscape":
            self._internal_exec_v2(self.__rotation_module, "forceReverseLandscape")
        elif orientation.lower() == "reverse_portrait":
            self._internal_exec_v2(self.__rotation_module, "forceReversePortrait")
        else:
            self._internal_exec_v2(self.__rotation_module, "setAuto")

    def set_user_orientation(self, orientation):
        """
        Set screen orientation.  This method is an alternative to set_display_orientation, and uses system setting instead of Android API.

        :type orientation: str
        :param orientation: Orientation to force.
                            Can be "portrait", "landscape", "reverse_portrait" or "reverse_landscape, other value
                            sets auto rotate screen
        """

        self._logger.debug("Setting orientation to %s "%orientation)
        if orientation=="landscape":
            orientation_value=1
        elif orientation=="reverse_portrait":
            orientation_value=2
        elif orientation=="reverse_landscape":
            orientation_value=3
        else:
            orientation_value=0

        cmd="adb shell content insert --uri content://settings/system --bind name:s:user_rotation --bind value:i:{0}".format(orientation_value)
        self._exec(cmd)

    def is_hdmi_connected(self):
        """
        Check whether HDMI is connected or not by reading dumpsys SurfaceFlinger
        :rtype: boolean
        :return: return True if HDMI exists.
        """

        cmd = 'adb shell "dumpsys SurfaceFlinger | grep External"'
        (return_code, output) = self._device.run_cmd(cmd, timeout=10)

        if return_code != Utils.Global.SUCCESS:
            msg = "uecmd is_hdmi_connected: Error '%s' when trying to check HDMI" % (output)
            self._logger.error(msg)
            raise AcsToolException(AcsToolException.OPERATION_FAILED, msg)

        re_match = re.match(".*External \((.*)\)", output)
        if re_match and len(re_match.groups())==1:
            out_msg = re_match.group(1)
        else:
            return False

        if out_msg == "connected":
            return True
        else:
            return False