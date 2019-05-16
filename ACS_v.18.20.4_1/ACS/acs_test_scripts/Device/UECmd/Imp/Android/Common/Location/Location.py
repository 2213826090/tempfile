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
:summary: This file implements the Localization UEcmd for Android device
:since: 14/11/2013
:author: mmorchex
"""

from acs_test_scripts.Device.UECmd.Imp.Android.Common.BaseV2 import BaseV2
from acs_test_scripts.Device.UECmd.Interface.Location.ILocation import ILocation
from acs_test_scripts.Device.UECmd.UECmdDecorator import need
from ErrorHandling.AcsConfigException import AcsConfigException


class Location(BaseV2, ILocation):

    """
    :summary: Location UEcommands operations for Android platform.
    """

    def __init__(self, device):
        """
        Constructor
        """

        BaseV2.__init__(self, device)
        # ILocalization.__init__(self, device)
        self._logger = device.get_logger()
        self._uecmd_default_timeout = device.get_uecmd_timeout()
        self._gps_module = "acscmd.location.GpsModule"

    @need('gps', False)
    def set_gps_power(self, mode):
        """
        Sets the GPS power to on/off.

        :type mode: str or int or boolean
        :param mode: can be ('on', '1', 1, True) to enable
                            ('off', '0', 0, False) to disable

        :return: None
        """
        current_mode = self.get_gps_power_status()
        if mode in ("on", "1", 1, True):
            if current_mode == 1:
                warning_msg = "gps Power is already on"
                self._logger.info(warning_msg)
                return
            else:
                mode = 1
        elif mode in ("off", "0", 0, False):
            if current_mode == 0:
                warning_msg = "gps Power is already off"
                self._logger.info(warning_msg)
                return
            else:
                mode = 0
        else:
            message = "Invalid parameter value for 'mode': %s" % mode
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, message)
        # Set the gps power mode
        function = "setGpsPower"
        cmd_args = "--ei mode %s" % mode
        self._internal_exec_v2(self._gps_module, function, cmd_args, is_system=True)

    @need('gps', False, 0)
    def get_gps_power_status(self):
        """
        Gets the GPS power status.

        :rtype: int
        :return: 0 for OFF or 1 for ON
        """
        function = "getGpsStatus"
        output = self._internal_exec_v2(self._gps_module, function, is_system=True)
        self._logger.info("Gps power status: " + output["getGpsStatus"])

        return int(output["getGpsStatus"])
