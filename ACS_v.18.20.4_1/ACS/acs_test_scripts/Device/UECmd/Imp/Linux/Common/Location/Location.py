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

:organization: INTEL NDG
:summary: This file implements the Localization UEcmd for Linux device
:since: 14/11/2013
:author: agoeax
"""

from acs_test_scripts.Device.UECmd.Interface.Location.ILocation import ILocation
from acs_test_scripts.Device.UECmd.Imp.Linux.Common.Base import Base

class Location(Base, ILocation):

    """
    :summary: Location UEcommands operations for Android platform.
    """

    def __init__(self, device):
        """
        Constructor
        """
        self._logger = device.get_logger()
        self.__gps_power = None

    def set_gps_power(self, mode):
        """
        Sets the GPS power to on/off.

        :type mode: str or int or boolean
        :param mode: can be ('on', '1', 1, True) to enable
                            ('off', '0', 0, False) to disable

        :return: None
        """
        self._logger.debug("set_gps_power: Not used on Linux devices")

    def get_gps_power_status(self):
        """
        Gets the GPS power status.

        :rtype: int
        :return: 0 for OFF or 1 for ON
        """
        self._logger.debug("get_gps_power_status: Not used on Linux devices")
        return 0