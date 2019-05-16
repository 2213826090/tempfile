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

:organization: SII on behalf of INTEL MCG PSI
:summary: This file implements Localization UECmds for Windows device
:since: 15 Sep 2014
:author: mcarriex
"""
from acs_test_scripts.Device.UECmd.Interface.Location.ILocation import ILocation
from acs_test_scripts.Device.UECmd.Imp.Windows.Common.Base import Base
from ErrorHandling.AcsBaseException import AcsBaseException


class Location(Base, ILocation):
    """
    Class that handles all location related operations.
    """

    def __init__(self, device):
        """
        Initializes this instance.
        """
        Base.__init__(self, device)
        ILocation.__init__(self, device)

    def set_gps_power(self, mode):
        """
        Sets the GPS power to on/off.

        :type mode: str or int or boolean
        :param mode: can be ('on', '1', 1, True) to enable
                            ('off', '0', 0, False) to disable

        :return: None
        """
        self._logger.debug("[NOT IMPLEMENTED] set_gps_power on Windows")

    def get_gps_power_status(self):
        """
        Gets the GPS power status.

        :rtype: int
        :return: 0 for OFF or 1 for ON
        """
        self._logger.debug("[NOT IMPLEMENTED] get_gps_power_status on Windows")
