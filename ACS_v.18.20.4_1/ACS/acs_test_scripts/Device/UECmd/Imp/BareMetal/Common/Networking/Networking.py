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

:organization: INTEL NDG SW Dev
:summary: This file implements Networking UECmds for Clark (mostly stubs)
:since: 21/03/2011
:author: jreynaud
"""
from acs_test_scripts.Device.UECmd.Interface.Networking.INetworking import INetworking
from acs_test_scripts.Device.UECmd.Imp.BareMetal.Common.Base import Base
import inspect


class Networking(Base, INetworking):

    """
    Class that handle all networking operations
    """
    def __init__(self, device):
        """
        Constructor
        """
        Base.__init__(self, device)
        INetworking.__init__(self, device)
        self._logger = device.get_logger()
        self._device = device

    def set_flight_mode(self, mode):
        """
        Sets the flight mode to off or on.

        :type mode: str or int
        :param mode: can be ('on', '1', 1) to enable
                            ('off', '0', 0) to disable

        :return: None
        """
        self._logger.debug("{0}: Not yet implemented on {1} device".
                           format(inspect.stack()[0][3], self._device.get_name()))

    def get_flight_mode(self):
        """
        Returns the flight mode.

        :rtype: int
        :return: 0 for OFF or 1 for ON
        """
        self._logger.debug("{0}: Not yet implemented on {1} device".
                           format(inspect.stack()[0][3], self._device.get_name()))
        return 0