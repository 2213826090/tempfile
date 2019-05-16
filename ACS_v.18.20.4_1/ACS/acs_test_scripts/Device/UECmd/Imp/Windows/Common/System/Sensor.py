"""
:copyright: (c)Copyright 2015, Intel Corporation All Rights Reserved.
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

:organization: INTEL OPM PC&WTE
:summary: This file implements the Sensor UEcmd for Windows device
:since: 18/12/2015
:author: mcarriex
"""

from acs_test_scripts.Device.UECmd.Imp.Windows.Common.Base import Base
from acs_test_scripts.Device.UECmd.Interface.System.ISensor import ISensor


class Sensor(Base, ISensor):
    """
    :summary: PhoneSystem UEcommands operations for Windows platforms
    using an C{Intent} based communication to the I{DUT}.
    """

    def __init__(self, device):
        """
        Constructor.
        """

        Base.__init__(self, device)
        ISensor.__init__(self, device)
        self._target_class = "acscmd.sensor.SensorInfoModule"
