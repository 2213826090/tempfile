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
:summary: This file implements the Modem Flashing UEcmds for Windows device
:since: 18/12/2015
:author: mcarriex
"""

import sys

from acs_test_scripts.Device.UECmd.Imp.Windows.Common.Base import Base
from acs_test_scripts.Device.UECmd.Interface.System.IModemFlashing import IModemFlashing
from acs_test_scripts.Device.UECmd.UECmdDecorator import need

# Import winreg only when running on Windows system
if "win" in sys.platform.lower():
    import _winreg


class ModemFlashing(Base, IModemFlashing):
    """
    :summary: Modem Flashing UEcommands operations for Windows platforms.
    """

    AT_PROXY_NORMAL_MODE = 1
    """
    The C{integer} value corresponding to AT proxy
    in normal mode.
    """

    AT_PROXY_TUNNELING_MODE = 2
    """
    The C{integer} value corresponding to AT proxy
    in tunneling mode.
    """

    AT_PROXY_PROPERTY_NAME = "persist.system.at-proxy.mode"
    """
    The name of the I{Windows} system property that steers
    AT proxy's behavior.
    """

    AT_PROXY_TTY = "/dev/ttyACM0"
    """
    The name of the AT proxy TTY.
    """

    @need('modem')
    def __init__(self, device):
        """
        Constructor.
        """
        Base.__init__(self, device)
        IModemFlashing.__init__(self, device)
        self._serial_number = device.get_serial_number()
        self._at_proxy_supported = device.get_config("isATProxySupported", "True", "str_to_bool")
        self._at_cmd_port = device.get_config("ATCmdPort", "True", "str_to_bool")
