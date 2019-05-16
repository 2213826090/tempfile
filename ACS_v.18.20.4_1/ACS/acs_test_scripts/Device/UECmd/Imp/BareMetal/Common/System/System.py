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

:organization: INTEL NDG SW
:summary: This file implements the System UEcmd for Clarks devices
:since: 12/09/2014
:author: jreynaux
"""

from datetime import datetime
from acs_test_scripts.Device.UECmd.Imp.BareMetal.Common.Base import Base
from acs_test_scripts.Device.UECmd.Interface.System.ISystem import ISystem
import UtilitiesFWK.Utilities as Util


class System(Base, ISystem):

    """
    :summary: System UEcommands operations for Sensor Enabled devices
                platforms using an UART based communication to the I{DUT}.
    """

    def __init__(self, device):
        """
        Constructor.
        """

        Base.__init__(self, device)
        ISystem.__init__(self, device)

    def get_date_and_time(self):
        """
        Get date and time on the device

        :rtype: list
        :return: operation status & date.
                 Returns date and time in following format in case of success: "%Y-%m-%d %H:%M:%S"
        """
        return_code = Util.Global.FAILURE
        output = 'Unable to retrieve system time !'
        self._logger.debug("Getting System time")

        timestamp = self._internal_exec(cmd="system_get_time", broadcast=False)

        if type(timestamp) is int:
            return_code = Util.Global.SUCCESS
            output = datetime.fromtimestamp(timestamp).strftime("%Y-%m-%d %H:%M:%S")
            self._logger.debug("System time is: {0}".format(output))

        return return_code, output