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

:organization: INTEL SVE DSV
:summary: This file implements the Moorefield platform DeviceChecks module
:since: 12/14/2014
:author: srdubbak
"""

from acs_test_scripts.Device.Module.Common.DeviceChecksModule.IDeviceChecksModule import IDeviceChecksModule
from ErrorHandling.DeviceException import DeviceException
from Device.Module.DeviceModuleBase import DeviceModuleBase


class CherrytrailDeviceChecksModule(IDeviceChecksModule, DeviceModuleBase):

    def __init__(self):
        super(CherrytrailDeviceChecksModule, self).__init__()

    def init(self):
        """
        Initialize nw module

        :rtype: UtilitiesFWK.Utilities.Global
        :return: Init status
        """
        verdict = Global.SUCCESS
        self._networking_properties = self.configuration
        return verdict