#!/usr/bin/env python
# -*- coding: utf-8 -*-
# pylint: disable=missing-docstring, invalid-name, unused-argument
"""
@copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
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

@organization: INTEL MCG PSI
@summary: System module to expose system properties
@since: 7/8/14
@author: kturban
"""
from Device.Module.DeviceModuleBase import DeviceModuleBase
from acs_test_scripts.Device.Module.Common.System.ISystemModule import ISystemModule
from UtilitiesFWK.AttributeDict import AttributeDict
from UtilitiesFWK.Utilities import Global


class SystemModule(ISystemModule, DeviceModuleBase):
    def __init__(self):
        super(SystemModule, self).__init__()
        self._system_info = AttributeDict()

    @property
    def system_properties(self):
        return self._system_info

    def init(self):
        """
        Initialize system module

        :rtype: UtilitiesFWK.Utilities.Global
        :return: init status
        """
        verdict = Global.SUCCESS
        # for now : multimedia_properties = module config
        # but we can imagine a generic multimedia properties interface between windows/android/linux
        # module would be os specific but keys inside multimedia properties would be generic
        self._system_info = self.configuration
        return verdict
