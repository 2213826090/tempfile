#!/usr/bin/env python
# -*- coding: utf-8 -*-

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

@organization: INTEL NDG SW
@summary: Pnp module Interface to expose pnp properties
@since: 2014/10/08
@author: jbourgex
"""
from acs_test_scripts.Device.Module.MCG.Pnp.IPnpModule import IPnpModule
from UtilitiesFWK.AttributeDict import AttributeDict
from Device.Module.DeviceModuleBase import DeviceModuleBase
from UtilitiesFWK.Utilities import Global


class PnpModule(IPnpModule, DeviceModuleBase):

    def __init__(self):
        super(PnpModule, self).__init__()
        self._pnp_properties = AttributeDict()

    @property
    def pnp_properties(self):
        return self._pnp_properties

    def init(self):
        """
        Initialize pnp module

        :rtype: UtilitiesFWK.Utilities.Global
        :return: Init status
        """
        verdict = Global.SUCCESS
        self._pnp_properties = self.configuration
        return verdict
