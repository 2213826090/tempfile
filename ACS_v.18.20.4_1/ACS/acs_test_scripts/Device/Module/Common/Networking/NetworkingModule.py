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
@summary: Networking module to expose networking properties
@since: 2014/09/25
@author: jreynaux
"""
from acs_test_scripts.Device.Module.Common.Networking.INetworkingModule import INetworkingModule
from UtilitiesFWK.AttributeDict import AttributeDict
from Device.Module.DeviceModuleBase import DeviceModuleBase
from UtilitiesFWK.Utilities import Global


class NetworkingModule(INetworkingModule, DeviceModuleBase):

    def __init__(self):
        super(NetworkingModule, self).__init__()
        self._networking_properties = AttributeDict()

    @property
    def networking_properties(self):
        return self._networking_properties

    def init(self):
        """
        Initialize nw module

        :rtype: UtilitiesFWK.Utilities.Global
        :return: Init status
        """
        verdict = Global.SUCCESS
        self._networking_properties = self.configuration
        return verdict

    def disable_one_time_setup_mode(self):
        """
        Disable the One Time Setup mode, and prevent it to start at boot.

        :note: Actually when power button is pressed 2s, it start the Out-Of-The-Box experience
                The wifi is on access point mode and IP set to 192.168.2.15, allowing customer to connect
                to device first and configure as he which.
                But this mode can interfere with usual wifi usage.

        :rtype: None
        """
        self.logger.info("Not implemented")
