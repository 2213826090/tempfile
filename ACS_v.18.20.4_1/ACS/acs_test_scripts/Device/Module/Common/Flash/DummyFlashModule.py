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
@summary: Flash Manager proxy that exposes the flash manager as a flash module
@since: 27/5/14
@author: kturban
"""
from acs_test_scripts.Device.Module.Common.Flash.IFlashModule import IFlashModule
from Device.Module.DeviceModuleBase import DeviceModuleBase
from UtilitiesFWK.Utilities import Global


class DummyFlashModule(IFlashModule, DeviceModuleBase):

    @property
    def flash_properties(self):
        return {}

    @property
    def device_properties(self):
        return {}

    def init(self, flash_input):
        self.logger.info("Dummy Init Flash Module Success")
        return Global.SUCCESS

    def flash(self, timeout):
        self.logger.info("Dummy Flash Module Success")
        return Global.SUCCESS
