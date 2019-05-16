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
@summary: EdisonFlashModule flash module extension
@since: 4/16/14
@author: sfusilie
"""
from acs_test_scripts.Device.Module.NDG.Flash.BaseFlashModule import BaseFlashModule
from acs_test_scripts.Device.Module.NDG.Flash.FlashManager.FlashDeviceInfo import FlashDeviceInfo


class EdisonFlashModule(BaseFlashModule):

    """
    Edison flash module implementation, changing only device state methods
    """
    # Define the different device state actions
    CHANGE_DEVICE_STATE = {"FLASH_SWITCH_OFF": "_flash_switch_off",
                            "WAIT_BOOT": "_wait_board_is_ready"}

    def __init__(self):
        super(EdisonFlashModule, self).__init__()
        self._device_info = FlashDeviceInfo(self.device_properties)
        self.__flash_input_data = None
        self.__flash_manager = None

    def _collect_flash_device_info(self):
        self._device_info.change_device_state_fct = self._switch_device_state

    @property
    def device_properties(self):
        info = {"software_release": "Linux",
                "hardware_model": "edison",
                "baseband_version": "",
                "firmware_version": "Yocto based"}
        return info
