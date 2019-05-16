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
@summary: NordicFlashModule flash module extension (apply for Clark, Lois, Diana ...)
@since: 9/15/14
@author: jreynaux
"""
from acs_test_scripts.Device.Module.NDG.Flash.BaseFlashModule import BaseFlashModule
from acs_test_scripts.Device.Module.NDG.Flash.FlashManager.FlashDeviceInfo import FlashDeviceInfo
from UtilitiesFWK.Utilities import AcsConstants


class SensorEnabledDeviceFlashModule(BaseFlashModule):

    """
    Nordic devices flash module implementation, changing only device state methods
    """
    # Define the different device state actions
    CHANGE_DEVICE_STATE = {"WAIT_BOOT": "_wait_board_is_ready"}

    def __init__(self):
        super(SensorEnabledDeviceFlashModule, self).__init__()
        self._device_info = FlashDeviceInfo()
        self.__flash_input_data = None
        self.__flash_manager = None

    def _collect_flash_device_info(self):

        self._device_info.change_device_state_fct = self._switch_device_state
        self._device_info.board_type = self.device.config.get_value("boardType")

    def update_device_info_with_build_info(self, dict_info):
        """
        Update device info with build properties from flash files
        """

        if dict_info and len(dict_info) > 0:
            self.logger.info("Update device info with build properties from flash files")
            # Update properties which can be updated
            self.device.device_properties.sw_release_flashed = dict_info.get("software_release", AcsConstants.NOT_AVAILABLE)

    @property
    def device_properties(self):
        """
        Property holding Device properties

        :return: Device properties
        :rtype: dict

        """
        return self._device_info
