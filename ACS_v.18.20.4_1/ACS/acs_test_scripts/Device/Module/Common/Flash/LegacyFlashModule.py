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
@since: 4/7/14
@author: sfusilie
"""
from acs_test_scripts.Device.Module.Common.Flash.IFlashModule import IFlashModule
from acs_test_scripts.Device.Module.Common.Flash.FlashManager.FlashDeviceInfo import FlashDeviceInfo
from acs_test_scripts.Device.Module.Common.Flash.FlashManager.FlashManager import FlashManager
from Device.Module.DeviceModuleBase import DeviceModuleBase
from UtilitiesFWK.Utilities import AcsConstants
from UtilitiesFWK.Utilities import Global


class LegacyFlashModule(IFlashModule, DeviceModuleBase):
    # Define the different device state actions
    # TODO: CHANGE_DEVICE_STATE shall be replaced by a better mechanism to describe flash device behavior
    #       either exposes the methods at device interface level
    #       either exposes the methods as another device module
    #       or internalize those methods in the flash module itself
    CHANGE_DEVICE_STATE = {"FLASH_SWITCH_OFF": "_flash_switch_off",
                           "POS_MODE_ON": "_pos_line_on",
                           "POS_MODE_OFF": "_pos_line_off",
                           "WAIT_BOOT": "_wait_board_is_ready",
                           "SWITCH_ON": "_flash_device_switch_on"}

    def __init__(self):
        super(LegacyFlashModule, self).__init__()
        self._device_info = FlashDeviceInfo({"software_release": "",
                                             "hardware_model": "",
                                             "baseband_version": "",
                                             "firmware_version": ""})
        self.__flash_input_data = None
        self.__flash_manager = None

    def _switch_device_state(self, device_state):
        """
        :type device_state: string belonging CHANGE_DEVICE_STATE dict keys
        :param device_state: state requested for the device

        :rtype: int (Global.SUCCESS, Global.FAILURE, Global.BLOCKED)
        :return: result of the switch state request

        """
        if device_state in self.CHANGE_DEVICE_STATE:
            # Switch the device in the state requested
            return_code = getattr(self.device, self.CHANGE_DEVICE_STATE[device_state])()
        else:
            self.logger.error("Device state requested does not exist")
            return_code = Global.FAILURE
        return return_code

    def _collect_flash_device_info(self):
        self._device_info.soc_serial_number = self.device.config.get_value("socSerialNumber")
        self._device_info.change_device_state_fct = self._switch_device_state
        self._device_info.adb_over_ethernet = self.device.config.get_value("adbOverEthernet", "False",
                                                                           "str_to_bool")
        # The device is going to be flashed
        # Collect needed flash device info
        if not self._device_info.adb_over_ethernet:
            # Build flash device info using the serial info define in the config (and not retrieved automatically).
            # If the user do not want to specify a serial number, we do not want to use one as it is
            # automatically retrieve after the first flash
            self._device_info.device_serial_number = self.device.config.serialNumber
            self._device_info.device_pos_serial_number = self.device.config.serialNumber
        else:
            # Be careful for a device over ethernet the serial number is the ip address of the board in POS mode
            # and ip address:port in MOS state (adb commands)
            # We always flash in POS mode so we use serial number = ip address for a device over ethernet
            self._device_info.device_serial_number = self.device.config.ipAddress
            self._device_info.device_pos_serial_number = self.device.config.POSipAddress
        # lock manager should be moved as virtual equipment
        self._device_info.lock_manager_ipAdress = self.device.config.get_value("lockManagerIpAdress", "localhost")
        self._device_info.lock_manager_port = self.device.config.get_value("lockManagerPort", "8005", int)

    @property
    def flash_properties(self):
        return self._device_info

    @property
    def device_properties(self):
        return {}

    def init(self, flash_input):
        """
        Initialize flash module

        :type flash_input: str
        :param flash_input: flash file path

        :rtype: UtilitiesFWK.Utilities.Global
        :return: init status
        """
        # please use the right dedicated flash module
        self.logger.warning("This module is deprecated and it is no more supported")
        verdict = Global.SUCCESS
        self.__flash_input_data = flash_input

        self._collect_flash_device_info()
        self.__flash_manager = FlashManager(self.configuration.OSflashTool,
                                            self.logger,
                                            self.configuration.get_value("flashBIOS", "False", "str_to_bool"),
                                            self.configuration.get_value("BIOSflashTool", "DEDIPROG"),
                                            self.configuration.get_value("voltage", "1.8V"))

        # Check flash files exist, with correct format and extract flash file info
        flash_info = self.__flash_manager.check_flash_file(flash_input)
        if flash_info:
            self._device_info.update(flash_info)
        self.update_device_info_with_build_info(self.flash_properties)
        return verdict

    def flash(self, timeout):
        """
        Start the flash process. Blocking call.

        :type timeout: float
        :param timeout: flash timeout

        :rtype: UtilitiesFWK.Utilities.Global
        :return: flash status
        """
        # please use the right dedicated flash module
        self.logger.warning("This module is deprecated and it is no more supported")
        flash_result = Global.FAILURE
        multiple_devices = self.device.config.get_value("multipleDevices", "False", "str_to_bool")
        ip_address = self.device.config.get_value("ipAddress", "192.168.42.1")
        pos_ip_adress = self.device.config.get_value("POSipAddress", "192.168.42.1")
        if multiple_devices and ip_address == pos_ip_adress and ip_address != "192.168.42.1":
            multiple_devices = False
        if self.__flash_manager is not None:
            flash_result = self.__flash_manager.flash(device_info=self._device_info, timeout=timeout,
                                                      multiple_devices=multiple_devices)
        if flash_result == Global.SUCCESS:
            self.logger.info("Device successfully flashed!")
            self.update_device_info_with_build_info(self.device_properties)
        else:
            self.logger.error("Flashing device has failed!..Exit flash!")
        return flash_result

    def update_device_info_with_build_info(self, dict_info):
        """
        Update device info with build properties from flash files
        """
        # please use the right dedicated flash module
        self.logger.warning("This module is deprecated and it is no more supported")
        if dict_info:
            self.logger.info("Update device info with build properties from flash files")

            # Update properties which can be updated
            self.device.device_properties.sw_release = dict_info.get("software_release", AcsConstants.NOT_AVAILABLE)
            self.device.device_properties.model_number = dict_info.get("hardware_model", AcsConstants.NOT_AVAILABLE)
            self.device.device_properties.baseband_version = dict_info.get(
                "baseband_version", AcsConstants.NOT_AVAILABLE)
            self.device.device_properties.fw_version = dict_info.get("firmware_version", AcsConstants.NOT_AVAILABLE)
