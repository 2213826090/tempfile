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

:organization: INTEL MCG PSI
:summary: This file implements the Galaxy Tab 3 platform
:since: 27/11/2013
:author: cbonnard
"""
import time

from Device.Model.AndroidDevice.IntelDeviceBase import IntelDeviceBase
from Device.Model.AndroidDevice.AndroidDeviceBase import AndroidDeviceBase
from UtilitiesFWK.Utilities import Global


class GalaxyTab3Device(IntelDeviceBase):

    """
    Galaxy Tab 3 platform implementation.
    It inherits from IntelDevice, because the Galaxy Tab 3 is based on a Clovertrail SOC
    """

    def __init__(self, config, logger):
        """
        Constructor

        :type  phone_name: str
        :param phone_name: Name of the current phone(e.g. PHONE1)
        """
        IntelDeviceBase.__init__(self, config, logger)

        # Flashing configuration elements (This device class only supports OS flashing procedure)

        # TODO: in case of Galaxy tab "fastbootKeyCombo" does not make sense because, there is no fastboot command for this device.
        # But to avoid adding other new key which do the same things, we reuse it.
        # Rework of device flashing (in collaboration with pupdr) is planned. This key could be refactored at this time.
        self._pos_keycombo = self.get_config("fastbootKeyCombo")

    def hard_shutdown(self, wait_for_board_off=False):
        """"
        Perform a hard shutdown and wait for the device is off

        :type wait_for_board_off: bool
        :param wait_for_board_off: Wait for device is off or not after soft shutdown

        :rtype: bool
        :return: If device is off or not
        """

        msg = "Hard shutting down the device..."
        self.get_logger().info(msg)

        self.inject_device_log("i", "ACS", msg)

        # Initialize variable
        device_off = False

        # Disconnect device before shutting down takes effect
        self.disconnect_board()

        # unplug cable after soft shutdown to avoid COS
        if not self._use_adb_over_ethernet:
            self._eqts_controller.disconnect_usb_host_to_dut()

        # Press HW power button 1 second to power on board, in case board if OFF
        # because on GalaxyTab3 pressing power button 10 seconds while device is OFF will power it on
        self._eqts_controller.press_power_button(1)
        time.sleep(1)
        self._eqts_controller.poweroff_device()

        # It should not be needed to wait after hard shutdown request to have effect
        # TO DO: most of the time this time sleep is not needed it shall be tuned in
        # Device_Catalog.xml for every device
        self.get_logger().info("Wait hard shutdown duration (%ds)..." % self._hard_shutdown_duration)
        time.sleep(self._hard_shutdown_duration)

        if wait_for_board_off:
            if self._check_shutdown(self._use_adb_over_ethernet):
                # Device is switched off
                device_off = True
        else:
            # By pass wait for device off.
            # We consider that the device is switched off
            device_off = True

        # Update device boot status according to hard shutdown result
        # Important when this function is called outside switch_off()
        if device_off:
            self._is_phone_booted = False

        return device_off

    def _get_device_boot_mode(self):
        """
        get the boot mode from adb

        :rtype: str
        :return: device state : MOS or UNKNOWN
        """
        return AndroidDeviceBase._get_device_boot_mode(self)

    def __hardware_pos_transition(self):
        """
        Switch to POS OS using key combo

        :rtype: bool
        :return: status of operation
        """
        status = False
        try:
            status = self._eqts_controller.press_key_combo(self._pos_keycombo)

        except Exception as e:
            self._logger.debug(str(e))

        return status

    def _pos_line_on(self):
        """
        Enter in the POS mode (used for flashing procedure) by triggering the provisioning line over the board

        :rtype: int (Global.SUCCESS, Global.FAILURE, Global.BLOCKED)
        :return: result of the switch state request
        """
        return_code = Global.FAILURE
        self.get_logger().info("Enable provisioning line: Trig POS mode")

        # Prepare power of device
        self._eqts_controller.plug_device_power()

        if self._pos_keycombo:
            # Switch to POS for flashing using key combo
            if self.__hardware_pos_transition():
                # Plug usb cable as it has never been done if we use the provisioning mode
                if self._eqts_controller.connect_usb_host_to_dut():
                    return_code = Global.SUCCESS

        else:
            return_code = Global.SUCCESS

            if not self._eqts_controller.connect_usb_host_to_dut():
                # We need to do it there because we may have only the power
                # control (no usb, no provisioning line), we have to try to start
                # the device
                self._eqts_controller.poweron_device()

        return return_code

    def _pos_line_off(self):
        """
         Disable device trig to POS. Stay in POS mode (Stay in flash mode)

        :rtype: int (Global.SUCCESS, Global.FAILURE, Global.BLOCKED)
        :return: result of the switch state request
        """
        # Nothing to disable
        return Global.SUCCESS
