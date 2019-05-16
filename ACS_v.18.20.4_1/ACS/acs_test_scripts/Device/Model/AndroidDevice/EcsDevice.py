"""

:copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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
:summary: This file implements the GMIN TCP-IP target platforms
:since: 3/4/2014
:author: mceausu
"""
import time
from Device.Model.AndroidDevice.AsusT100Device import AsusT100Device
from UtilitiesFWK.Utilities import Global

class EcsDevice(AsusT100Device):

    """
    GMIN ADB over USB target platforms implementation
    """

    def __init__(self, config, logger):
        """
        Constructor
        """
        AsusT100Device.__init__(self, config, logger)
        self.dnx_push_relay_timeout = self.config.get_value("dnxKeyComboPressTimeout", 5)
        self.pos_push_relay_timeout = self.config.get_value("posKeyComboPressTimeout", 8)
        self.prov_os_boot_duration = self.config.get_value("provOsBootDuration", 5, int)
        self._flash_module = self.get_device_module("FlashModule")
        self._flash_configuration_value = self._flash_module.configuration.get_value("flashConfigurationType")

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

        self._eqts_controller.poweron_device()
        self.get_logger().info("Wait between ON/OFF 2s")
        time.sleep(2)
        self._eqts_controller.poweroff_device()
        self._eqts_controller.cut_device_power()

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

    def __hardware_pos_transition(self):
        """
        :rtype: bool
        :return: status of operation (True = OK, False = NOK)
        """
        status = False
        try:
            for x in range(0,10):
                self.get_logger().debug("Wait POS (%ds) ... (%d/9)"%(self.prov_os_boot_duration, x))
                time.sleep(self.prov_os_boot_duration)
                if self._check_pos_mode() == "POS":
                    status = True
                    break
        except Exception as e:
            self._logger.debug(str(e))
        return status

    def _pos_line_on(self):
        """
        Enter in the POS mode (used for flashing procedure) by trigging the provisionning line over the board

        :rtype: int (Global.SUCCESS, Global.FAILURE, Global.BLOCKED)
        :return: result of the switch state request

        """
        return_code = Global.FAILURE
        self.get_logger().info("Enable provisioning line: Trig POS mode")

        # Enter Boot loader
        if self._flash_configuration_value and self._flash_configuration_value.strip() and self._flash_configuration_value.strip() == "update":
                #enter POS = fastboot mode
                self._eqts_controller.press_key_combo(keycombo_list="VOLUME_DOWN+POWER_BUTTON," + self.pos_push_relay_timeout, push_delay=2)
        else:
            #enter DNX_OS mode
            self._eqts_controller.press_key_combo(keycombo_list="VOLUME_DOWN+VOLUME_UP+POWER_BUTTON," + self.dnx_push_relay_timeout, push_delay=2)

        # Switch to POS for flashing
        if not self.__hardware_pos_transition():
            self._logger.debug("Boot in POS through hardware means failed, try to reboot in POS by software method")

            # Switch off and switch on the device
            if self.hard_shutdown(wait_for_board_off=True):
                return_code, _ = self.switch_on()

                if return_code == Global.SUCCESS:
                    # Try to do pass in POS with software command
                    if not self._software_pos_transition():
                        self._logger.debug("Boot in POS through software means failed")
                    else:
                        return_code = Global.SUCCESS
        else:
            return_code = Global.SUCCESS
        return return_code
