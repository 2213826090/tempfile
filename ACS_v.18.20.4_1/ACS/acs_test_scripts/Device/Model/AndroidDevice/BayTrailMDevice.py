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
:summary: This file implements the BayTrail M platform
:since: 28/11/2013
:author: ssavrimoutou
"""

import time
from UtilitiesFWK.Utilities import Global
from Device.Model.AndroidDevice.AndroidDeviceBase import AndroidDeviceBase
from Device.Model.AndroidDevice.IntelDeviceBase import IntelDeviceBase
from Device.Model.AndroidDevice.BayTrailDevice import BayTrailDevice


class BayTrailMDevice(BayTrailDevice):

    """
        BayTrail M device implementation
    """

    def get_boot_mode(self):
        """
        get the boot mode from adb

        :rtype: string
        :return: device state
        """
        if self._android_codeline == "AIA":
            mode = AndroidDeviceBase.get_boot_mode(self)
        else:
            mode = IntelDeviceBase.get_boot_mode(self)

        return mode

    def switch_on(self, boot_timeout=None, settledown_duration=None, simple_switch_mode=False):
        """
        Switch ON the device
        This can be done either via the power supply or via IO card

        :param boot_timeout: Total time to wait for booting
        :type boot_timeout: int

        :param settledown_duration: Time to wait until start to count for timeout,
                                    Period during which the device must have started.
        :type settledown_duration: int

        :param simple_switch_mode: a C{boolean} indicating whether we want
                                to perform a simple switch on.
        :type simple_switch_mode: bool

        :rtype: list
        :return: Output status and output log
        """
        self.get_logger().info("Switching on the device...")
        return_code = Global.FAILURE
        return_message = ""

        if self._is_phone_booted:
            return_code = Global.SUCCESS
            return_message = "Device already switched on."
            self.get_logger().info(return_message)
            if not self.is_available():
                self.connect_board()

        else:
            self._acs_agent.is_started = False
            # Handle entry parameter boot_timeout
            boot_timeout = self._init_boot_timeout(boot_timeout)

            self.get_logger().info("Switching on the device...")

            device_state = self.get_state()

            # Ask for device state
            if device_state != "alive":
                # Device state is not detected or device is offline => try to switch on
                self._eqts_controller.plug_device_power()
                self._eqts_controller.poweron_device()

                # Ask for device state
                device_state = self.get_state()
                start_time = time.time()
                end_time = start_time + boot_timeout

                while device_state != "alive" and time.time() < end_time:
                    device_state = self.get_state()
                    time.sleep(1)
                # update remaining boot timeout
                boot_timeout -= time.time() - start_time

                if device_state != "alive":
                    return_code = Global.FAILURE
                    return_message = "Device is %s, unable to boot after %d !" % (device_state, boot_timeout)

            if device_state == "alive":
                # Device is "alive"
                # Get the device mode
                device_boot_mode = self.get_boot_mode()
                start_time = time.time()
                end_time = start_time + boot_timeout
                while device_boot_mode == "UNKNOWN" and time.time() < end_time:
                    device_boot_mode = self.get_boot_mode()
                    time.sleep(1)
                # update remaining boot timeout
                boot_timeout -= time.time() - start_time

                if device_boot_mode == "MOS":
                    self._is_phone_booted = True
                    return_code = Global.SUCCESS
                    return_message = "Device booted"
                    # Device already booted
                    self.get_logger().info(return_message)
                    self._is_device_connected = False
                    self.connect_board()
                elif device_boot_mode != "UNKNOWN":
                    self.get_logger().info("Device is currently in %s , try a reboot" % device_boot_mode)
                    # Reboot the device in MOS
                    self._is_phone_booted = self.reboot(mode="MOS",
                                                        wait_for_transition=True,
                                                        transition_timeout=int(boot_timeout),
                                                        skip_failure=True,
                                                        wait_settledown_duration=True)

                    if self._is_phone_booted:
                        return_code = Global.SUCCESS
                        return_message = "Device successfully booted"
                    else:
                        return_code = Global.FAILURE
                        return_message = "Device has failed to boot after %d seconds!" % boot_timeout
                else:
                    return_code = Global.FAILURE
                    return_message = "Device has failed to boot after %d seconds!" % boot_timeout

        if return_code != Global.SUCCESS:
            self.hard_shutdown(wait_for_board_off=True)

        # When device booted, can retrieve device info
        return return_code, return_message
