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
:summary: This file implements the SharkBay platform
:since: 27/11/2013
:author: cbonnard
"""

import time
from UtilitiesFWK.Utilities import Global
from Device.Model.AndroidDevice.IntelDeviceBase import IntelDeviceBase
from Device.Model.AndroidDevice.AndroidDeviceBase import AndroidDeviceBase
from ErrorHandling.DeviceException import DeviceException


class AsusT100Device(IntelDeviceBase):

    """
    SharkBay (haswell&broadwell) platform implementation
    """

    def __init__(self, config, logger):
        """
        Constructor

        :type  phone_name: str
        :param phone_name: Name of the current phone(e.g. PHONE1)
        """
        IntelDeviceBase.__init__(self, config, logger)

        # Benefit of settle duration to set fastboot settle duration
        self._fastboot_settledown_duration = self._settledown_duration
        self._keyboard_emulator_sleep_between_char = 1
        self._multiple_devices = self.config.get_value("multipleDevices", "False", "str_to_bool")

    def _get_device_boot_mode(self):
        """
        Get the boot mode from adb

        :rtype: str
        :return: device state (MOS or UNKNOWN)
        """
        return AndroidDeviceBase._get_device_boot_mode(self)

    def _finalize_os_boot(self, timer, boot_timeout, settledown_duration):
        return AndroidDeviceBase._finalize_os_boot(self, timer=timer, boot_timeout=boot_timeout, settledown_duration=0)

    def __hardware_pos_transition(self):
        """
        Use keyboard emulator to switch to POS OS
        :rtype: bool
        :return: status of operation (True = OK, False = NOK)
        """
        status = False
        try:
            # Be careful - need to define possible different key combo with keyboard emulator
            # for ASUS T100 device to pass in fastboot mode
            pos_on_commands = 'f' * 35
            # press f until reaching fastboot mode for 35s
            for char in pos_on_commands:
                self._eqts_controller.write_keyboard_commands(char, raise_exception=True)
                time.sleep(self._keyboard_emulator_sleep_between_char)
            self.get_logger().debug("Wait settle down fastboot duration (%ds)", self._fastboot_settledown_duration)
            time.sleep(self._fastboot_settledown_duration)
            # Test if we we are in POS mode
            if self._check_pos_mode() == "POS":
                status = True
        except Exception as e:
            self._logger.debug(str(e))

        return status

    def get_boot_mode(self, check_pos_forced=False):
        """
        get the boot mode from adb

        :rtype: string
        :return: device state : MOS, ROS, POS, DNX, COS or UNKNOWN
        """
        current_state = "UNKNOWN"
        retry = 2
        # Implement a retry mechanism as on some benches as on first
        # run of adb (if shutdown previously), it always return unknown
        #
        while retry > 0 and current_state.lower() == "unknown":
            retry -= 1
            current_state = self._get_device_boot_mode()
            if current_state.lower() == "unknown" and (not self._multiple_devices or check_pos_forced):
                # Fastboot command help find if device is in DnX or POS
                # For now we do not differentiate POS and DNX:
                # if DNX or POS -> POS
                # because DNX is tricky to detect and no req has been ask to do so
                current_state = self._check_pos_mode()
            time.sleep(3)

        self._logger.debug("device boot mode is %s" % current_state)

        return current_state

    def reboot(self, mode="MOS", wait_for_transition=True,
               transition_timeout=None, skip_failure=False,
               wait_settledown_duration=False):
        """
        Perform a SOFTWARE reboot on the device.
        By default will bring you to MOS and connect acs once MOS is seen.
        this reboot require that you are in a state where adb or fastboot command can be run.

        :type mode: str or list
        :param mode: mode to reboot in, support MOS, COS, POS, ROS. It can be a list of these modes
               (ie ("COS","MOS"))
               .. warning:: it is not always possible to reboot in a mode from another mode
                eg: not possible to switch from ROS to COS

        :type wait_for_transition: bool
        :param wait_for_transition: if set to true,
                                    it will wait until the wanted mode is reached

        :type transition_timeout: int
        :param transition_timeout: timeout for reaching the wanted mode
                                    by default will be equal to boot timeout set on
                                    device catalog

        :type skip_failure: bool
        :param skip_failure: skip the failure, avoiding raising exception, using
                                this option block the returned value when it is equal to False

        :type wait_settledown_duration: bool
        :param wait_settledown_duration: if set to True, it will wait settleDownDuration seconds
                                          after reboot for Main OS only.

        :rtype: bool
        :return: return True if reboot action succeed depending of the option used, False otherwise
                 - if wait_for_transition not used, it will return True if the reboot action has been seen
                   by the device
                 - if wait_for_transition used , it will return True if the reboot action has been seen
                   by the device and the wanted reboot mode reached.
        """
        if not isinstance(mode, (list, tuple, set, frozenset)):
            mode = [mode]

        for os_mode in mode:
            msg = "Undefined error while rebooting"
            output = Global.FAILURE
            transition_timeout = self._init_boot_timeout(transition_timeout)
            transition_timeout_next = 0
            rebooted = False
            os_mode = os_mode.upper()

            # Wait for device to be ready
            if wait_settledown_duration:
                settledown_duration = self._settledown_duration
            else:
                settledown_duration = None

            # List here command and combo list
            reboot_dict = {"MOS": "adb reboot",
                           "POS": "adb reboot bootloader",
                           "ROS": "adb reboot recovery",
                           "COS": self._soft_shutdown_cmd}

            # exist if mode is unknown
            if os_mode not in reboot_dict:
                msg = "unsupported boot mode %s" % str(os_mode)
                self.get_logger().error(msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

            self.get_logger().info("Trying to reboot device in %s mode" % str(os_mode))
            # get actual boot mode
            actual_state = self.get_boot_mode()
            # boot the device
            if actual_state != "UNKNOWN":
                cmd = reboot_dict[os_mode]
                # inject adb logs
                if actual_state in ["ROS", "MOS", "COS"]:
                    self.inject_device_log("i", "ACS", "Trying to reboot device in %s" % os_mode)

                # disconnect acs only if we was in MOS
                if actual_state == "MOS":
                    # Stop extra threads before sending the reboot command
                    # in case the device reboots whereas extra threads are still active
                    self._stop_extra_threads()

                # overide cmd in case we are in POS
                elif actual_state == "POS" and os_mode == "MOS":
                    cmd = "fastboot reboot"
                # override cmd in case we are in POS and we want to reboot in POS
                elif actual_state == "POS" and os_mode == "POS":
                    cmd = "fastboot reboot-bootloader"

                # Send the reboot cmd
                start_time = time.time()
                self._logger.debug("*** RUN reboot command: {0}, timeout={1}".format(cmd, transition_timeout))
                output = self.run_cmd(cmd, transition_timeout, force_execution=True,
                                      wait_for_response=self._wait_reboot_cmd_returns)

                if not self._wait_reboot_cmd_returns:
                    self.get_logger().info("Wait soft shutdown duration (%ds)..." % self._soft_shutdown_duration)
                    time.sleep(self._soft_shutdown_settle_down_duration)

                transition_timeout_next = transition_timeout - (time.time() - start_time)

                # Disconnect the board in
                if actual_state == "MOS":
                    # Need to send the reboot cmd before disconnecting in case adb over ethernet
                    self.disconnect_board()

                # Consider after reboot that we shall restart the acs agent
                self._acs_agent.is_started = False

                # check reboot result
                if self._wait_reboot_cmd_returns and output[0] == Global.FAILURE:
                    msg = "error happen during reboot command or no reboot command reply received from the device"
                    # Update actual device state
                    actual_state = self.get_boot_mode()
                else:
                    # waiting for transition
                    if wait_for_transition and transition_timeout_next > 0:
                        # Check that we boot in right mode
                        # Handle MOS state
                        if os_mode == "MOS":
                            return_code, _ = self._wait_board_is_ready(boot_timeout=transition_timeout_next,
                                                                       settledown_duration=settledown_duration)

                            if return_code == Global.SUCCESS:
                                actual_state = "MOS"
                                rebooted = True
                            else:
                                actual_state = self.get_boot_mode()
                        else:  # POS, ROS, COS
                            # some time is always available to finalize boot procedure
                            start_time = time.time()
                            while ((time.time() - start_time) < transition_timeout_next) and not rebooted:
                                actual_state = self.get_boot_mode(check_pos_forced=True)
                                if os_mode == actual_state:
                                    self._logger.info("Device has been seen booted in %s after %s seconds" %
                                                      (os_mode, str((time.time() - start_time))))
                                    rebooted = True
                        if rebooted:
                            # inject adb logs
                            self.inject_device_log("i", "ACS", "Device successfully booted in %s" % os_mode)
                        else:
                            # Device fail to reach the wanted mode
                            msg = "Device fail to boot in %s before %d second(s) (current mode = %s)" \
                                  % (os_mode, transition_timeout, actual_state)
                    else:
                        # We do not wait for end of reboot command or time to reboot is already completed
                        # Get the actual state
                        actual_state = self.get_boot_mode()

                        # We do not check the final state but reboot command has succeed
                        rebooted = True

                        msg = "Device reboot command sent (mode requested = %s), " \
                              "we do not wait the end of reboot procedure (current mode = %s)" % (os_mode, actual_state)
                        self.get_logger().warning(msg)

                # Post processing after boot procedure successful or failure
                if actual_state == "MOS":
                    # connect device if mode is MOS
                    self.connect_board()
                    # If the Agent was previously installed, start it
                    self.init_acs_agent()
                elif actual_state in ["ROS", "COS"]:
                    # Try to enable adb root
                    if self._enableAdbRoot:
                        if not self.enable_adb_root():
                            # Only warning message as we are not in MOS
                            self.get_logger().warning(
                                "Device failed to enable adb root, after rebooting in %s" % os_mode)
            else:
                # exist if device state is seen as UNKNOWN
                msg = "Device is in mode %s, cant launch reboot" % actual_state

            if not rebooted:
                self.get_logger().error(msg)
                if not skip_failure:
                    raise DeviceException(DeviceException.OPERATION_FAILED, msg)
                break

        return rebooted

    def _software_pos_transition(self):
        """
        Use reboot to switch to POS OS
        :rtype: bool
        :return: status of operation (True = OK, False = NOK)
        """
        return self.reboot(mode="POS", skip_failure=True)

    def _pos_line_off(self):
        """
        Disable device trig to POS. Stay in POS mode (Stay in flash mode)

        :rtype: int (Global.SUCCESS, Global.FAILURE, Global.BLOCKED)
        :return: result of the switch state request
        """
        # Nothing to disable
        return Global.SUCCESS

    def _pos_line_on(self):
        """
        Enter in the POS mode (used for flashing procedure) by trigging the provisionning line over the board

        :rtype: int (Global.SUCCESS, Global.FAILURE, Global.BLOCKED)
        :return: result of the switch state request

        """
        return_code = Global.FAILURE
        self.get_logger().info("Enable provisioning line: Trig POS mode")

        # Start device
        self._eqts_controller.plug_device_power()
        self._eqts_controller.poweron_device()

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
