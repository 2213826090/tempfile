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

:organization: INTEL NDG SW DEV
:summary: This file implements the LinuxDeviceBase reference device for ACS Dev
:since: 10 March 2014
:author: jreynaux
"""
import time

from LinuxDeviceBase import LinuxDeviceBase
from UtilitiesFWK.Utilities import Global
from ErrorHandling.DeviceException import DeviceException
from Device.DeviceManager import DeviceManager


class EdisonDevice(LinuxDeviceBase):

    def __init__(self, config, logger):
        """
        :type  config: dict
        :param config: Device configuration to use

        :type  logger: logger
        :param logger: Logger to use
        """
        LinuxDeviceBase.__init__(self, config, logger)
        self._os_flash_tool = self.get_config("OSflashTool", "EFT")
        self._use_usb_over_ethernet = self.get_config("usbOverEthernet", "False", "str_to_bool")

        # Block until reboot command returns or not (in some cases, the command is blocked whereas reboot is
        # correctly requested
        self._wait_reboot_cmd_returns = self.get_config("waitSoftRebootCmdReturns", "True", "str_to_bool")

    def reboot(self, mode="MOS", wait_for_transition=False,
               transition_timeout=None, skip_failure=False,
               wait_settledown_duration=False):
        """
        Perform a SOFTWARE reboot on the device.
        By default will bring you to MOS and connect acs once MOS is seen.
        this reboot require that you are in a state where adb or fastboot command can be run.

        :type mode: str
        :param mode: mode to reboot in, support MOS, COS, POS, ROS
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
        msg = "Undefined error while rebooting"
        output = Global.FAILURE
        transition_timeout = self._boot_timeout
        transition_timeout_next = 0
        rebooted = False
        mode = mode.upper()

        # List here command and combo list
        reboot_dict = {"MOS": "/sbin/reboot > /dev/null",
                       "COS": "/bin/systemctl reboot",
                       "ROS": "/sbin/reboot ota > /dev/null"}

        # Wait for device to be ready
        if wait_settledown_duration:
            settledown_duration = self._settledown_duration
        else:
            settledown_duration = None

        # exist if mode is unknown
        if mode not in reboot_dict:
            msg = "unsupported boot mode %s" % str(mode)
            self.get_logger().error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)
        else:
            cmd = reboot_dict[mode]

        self.get_logger().info("Trying to reboot device in %s mode" % str(mode))
        # get actual boot mode
        actual_state = self.get_boot_mode()
        # boot the device
        if actual_state != "UNKNOWN":
            cmd = reboot_dict[mode]

            # Send the reboot cmd
            start_time = time.time()
            output = self.run_cmd(cmd, timeout=transition_timeout)

            if not self._wait_reboot_cmd_returns:
                self.get_logger().info("Wait soft shutdown settle down duration (%ds)..."
                                       % self._soft_shutdown_settle_down_duration)
                time.sleep(self._soft_shutdown_settle_down_duration)

            transition_timeout_next = transition_timeout - (time.time() - start_time)

            # Reconfigure client after reboot
            self._base.get_ssh_api().configure_client()
            # Force login uart after reboot
            if self._base.get_uart_api() is not None:
                self._base.get_uart_api().logged = False

            # check reboot result
            if self._wait_reboot_cmd_returns and output[0] == Global.FAILURE:
                msg = "error happen during reboot command or no reboot command reply received from the device"
                # Update actual device state
                actual_state = self.get_boot_mode()
            else:
                # waiting for transition
                if wait_for_transition and transition_timeout_next > 0:
                    return_code, _ = self._wait_board_is_ready(boot_timeout=transition_timeout_next,
                                                                settledown_duration=settledown_duration)

                    if return_code == Global.SUCCESS:
                        actual_state = "MOS"
                        rebooted = True
                    else:
                        actual_state = self.get_boot_mode()

                    if rebooted:
                        # Log It
                        self._logger("Device successfully booted in %s" % mode)
                    else:
                        # Device fail to reach the wanted mode
                        msg = "Device fail to boot in %s before %d second(s) (current mode = %s)" \
                              % (mode, transition_timeout, actual_state)
                else:
                    # We do not wait for end of reboot command or time to reboot is already completed
                    # Get the actual state
                    actual_state = self.get_boot_mode()
                    # We do not check the final state but reboot command has succeed
                    rebooted = True
                    msg = "Device reboot command sent (mode requested = %s), " \
                          "we do not wait the end of reboot procedure (current mode = %s)" % (mode, actual_state)
                    self.get_logger().warning(msg)

                # Post processing after boot procedure successful or failure
                # if actual_state == "MOS":
                #     # connect device if mode is MOS
                #     self.connect_board()
        else:
            # exist if device state is seen as UNKNOWN
            msg = "Device is in mode %s, cant launch reboot" % actual_state

        if not rebooted:
            self.get_logger().error(msg)
            if not skip_failure:
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        return rebooted

    def __get_device_boot_mode(self, timeout):
        """
        Get the boot mode from ssh

        :rtype: str
        :return: device state : MOS, ROS, COS or UNKNOWN
        """
        boot_mode = "UNKNOWN"
        status, output = self.run_cmd("cat /proc/version", timeout=timeout)
        if output is not None and "Linux" in output and status == Global.SUCCESS:
            boot_mode = "MOS"
        return boot_mode

    def _check_boot_mode(self, mode, timeout=0):
        """
        Check the if the UDT enter in given B{boot} mode before given I{timeout}

        :type mode: str
        :param mode: mode to check
        :see _get_device_boot_mode()

        :type timeout: int
        :param timeout: Timeout in second to wait for reaching given mode.
                        If 0 given to check on loop, only one time check

        :rtype: bool
        :return: True when given mode is reached on time, False otherwise
        """
        msg = "Mode %s not found after %d s!" % (mode, timeout)
        mode_found = False
        start_time = time.time()
        # If timeout is 0, goes in loop only one time
        if timeout <= 0:
            timeout = start_time - time.time() + 1

        while (time.time() - start_time) < timeout:

            time.sleep(1)
            dut_mode = self.__get_device_boot_mode(timeout=1)
            if mode in dut_mode:
                mode_found = True
                seconds = time.time() - start_time
                msg = "Device successfully booted in %s mode after %.2fs" % (mode, seconds)
                self.get_logger().info(msg)
                return mode_found

        self.get_logger().error(msg)
        return mode_found

    def _flash_switch_off(self):
        """
        Switch off the device before doing a Flash procedure
        This can be done either via the power supply or via IO card or both

        :rtype: int (Global.SUCCESS, Global.FAILURE, Global.BLOCKED)
        :return: result of the switch state request
        """
        # If IO_CARD configured, use it by default
        if self._eqts_controller.is_iocard_configured():
            self._logger.info("Reboot using relay card")

            self._eqts_controller.poweroff_device()
            self._eqts_controller.cut_device_power()

            time.sleep(2)

            self._eqts_controller.plug_device_power()
            self._eqts_controller.poweron_device()

            rebooted = True

        else:
            # Mean reboot but when board not available, return status
            rebooted = self.reboot(mode="MOS", wait_for_transition=False)

        if rebooted:
            return_code = Global.SUCCESS
        else:
            return_code = Global.FAILURE

        return return_code

    def _wait_board_is_ready(self, boot_timeout=None, settledown_duration=None):
        """
        Wait until device is ready to be used

        :type boot_timeout: int
        :param boot_timeout: max time in seconds to wait for device

        :type settledown_duration: int
        :param settledown_duration: fixed time waited if requested after boot procedure

        :rtype: int
        :return: Device status - ready to be used (boot procedure OK) or NOT (Global.SUCCESS, Global.FAILURE)

        :rtype: str
        :return: error message
        """
        if boot_timeout is None:
            # Timeout for total boot duration
            boot_timeout = self.get_boot_timeout()

        return_code = Global.FAILURE
        return_message = "Board still not ready after %d" % boot_timeout

        if self._settledown_duration is not None and self._settledown_duration != 0:
            self._logger.info("Wait settle down duration ({0}s)"
                              " to left partitions being formatted at first boot".format(self._settledown_duration))
            time.sleep(self._settledown_duration)

        # if board alive clean known_host file
        if "alive" in self.get_state(check_shell=False):
            self._base.get_ssh_api().clean_user_known_host_file(self._ip_address)

        # Now check device state (ping + echo OK)
        if self._check_state("alive", boot_timeout):
            return_code = Global.SUCCESS
        else:
            self._logger.error(return_message)

        if return_code == Global.SUCCESS:
            # if ok check boot mode
            if self._check_boot_mode("MOS", boot_timeout):
                return_code = Global.SUCCESS
            else:
                self._logger.error(return_message)

        return return_code

    def switch_on(self, boot_timeout=None, settledown_duration=None,
                  simple_switch_mode=False):
        """
        Switch ON the device

        :type boot_timeout: int
        :param boot_timeout: Total time to wait for booting

        :type settledown_duration: int
        :param settledown_duration: Time to wait until start to count for timeout,
                                    Period during which the device must have started.

        :type simple_switch_mode: bool
        :param simple_switch_mode: kept for compatibility

        :rtype: tuple
        :return: (return code, str message)
        """
        self.get_logger().info("Switch on the device...")
        return_code = Global.FAILURE
        # Check if board is not already on
        if self.get_state(check_shell=False) == "alive":
            return_code = Global.SUCCESS
            return_message = "Device already switched on."
            self.get_logger().info(return_message)
            if not self.is_available():
                self.connect_board()
        else:
            # If IO_CARD configured, use it by default
            if self._eqts_controller.is_iocard_configured():
                if not self.__hard_poweron(True):
                    return_message = "Unable to hard switch on the device"
                else:
                    return_code = Global.SUCCESS
                    return_message = "Device successfully powering on"
            else:
                return_message = "device not seen"
                self._logger.error(self.__class__.__name__ + ".switch_on() not yet implemented for boot part")

        return return_code, return_message

    def switch_off(self):
        """
        Switch OFF the device.

        :rtype: tuple
        :return: (return code, str message)
        """
        self.get_logger().info("Switch off the device...")
        return_code = Global.FAILURE
        return_message = "An error occurred during switch_off"

        # If IO_CARD configured, use it by default
        if self._eqts_controller.is_iocard_configured():
            # TODO: This workaround should be fixed by other way in device
            # Need to sync device before hard switch off, to avoid file corruption
            self._logger.debug("Syncing filesystem before hard switch off")
            self.run_cmd("sync")

            if not self.hard_shutdown(True):
                return_message = "Unable to hard switch off the device"
            else:
                return_code = Global.SUCCESS
                return_message = "Device successfully powering off"
        else:
            if not self.soft_shutdown(True):
                return_message = "Unable to soft switch off the device"
            else:
                return_code = Global.SUCCESS
                return_message = "Device successfully powering off"

        return return_code, return_message

    def hard_shutdown(self, wait_for_board_off=False):
        """"
        Perform a hard shutdown and wait for the device is off

        :type wait_for_board_off: bool
        :param wait_for_board_off: Wait for device is off or not after soft shutdown

        :rtype: bool
        :return: If device is off or not
        """
        msg = "Hard shutting down the device..."
        self.get_logger().debug(msg)

        # Initialize variable
        device_off = False

        # Disconnect device before shutting down takes effect
        self.disconnect_board()

        self._eqts_controller.poweroff_device()
        self._eqts_controller.cut_device_power()

        # It should not be needed to wait after hard shutdown request to have effect
        # TO DO: most of the time this time sleep is not needed it shall be tuned in
        # Device_Catalog.xml for every device
        self.get_logger().info("Wait hard shutdown duration (%ds)..." % self._hard_shutdown_duration)
        time.sleep(self._hard_shutdown_duration)

        if wait_for_board_off:
            if self._check_boot_mode("UNKNOWN", 5):
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

    def __hard_poweron(self, wait_for_board_on=False):
        """"
        Perform a hard power on and wait for the device is on or not

        :type wait_for_board_on: bool
        :param wait_for_board_on: Wait for device is ON (MOS) or not after poweron

        :rtype: bool
        :return: If device is on or not
        """
        msg = "Hard power on the device..."
        self.get_logger().debug(msg)

        # Initialize variable
        device_on = False

        self._eqts_controller.plug_device_power()
        self._eqts_controller.poweron_device()

        # It should not be needed to wait after hard power on request to have effect
        # TO DO: most of the time this time sleep is not needed it shall be tuned in
        # Device_Catalog.xml for every device
        self.get_logger().info("Wait powering on duration (%ds)..." % self._boot_timeout)
        time.sleep(self._boot_timeout)

        if wait_for_board_on:
            if self._check_boot_mode("MOS", 5):
                # Device is switched off
                device_on = True
        else:
            # By pass wait for device off.
            # We consider that the device is switched off
            device_on = True

        # Update device boot status according to hard shutdown result
        # Important when this function is called outside switch_off()
        if device_on:
            # self._is_phone_booted = True
            # Connect device after power on
            self.connect_board()

        return device_on

    def use_ethernet_connection(self):
        """
        get if we want to use an ethernet connection to
        communicate with the device

        :rtype: bool
        :returns: true if the option as be set to true on device catalog, false otherwise
        """
        self.get_logger().info("Ethernet not used")

        return False

    def connect_board(self):
        """
        Opens connection to the board.

        :rtype: None
        """
        error_msg = "Unknown error while connecting device"
        self._connection_lock.acquire()

        try:
            self.get_logger().info("Connecting to the device....")
            if not self._is_device_connected:

                # CHeCK SSH CONNECTION
                self._is_device_connected = self._check_state("alive", self._boot_timeout)

                # DEVICE PREPARATION
                if self._is_device_connected:
                    # self._start_extra_threads()
                    # Update time to bench time
                    status, error_msg = self.synchronyze_board_and_host_time()
                    if status == Global.SUCCESS:
                        # Retrieve/update device properties
                        DeviceManager().update_device_properties(self.whoami().get('device'))
                        self.get_logger().info("Device connected !")
                    else:
                        error_msg = "Device failed to synchronize time with host (%s)" % (error_msg)
            else:
                self.get_logger().info("Device already connected !")

        except Exception as error:  # pylint: disable=W0703
            error_msg = str(error)
            self._is_device_connected = False
            # self._stop_extra_threads()
        finally:
            if not self._is_device_connected:
                self.get_logger().debug("Error happen during device connection: %s" % error_msg)
            self._connection_lock.release()

        return self._is_device_connected

    def _check_state(self, state, timeout):
        """
        Check the if the UDT enter in given B{boot} mode before given I{timeout}

        :type state: str
        :param state: mode to check
        :see _get_device_boot_mode()

        :type timeout: int
        :param timeout: Timeout in second to wait for reaching given mode.
                        If 0 given to check on loop, only one time check.

        :rtype: bool
        :return: True when given mode is reached on time, False otherwise
        """
        msg = "State %s not found after %d s!" % (state, timeout)
        state_found = False
        start_time = time.time()
        # If timeout is 0, goes in loop only one time
        if timeout <= 0:
            timeout = start_time - time.time() + 1

        self._base.get_ssh_api().configure_client()
        while (time.time() - start_time) < timeout:
            time.sleep(1)
            dut_state = self.get_state(check_shell=True)
            if state in dut_state:
                state_found = True
                seconds = time.time() - start_time
                msg = "Device successfully switch in %s state after %.2fs" % (state, seconds)
                self.get_logger().info(msg)
                return state_found

        self.get_logger().error(msg)
        return state_found
