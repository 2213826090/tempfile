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

:organization: INTEL PCCG
:summary: This file implements the IMIN-LEGACY BYTM64 TCP-IP target platforms
:since: 12/18/2014
:author: syang5
"""
import time
from Device.Model.AndroidDevice.AsusT100Device import AsusT100Device
from UtilitiesFWK.Utilities import Global
from Utilities.UartUtilities import UartUtilities
import serial

class Bytm64CrbLlPDevice(AsusT100Device):

    """
    GMIN ADB over USB target platforms implementation
    """

    def __init__(self, config, logger):
        """
        Constructor
        """
        AsusT100Device.__init__(self, config, logger)
        self.__pos_on_commands = "<downarrow>"
        self.uart_api = UartUtilities(logger)
        self.serialPort = self.get_config("serialPort", "")
        self.baudrate = self.get_config("serialBaudRate", 115200, int)

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

        self._eqts_controller.poweron_device()
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
        Use keyboard emulator to switch to POS OS
        :rtype: bool
        :return: status of operation (True = OK, False = NOK)
        """
        status = False
        try:
            end_time = time.time() + 10
            ser = None
            if self.serialPort != '':
                ser = serial.Serial(port=self.serialPort,
                                    baudrate=self.baudrate)

            while time.time() < end_time:
                self._eqts_controller.write_keyboard_commands(self.__pos_on_commands)
                time.sleep(0.2)
                if self.serialPort != '' and ser is not None and ser.isOpen():
                    ser.write('f\r\n')

            if self.serialPort != '' and ser is not None:
                ser.close()
                ser = None

            time.sleep(10)
            # Test if we we are in POS mode
            end_time = time.time() + self._settledown_duration
            while time.time() < end_time:
                if self._check_pos_mode() == "POS":
                    status = True
                    break
                else:
                    time.sleep(2)
        except Exception as e:
            self._logger.debug(str(e))
        return status

    def _software_pos_transition(self):
        """
        Use reboot to switch to POS OS
        :rtype: bool
        :return: status of operation (True = OK, False = NOK)
        """
        reboot_result = self.reboot(mode="POS", skip_failure=True)
        if reboot_result:
            self.disconnect_board()
            self._is_phone_booted = False
        return reboot_result

    def _check_pos_mode(self):
        """
            check if device is in POS

            :rtype: str
            :return: device state is POS or UNKNOWN
        """
        current_state = "UNKNOWN"
        output = self.run_cmd("fastboot devices", 10, True)
        mode = (output[1].strip()).lower()
        if mode.endswith("fastboot"):
            current_state = "POS"

        self._logger.debug("_check_pos_mode: %s" % current_state)

        return current_state

    def _pos_line_on(self):
        """
        Enter in the POS mode (used for flashing procedure) by trigging the provisionning line over the board

        :rtype: int (Global.SUCCESS, Global.FAILURE, Global.BLOCKED)
        :return: result of the switch state request

        """
        return_code = Global.FAILURE
        self.get_logger().info("Enable provisionning line: Trig POS mode")

        # Start device
        #self._eqts_controller.plug_device_power()
        self._eqts_controller.poweron_device()
        in_pos_state = self.__hardware_pos_transition()
        i = 0

        while not in_pos_state and i < 4:
            self.hard_shutdown(wait_for_board_off=True)
            #self._eqts_controller.plug_device_power()
            self._eqts_controller.poweron_device()
            in_pos_state = self.__hardware_pos_transition()
            i += 1

        # Switch to POS for flashing
        if not in_pos_state:
            time.sleep(self._settledown_duration)
            self._logger.debug("Boot in POS through hardware means failed, try to reboot in POS by software method")
            if self._software_pos_transition():
                return_code = Global.SUCCESS

            # Switch off and switch on the device
            elif self.hard_shutdown(wait_for_board_off=True):
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

    def get_boot_mode(self):
        self._logger.debug("get_boot_mode called")
        return AsusT100Device.get_boot_mode(self)

    def __bringup_dhcp_thru_serial(self):
        if self.serialPort != '':
            self.uart_api.configure_port(port=self.serialPort, baudrate=self.baudrate)
            self.uart_api.uart_run_cmd(command='su -c "/system/bin/netcfg" eth0 dhcp')

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
                if device_state == "offline":
                    self.get_logger().info("Device is in offline state")
                    self._eqts_controller.poweroff_device()
                self._eqts_controller.plug_device_power()
                self._eqts_controller.poweron_device()
                time.sleep(15)

                # Ask for device state
                device_state = self.get_state()
                start_time = time.time()
                end_time = start_time + boot_timeout
                iteration = 1

                while device_state != "alive" and time.time() < end_time:
                    device_state = self.get_state()
                    time.sleep(1)
                    # dhcp service might be stopped on device side. try to bring up
                    if device_state != "alive" and (iteration % 5 == 0):
                        self.get_logger().info("Trying to bring up dhcp during boot time")
                        self.__bringup_dhcp_thru_serial()
                    iteration += 1

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
                    if self.get_state() == "offline":
                        self.get_logger().info("Device is in offline state")
                        self._eqts_controller.poweroff_device()
                        self._eqts_controller.plug_device_power()
                        self._eqts_controller.poweron_device()
                        time.sleep(self._settledown_duration)
                    # dhcp service might be stopped on device side. try to bring up
                    if self.get_state() != 'alive':
                        self.get_logger().info("Trying to bring up dhcp after boot")
                        self.__bringup_dhcp_thru_serial()

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
        self.get_logger().debug("_wait_board_is_ready.....")
        if boot_timeout is None:
            # Timeout for total boot duration
            boot_timeout = self.get_boot_timeout()

        # By default we do wait settledown_duration after the boot procedure only if required by the user
        # if user needs a settledown duration after boot procedure it shall be set to a value (not None)

        # Loop while boot time not exceeded & read device state
        timer = 0
        status = Global.FAILURE
        return_code = Global.FAILURE
        return_message = ""

        while status == Global.FAILURE and timer < boot_timeout:
            # pylint: disable=W0702
            # at this method, the device is not yet available
            # we can have unexpected behavior on run_cmd
            # agree to catch all exception
            t_0 = time.time()
            try:
                device_state = self.get_state()
                if device_state == 'alive':
                    status = Global.SUCCESS
                elif device_state == 'offline':
                    self.get_logger().info("Device is in offline state")
                    self._eqts_controller.poweroff_device()
                    self._eqts_controller.plug_device_power()
                    self._eqts_controller.poweron_device()
                    time.sleep(self._settledown_duration)
                elif self._is_tcp_port_alive(self._ip_address, self._adb_port) is False:
                    self.get_logger().debug("trying to start dhcp service")
                    self.__bringup_dhcp_thru_serial()
                elif device_state == 'unknown':
                    self.get_logger().info("Device is in unknown state. adb kill-server called.")
                    self.run_cmd("adb kill-server", 10, True)
            except (KeyboardInterrupt, SystemExit):
                raise
            except Exception as error:
                # Inform user of the exception
                self.get_logger().debug("Exception while booting the device : %s", str(error))
                # Wait 1 seconds to avoid relaunching the command multiple times
                time.sleep(1)

            t_1 = time.time()
            timer += t_1 - t_0

        if timer < boot_timeout and status == Global.SUCCESS:
            return_code, return_message = self._finalize_os_boot(timer, boot_timeout, settledown_duration)
        else:
            # Device still not booted
            # Device not connected or adb connection issue
            return_message = "Device has failed to boot after %d seconds! " % boot_timeout
            return_message += "Device not detected or adb connection issue"
            self.get_logger().warning(return_message)

        return return_code, return_message
