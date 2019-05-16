"""
@copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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

@organization: INTEL NDG SW DEV
@summary: This file implements the LinuxDeviceBase reference device for ACS Dev
@since: 10 March 2014
@author: jreynaux
"""
from os import path
import posixpath
import threading
import re
import time
from acs_test_scripts.Device.UECmd.Imp.Linux.Common.Base import Base
import UtilitiesFWK.Utilities as Util
from Device.DeviceController import DeviceController
from Device.DeviceManager import DeviceManager
from Device.Model.DeviceBase import DeviceBase
from UtilitiesFWK.Utilities import Global, AcsConstants
from ErrorHandling.DeviceException import DeviceException
from Core.PathManager import Paths
from Device.Module.DeviceModuleFactory import DeviceModuleFactory
from Device.Model.AndroidDevice.EmbeddedLog import EmbeddedLog
from UtilitiesFWK.Utilities import internal_shell_exec


class LinuxDeviceBase(DeviceBase):
    """
    Linux Base implementation
    """

    OS_TYPE = 'LINUX'

    def __init__(self, config, logger):
        """
        Constructor

        :type  config: dict
        :param config: Device configuration to use

        :type  logger: logger
        :param logger: Logger to use
        """
        DeviceBase.__init__(self, config, logger)

        self._os_version = self.get_config("OSVersion", "Unknown").title()
        self._ip_address = self.get_config("ipAddress", "192.168.2.15")
        self._my_name = path.basename(__file__).split(".")[0]

        # mandatory parameter
        self._prss_pw_btn_time_switch_off = self.get_config("pressPowerBtnTimeSwitchOff", 10, int)
        self._prss_pwr_btn_time_switch_on = self.get_config("pressPowerBtnTimeSwitchOn", 3, int)

        # timeouts
        self._call_setup_timeout = self.get_config("callSetupTimeout", 15, int)
        self._uecmd_default_timeout = int(self.get_config("defaultTimeout", 50, int))
        self._boot_timeout = self.get_config("bootTimeout", 300, int)
        self._settledown_duration = self.get_config("settleDownDuration", 0, int)
        self._usb_sleep_duration = self.get_config("usbSleep", 10, int)
        self._hard_shutdown_duration = self.get_config("hardShutdownDuration", 20, int)
        self._soft_shutdown_duration = self.get_config("softShutdownDuration", 30, int)
        self._soft_shutdown_settle_down_duration = self.get_config("softShutdownSettleDownDuration", 20.0, float)

        # serial_number is the identifier used to communicate with the board (serialno if USB, IP if ethernet)
        self._serial_number = self.get_config("serialNumber", "")

        # Flashing configuration elements
        self._acs_flash_file_path = Paths.FLASH_FILES
        self._flash_time_out = self.get_config("flashTimeout", 600, int)

        # EM parameter
        # self._default_wall_charger = self.get_config("defaultWallCharger", "")

        self._binaries_path = self.get_config("binPath", "/usr/bin/")
        self._userdata_path = self.get_config("userdataPath", "/home/root/")
        self._ext_sdcard_path = self.get_config("sdcard_ext", "/sdcard/")
        self._ext_usbdrive_path = self.get_config("usbdrive_ext", "/usbdrive/")

        # PUPDR parameter
        self._soft_shutdown_cmd = "/sbin/poweroff"

        # misc var
        self._connection_lock = threading.Lock()
        self._is_device_connected = False

        # device info

        # Equipment controller
        self._eqts_controller = DeviceController.DeviceController(self._device_config.device_name, self._device_config,
                                                                  None, self.get_logger())

        # Embedded log feature
        self._embedded_log = EmbeddedLog.EmbeddedLog(self)

        # Start PTI and/or serial logs if required
        # Before establishing first connection to the device, enable PTI and/or Serial logging
        # to capture a maximum of traces. AP and BP logs will be handled further down the line
        if self._embedded_log:
            self._embedded_log.stop("SERIAL")
            time.sleep(1)
            self._embedded_log.start("SERIAL")

        self._base = None

    def initialize(self):
        """
        Initialize the environment of the target.

        @rtype: None
        """
        self._base = Base(self)

    def cleanup(self, campaign_error):
        """
        Clean up the environment of the target.

        :type campaign_error: bool
        :param campaign_error: Notify if errors occured

        :rtype: tuple of int and str
        :return: status and final dut state
        """

        # In this function the device supports:
        # - ON (MOS) state
        # - OFF state
        # We need to handle all those items

        # Disconnect the DUT

        status = False
        final_dut_state = str(self._global_config.campaignConfig.get("finalDutState"))
        self.get_logger().debug("Try to leave device in following final state : %s" % (final_dut_state,))
        try:
            dut_state = self.cleanup_final_state(final_dut_state, campaign_error)
        except (KeyboardInterrupt, SystemExit):
            raise
        except DeviceException as device_io_exception:
            self.get_logger().error("Error happened when leaving device in %s ! (%s)" %
                                    (final_dut_state, device_io_exception.get_error_message()))
            dut_state = Util.DeviceState.UNKNOWN

        if final_dut_state == Util.DeviceState.UNKNOWN:
            self.get_logger().error("Cannot leave device in following final state : %s" % (final_dut_state,))
        else:
            status = True

        self.get_device_controller().release()

        return status, dut_state

    def cleanup_logs(self, campaign_error):
        """
        clean any log of the device

        :type campaign_error: bool
        :param campaign_error: Notify if errors occured
        """
        # Stop embedded logs
        if self._embedded_log:
            self._embedded_log.stop("SERIAL")

    def cleanup_final_state(self, final_dut_state, campaign_error):
        """
        Set final state of the device
        :param campaign_error:
        :param final_dut_state:
        :return: dut state
        """
        if final_dut_state == Util.DeviceState.ON:
            if self.get_boot_mode() != "MOS":
                # Power on the DUT
                status = (self.switch_on()[0] == Global.SUCCESS)
            else:
                self.get_logger().info("Device already power on")
                status = True
            self.cleanup_logs(campaign_error)
        elif final_dut_state == Util.DeviceState.OFF:
            self.cleanup_logs(campaign_error)
            # Power off the DUT
            status = (self.switch_off()[0] == Global.SUCCESS)
        elif final_dut_state == Util.DeviceState.NC:
            status = True
        else:
            self.cleanup_logs(campaign_error)
            # Charging mode cannot be detected on those devices
            # Just switch it off!
            self.get_logger().warning("Unsupported state for this device "
                                      "Trying to switch it off ...")
            status = (self.switch_off()[0] == Global.SUCCESS)

        if not status:
            final_dut_state = Util.DeviceState.UNKNOWN

        return final_dut_state

    def get_device_os_path(self):
        """
        get a module to manipulate device path

        @rtype:  path
        @return: a module to manipulate device path
        """
        return posixpath

    # -----------------------------GETTER FUNCTIONS---------------------------

    def get_device_logger(self):
        """
        Return device logger

        :rtype: object
        :return: device logger instance.
        """
        return self._embedded_log

    def get_device_controller(self):
        """
        Returns the device controller object of the device model
        :rtype: DeviceController object
        :return: device controller
        """

        return self._eqts_controller

    def get_boot_timeout(self):
        """
        Return the boot timeout set in catalog.

        :rtype: int
        :return: boot time needed in seconds.
        """
        return self._boot_timeout

    def get_name(self):
        """
        Returns the phone class name.

        @rtype: str
        @return: the name of the phone class.
        """
        return self._my_name

    def get_device_ip(self):
        """
        Returns the device ip address.

        @rtype: str
        @return: the ip address.
        """
        return self._ip_address

    def get_sdcard_path(self):
        """
        Return the path to the external sdcard
        :rtype: str
        :return: sdcard path
        """
        return self._ext_sdcard_path

    def get_usbdrive_path(self):
        """
        Return the path to the external USB drive
        :rtype: str
        :return: USB drive path
        """
        return self._ext_usbdrive_path

    def get_uecmd_timeout(self):
        """
        Returns the timeout to use for UE Commands.

        @rtype: int
        @return: the timeout for UE Commands.
        """
        return self._uecmd_default_timeout

    def get_sw_release(self):
        """
        Get the SW release of the device.

        :rtype: str
        :return: SW release
        """
        self.get_logger().warning("Deprecated method, you should use device property"
                                  " : device.device_properties.sw_release")
        return self.device_properties.sw_release

    def get_model_number(self):
        """
        Get the Model Number of the device.

        :rtype: str
        :return: Model Number
        """
        self.get_logger().warning("Deprecated method, you should use device property"
                                  " : device.device_properties.model_number")
        return self.device_properties.model_number

    def get_baseband_version(self):
        """
        Get the Baseband Version of the device.
        (Modem sw release)

        :rtype: str
        :return: Baseband Version
        """
        self.get_logger().warning("Deprecated method, you should use device property"
                                  " : device.device_properties.baseband_version")
        return self.device_properties.baseband_version

    def get_kernel_version(self):
        """
        Get the Kernel Version of the device.
        (Linux kernel version)

        :rtype: str
        :return: Kernel Version
        """
        self.get_logger().warning("Deprecated method, you should use device property"
                                  " : device.device_properties.kernel_version")
        return self.device_properties.kernel_version

    def get_device_id(self):
        """
        Return the unique id of the device
        """
        self.get_logger().warning("Deprecated method, you should use device property"
                                  " : device.device_properties.device_id")
        return self.device_properties.device_id

    def get_os_version_name(self):
        """
        Get the version's name of the os.

        :rtype: str
        :return: os version's name
        """
        return self._os_version

    def get_device_info(self):
        """
        Get device information.

        :rtype: dict
        :return a dictionary containing following values (key)
            - Build number (SwRelease)
            - Device IMEI (Imei)
            - Model number (ModelNumber)
            - Baseband version (BasebandVersion)
            - Kernel version (KernelVersion)
            - Firmware version (FwVersion)
            - acs agent version (AcsAgentVersion)
            - hardware variant (BoardType)
        """
        device_info = \
            {"SwRelease": self.device_properties.sw_release,
             "DeviceId": self.device_properties.device_id,
             "Imei": AcsConstants.NOT_AVAILABLE,
             "ModelNumber": self.device_properties.model_number,
             "FwVersion": self.device_properties.fw_version,
             "BasebandVersion": AcsConstants.NOT_AVAILABLE,
             "KernelVersion": self.device_properties.kernel_version,
             "AcsAgentVersion": AcsConstants.NOT_AVAILABLE,
             "BoardType": self.device_properties.board_type}
        return device_info

    def retrieve_device_info(self):
        """
        Retrieve device information in order to fill related global parameters.
        Retrieved values will be accessible through its getter.

            - Build number (SwRelease)
            - Device IMEI
            - Model number
            - Baseband version
            - Kernel version
            - Firmware version
            - acs agent version
            - serial number
            - Store all device properties

        :rtype: dict
        :return: Dict of properties and their associated values
        """
        # retrieve software release if not already done
        self.device_properties.sw_release = self._retrieve_sw_release()

        # retrieve model number if not already done
        self.device_properties.model_number = self._retrieve_model_number()

        # retrieve kernel version if not already done
        self.device_properties.kernel_version = self._retrieve_kernel_version()

        # retrieve firmware version if not already done
        self.device_properties.fw_version = self._retrieve_fw_version()

        if self._serial_number in [None, "None", ""]:
            # Retrieve current value of serial number
            self._serial_number = self.retrieve_device_id()

        # retrieve device unique id (if device is connected over USB, device_id is serial_number)
        if self.device_properties.device_id in [None, "None", "", AcsConstants.NOT_AVAILABLE]:
            self.device_properties.device_id = self._serial_number

        # retrieve board type from device parameters given to ACS command line option
        self.device_properties.board_type = self.get_config("boardType", "ndg_device", str)

        return self.get_device_info()

    def retrieve_device_id(self):
        """
        Retrieve the unique id of the device.

        :rtype: str
        :return: unique id of the device, or None if unknown
        """
        device_id = AcsConstants.NOT_AVAILABLE
        cmd_str = "cat /factory/serial_number"

        status, status_msg = self.run_cmd(cmd_str, self._uecmd_default_timeout, silent_mode=True)

        if status == Global.SUCCESS:
            if status_msg not in (None, "") and status_msg.lower().find("not found") == -1:
                # FZED433D001BL501
                device_id = status_msg.strip()
        else:
            self.get_logger().warning("Impossible to retrieve Device ID")

        return device_id

    def _retrieve_fw_version(self):
        """
        Retrieve the firmware version (from dmesg)

        :rtype: str
        :return: the firmware version, or None if unable to retrieve it
        """
        fw_version = AcsConstants.NOT_AVAILABLE
        cmd_str = "dmesg | grep ifwi | cut -d ':' -f 2"

        status, status_msg = self.run_cmd(cmd_str, self._uecmd_default_timeout, silent_mode=True)

        if status == Global.SUCCESS:
            if status_msg not in (None, "") and status_msg.lower().find("not found") == -1:
                # 237.011
                fw_version = status_msg.strip()
        else:
            self.get_logger().warning("Impossible to retrieve FW version ID")

        return fw_version

    def _retrieve_sw_release(self):
        """
        Retrieve the software version

        :rtype: str
        :return: the sw release, or None if unable to retrieve it
        """
        sw_release = AcsConstants.NOT_AVAILABLE
        cmd_str = "cat /etc/version"

        status, status_msg = self.run_cmd(cmd_str, self._uecmd_default_timeout, silent_mode=True)

        if status == Global.SUCCESS:
            if status_msg not in (None, "") and status_msg.lower().find("not found") == -1:
                # edison-latest-build_build_173_2014-06-18_16-23-20
                sw_release = status_msg.rstrip()
        else:
            self.get_logger().warning("Impossible to retrieve SW release")

        return sw_release

    def _retrieve_kernel_version(self):
        """
        Retrieve the kernel version (from uname)

        :rtype: str
        :return: the kernel version, or None if unable to retrieve it
        """
        kernel_version = AcsConstants.NOT_AVAILABLE
        cmd_str = "uname -r"

        status, status_msg = self.run_cmd(cmd_str, self._uecmd_default_timeout, silent_mode=True)

        if status == Global.SUCCESS:
            if status_msg not in (None, ""):
                kernel_version = status_msg
        else:
            self.get_logger().warning("Impossible to retrieve Kernel Version")

        return kernel_version

    def _retrieve_model_number(self, ):
        """
        Retrieve the model number
        :type properties: dict
        :param properties: a dictionnary containing all system prop (gotten through getprop)
        :rtype: str
        :return: the model number, or None if unable to retrieve it
        """
        model_number = AcsConstants.NOT_AVAILABLE
        cmd_str = "uname -m"

        status, status_msg = self.run_cmd(cmd_str, self._uecmd_default_timeout, silent_mode=True)

        if status == Global.SUCCESS:
            if status_msg not in (None, ""):
                model_number = status_msg
        else:
            self.get_logger().warning("Impossible to retrieve model number")

        return model_number

    def inject_device_log(self, priority, tag, message):
        """
        Logs to device log

        :type priority: str
        :type tag: str
        :type message: str

        :param priority: Priority of og message, should be:
                         v: verbose
                         d: debug
                         i: info
                         w: warning
                         e: error

        :param tag: Tag to be used to identify the log onto the logger
        :param message: Message to be written on log.

        :return: True if command succeed, False otherwise
        :rtype: bool
        """
        return True

    # -----------------------------BOOT/REBOOT/TURN OFF FUNCTIONS-------------------

    def init_device_connection(self, skip_boot_required, first_power_cycle, power_cycle_retry_number):
        """
        Init the device connection.

        Call for first device connection attempt or to restart device before test case execution
        """
        error_msg = ""
        connection_status = False
        power_cycle_occurence = 0
        boot_failure_occurence = 0
        connection_failure_occurence = 0

        # Perform boot sequence only if it is required
        if not skip_boot_required:

            # No need to switch off the board if ACS is just started
            # We switch off between each test cases, but not for the first test case in campaign
            if not first_power_cycle:
                # Switch off the device
                self.switch_off()

            # Loop as long as power cycle phase (boot + connect board) is failed according to power_cycle_retry_number
            while not connection_status and power_cycle_occurence < power_cycle_retry_number:
                # Try to switch ON : boot board + connect board
                # It consists in doing hardware power up + software boot + connection to device
                error_code, error_msg = self.switch_on()

                # We have done one extra power cycle occurrence
                power_cycle_occurence += 1

                # if Boot has failed, redo a complete power cycle phase (switch on +
                # connect board) and before switch off the DUT
                if error_code != Global.SUCCESS:
                    error_msg = "Device has failed to boot ! "
                    self.get_logger().error(error_msg)

                    # Increment tracking variable with number of boot failure attempts
                    boot_failure_occurence += 1
                else:
                    # Check if the connection attempt is OK when boot procedure succeeds
                    # connect_board() is called inside switch_on method only when device is booted
                    if not self.is_available():
                        # Last connection attempt fails
                        # if connection has failed, redo a complete power cycle phase
                        # (switch on + connect board) and before switch off the DUT
                        # Increment tracking variable with number of connection failure attempts
                        connection_failure_occurence += 1

                        error_msg = ("Host has failed to connect "
                                     "to the device : failure occurrence = %s ! " % connection_failure_occurence)
                        self.get_logger().error(error_msg)
                    else:
                        # Whole power cycle phase has succeeded, go out the current method
                        connection_status = True

                # First issue in the power cycle phase (either during boot phase or board connection)
                if not connection_status:
                    # If next power cycle occurrence is the last boot procedure retry, try a hard switch off
                    # else do a normal switch off
                    if power_cycle_occurence > 1:
                        # Hard shutdown the device
                        self.hard_shutdown(wait_for_board_off=True)
                    else:
                        # Switch off the device
                        self.switch_off()
        else:
            # if boot is not required and device is booted then try to connect to the
            # device and update device information (SSN ...)
            if self.get_state() == "alive":
                self.connect_board()
                connection_status = True
            else:
                error_msg = ("[skipBootOnPowerCycle] option set in "
                             "Campaign Config file and device not seen connected to host PC")
                self.get_logger().warning(error_msg)
                # WARNING: We are not sure of the connection but we do not change existing behavior
                connection_status = True

        return connection_status, error_msg, power_cycle_occurence, boot_failure_occurence, connection_failure_occurence

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
        return_code = Global.FAILURE
        if self.get_state(check_shell=False) == "alive":
            return_code = Global.SUCCESS
            return_message = "Device already switched on."
            self.get_logger().info(return_message)
            if not self.is_available():
                self.connect_board()
        else:
            return_message = "device not seen"
            self._logger.error(self.__class__.__name__ + ".switch_on() not yet implemented for boot part")

        return return_code, return_message

    def switch_off(self):
        """
        Switches the board off.

        :rtype: tuple
        :return: (return code, str message)
        """
        self.get_logger().info("Switch off the board...")
        self.disconnect_board()
        self._logger.error(self.__class__.__name__ + ".switch_off() not yet implemented for the shutdown part")
        return Global.SUCCESS, "No errors"

    def soft_shutdown_cmd(self):
        """
        Soft Shutdown the device, if permissions are ok (root)
        """
        status = False

        try:
            result, output = self.run_cmd(self._soft_shutdown_cmd,
                                          self._uecmd_default_timeout,
                                          force_execution=True,
                                          wait_for_response=True)

            if result == Global.SUCCESS:
                status = True
            else:
                self.get_logger().error("Soft shutdown command failed! (%s)" % (str(output)))

        except Exception as error:  # pylint: disable=W0703
            self.get_logger().debug("Soft shutting down the device failed: %s" % str(error))

        return status

    def soft_shutdown(self, wait_for_board_off=False):
        """"
        Perform a soft shutdown and wait for the device is off

        :type wait_for_board_off: bool
        :param wait_for_board_off: Wait for device is off or not after soft shutdown

        :rtype: bool
        :return: If device is off or not
        """
        msg = "Soft shutting down the device..."
        self.get_logger().info(msg)

        # Initiliaze variable
        device_off = False

        # if device is already OFF do not do soft shutdown procedure
        if self.is_available():

            # Call device soft shutdown
            if self.soft_shutdown_cmd():
                self.disconnect_board()

                # self.run_cmd("poweroff")

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
                state = self.get_state(check_shell=True)
                if "alive" in state:
                    self._is_device_connected = True

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
                        error_msg = "Device failed to synchronize time with host (%s)" % error_msg
            else:
                self.get_logger().info("Device already connected !")

        except Exception as error:
            error_msg = str(error)
            self._is_device_connected = False
            # self._stop_extra_threads()
        finally:
            if not self._is_device_connected:
                self.get_logger().debug("Error happen during device connection: %s" % error_msg)
            self._connection_lock.release()

        return self._is_device_connected

    def disconnect_board(self):
        """
        Disconnect the device.
        """
        self._connection_lock.acquire()
        try:
            self.get_logger().info("Disconnecting the device...")
            # to disconnect acs we just need that acs is connected.
            if self.is_available():
                # Stop extra threads
                # self._stop_extra_threads()
                self.get_logger().info("Disconnected!")
            else:
                self.get_logger().info("Device already disconnected !")
            self._is_device_connected = False
        except Exception as error:
            self.get_logger().debug("Error happen during device disconnection: %s" % str(error))
        finally:
            self._connection_lock.release()

    def synchronyze_board_and_host_time(self):
        """
        Synchronize the device time and the acs host time
        """
        result, return_msg = self.run_cmd("date --set=@%d" % int(time.time()), 1, force_execution=True)
        if result == Global.SUCCESS:
            _, device_time = self.run_cmd("date +\'%d/%m %T\'", 1, force_execution=True, silent_mode=True)
            # Log device time for MPTA sync features
            # Please do not erase/change this log message
            # w/o informing MPTA dev guys
            self.get_logger().info("DeviceTime: %s" % device_time)
        return result, return_msg

    # ----------------------------STATE CHECK FUNCTIONS-----------------------

    def is_available(self):
        """
        Checks if the board can be used.

        :return: availability status
        :rtype: bool
        """
        return self._is_device_connected

    def is_booted(self):
        """
        Returns a C{bool} indicating whether the phone is booted or not.

        :rtype: bool
        :return: the I{booted} status of the phone.
        """
        return "alive" in self.get_state()

    def get_state(self, check_shell=False):
        """
        get the device state.

        :type  check_shell: bool
        :param check_shell: kept for compatibility only, not used

        :rtype: string
        :return: device state : alive, unknown, offline
        """
        current_state = "alive"
        self.get_logger().info("Getting device current_state ...")
        cmd = "ping -c 2 -w 5 %s" % self._ip_address

        status, value = internal_shell_exec(cmd=cmd, timeout=5, silent_mode=False)
        try:
            if status == Global.SUCCESS and value is not None:
                output_match = re.search("received, ([0-9]+\.?[0-9]*)% packet loss", value.lower())
                if output_match is not None and float(str(output_match.group(1))) <= 50:
                    # self._logger.debug("It's alive!!!")
                    if check_shell:
                        try:
                            self._logger.debug("ping answered, check if the DUT answer to SSH or UART")

                            cmd = "echo {0}".format(id(self))
                            status, output = self._base._internal_exec(cmd, timeout=1, raise_exception=True)
                            if not output == str(id(self)):
                                raise DeviceException(DeviceException.INTERNAL_EXEC_ERROR, "offline")
                        except DeviceException as e:
                            if e.get_specific_message() == "offline":
                                raise e
                            else:
                                # ping ok, but no answer from ssh or uart, state unknown
                                self._logger.debug("ping answered, but no reply on ssh or uart.")
                                raise DeviceException(DeviceException.INTERNAL_EXEC_ERROR, "unknown")
            else:
                # No ping answered, board should offline or disconnected
                if not check_shell:
                    self._logger.debug("No ping answered, board seems offline")
                    raise DeviceException(DeviceException.INTERNAL_EXEC_ERROR, "offline")
                else:
                    try:
                        self._logger.debug("No ping answered, try to contact the DUT on SSH or UART")
                        cmd = "echo {0}".format(id(self))
                        status, output = self._base._internal_exec(cmd, timeout=1, raise_exception=True)
                        if not output == str(id(self)):
                            raise DeviceException(DeviceException.INTERNAL_EXEC_ERROR, "alive")

                    except DeviceException as e:
                        # ping not ok, but answer from uart, state offline
                        self._logger.debug("no ping answered and no reply on ssh or uart.")
                        if e.get_specific_message() == "alive":
                            raise e
                        else:
                            raise DeviceException(DeviceException.INTERNAL_EXEC_ERROR, "offline")


        except DeviceException as e:
            current_state = e.get_specific_message()

        self._logger.debug("Device on \"{0}\" state.".format(current_state))
        return current_state

    def get_boot_mode(self):
        """
        get the boot mode from ssh

        :rtype: string
        :return: device state : MOS, ROS, POS, DNX, COS or UNKNOWN
        """
        self._logger.debug("Getting boot mode ...")
        current_mode = "UNKNOWN"
        retry = 2
        # Implement a retry mechanism as on some benches as on first
        # run it can return unknown

        while retry > 0 and current_mode.lower() == "unknown":
            retry -= 1
            status, output = self._base._internal_exec("cat /proc/version", timeout=1, raise_exception=False, use_uart=False)
            if status and "Linux" in output:
                current_mode = "MOS"

        self.get_logger().debug("Device on \"{0}\" boot mode.".format(current_mode))
        return current_mode

    def flash(self, flash_file=None, timeout=None):
        """
        Flash a new software on the device target

        :type  flash_file: list
        :param flash_file: paths to flash files
        :type  timeout: int
        :param timeout: Flash execution timeout in s
        :rtype: int
        :return: result of the flash procedure (Global.SUCCESS, Global.FAILURE, Global.BLOCKED)
        :rtype: str
        :return: Output message status
        """

        # Parse flash file and TimeOut arguments
        # Take default values for flash file and TimeOut (from Device Catalog, ...)
        if flash_file is None:
            flash_file = self._acs_flash_file_path
            self.get_logger().warning(
                "Flash file is not defined, using value from ACS -f option ({0})".format(flash_file))

        if timeout is None:
            timeout = self._flash_time_out
            self.get_logger().warning(
                "Flash timeout is not defined, using value from Device_Catalog ({0})".format(timeout))
        elif not str(timeout).isdigit() or timeout < 0:
            # Check if we use an integer and check if not a negative value
            timeout = self._flash_time_out
            self.get_logger().warning(
                "Flash timeout should be an integer > 0, using value from Device_Catalog ({0})".format(timeout))
        else:
            # Value is good: Inform user on the flash timeout value taken
            self.get_logger().info("Flash timeout defined by the user used ({0} seconds)".format(timeout))

        # Instantiates flash manager for the device (os flashing is enabled)
        # It will check all the flash tools availability
        flash_module = DeviceModuleFactory.create("FlashModule", self, self._global_config)
        flash_module.init(flash_file)

        # Flash the platform
        # It will check first the flash files availability + retrieve build properties from flash files + do the flash
        self.get_logger().info("Flashing file %s onto %s target ...", flash_file, self.get_phone_model())
        return_code = flash_module.flash(timeout)

        if return_code != Global.SUCCESS:
            verdict_msg = "Flashing device has failed!..Exit flash!"
            self.get_logger().error(verdict_msg)

            # we do not know the state of the device,
            # we do not change the state, cause possible new flash after, or flash
            # failure trig a CRITICAL failure and campaign exit
        else:
            verdict_msg = "Device successfully flashed!"
            self.get_logger().info(verdict_msg)
            # Update device info with build info (more info may be available after flash)
            # self.update_device_info_with_build_info(flash_module.device_properties)

        return return_code, verdict_msg

    # -----------------------------PUSH/PULL/DUT EMBEDDED CODE EXECUTION--------------------

    def push(self, src_path, dest_path, timeout=None):
        """"
        This function push files onto target from a given folder

        :type src_path: str
        :param src_path: the source path , eg c:\acs\scripts\from.txt

        :type dest_path: str
        :param dest_path: the destination path on target , eg c:\documents\to.txt

        :rtype: tuple
        :return: Nothing if operation success, otherwise a String containing the full error message
        """
        output = Global.FAILURE, "Unable to push file"
        # Copying onto target
        if path.isfile(src_path):
            if timeout is None:
                item_size = path.getsize(src_path)
                timeout = int(item_size / 1024) + 20
            if self._push_file(src_path, dest_path, timeout):
                output = Global.SUCCESS, "File {0} pushed".format(path.basename(src_path))
        else:
            msg = "Cannot push '%s', file does not exist." % src_path
            self.get_logger().error(msg)
            output = Global.FAILURE, msg

        return output

    def pull(self, remotepath, localpath, timeout=0):
        """
        Pull a file from remote path to local path

        :type remotepath: str
        :param remotepath: the remote path , eg /acs/scripts/from.txt
        :type localpath: str
        :param localpath: the local path , eg /to.txt
        :type timeout: float or None
        :param timeout: Set a timeout in seconds on blocking read/write operations.
        timeout=0.0 is equivalent to set the channel into a no blocking mode.
        timeout=None is equivalent to set the channel into blocking mode.
        """
        output = Global.FAILURE, "Unable to pull file"
        # Copying onto target
        if self._pull_file(remotepath, localpath, timeout):
            output = Global.SUCCESS, "File {0} retreived".format(remotepath)
        return output

    def run_cmd(self, cmd, timeout=30,
                force_execution=False,
                wait_for_response=True,
                silent_mode=False):
        """
        Execute the input command and return the result message
        If the timeout is reached, return an exception

        :type  cmd: string
        :param cmd: cmd to be run

        :type  timeout: integer
        :param timeout: Script execution timeout in ms

        :type force_execution: bool
        :param force_execution: Force execution of command
                    without check phone connected (dangerous)

        :type wait_for_response: bool
        :param wait_for_response: Wait response from the device before stating on command

        :type silent_mode: bool
        :param silent_mode: Flag indicating some information are display as log.

        :return: Execution status & output string
        :rtype: Integer & String
        """
        result = Global.FAILURE
        msg = "Cannot run the cmd, device not connected!"

        result, msg = self._base._internal_exec(cmd=cmd, timeout=timeout, silent_mode=False, raise_exception=False,
                                                use_uart=False, wait_for_response=wait_for_response)

        msg = msg.rstrip("\r\n")

        if "sh: " + cmd + ": not found" in msg:
            # Ugly patch
            # Sometime the ssh command succeed but in fact sh command failed.
            result = False

        if result:
            ret = Global.SUCCESS
        else:
            ret = Global.FAILURE

        return ret, msg

    # ----------------------------- DEVICE PROPERTIES --------------------

    @property
    def multimedia_path(self):
        """
        Return the path to the local multimedia files
        :rtype: str
        :return: local multimedia directory
        """
        return self._userdata_path

    @property
    def binaries_path(self):
        """
        Return the path to the local binaries folder
        :rtype: str
        :return: local binaries directory
        """
        return self._binaries_path

    # ----------------------------- SPECIFIC LINUX DEVICE CODE --------------------

    def _push_file(self, src_path, dest_path, timeout=30):
        """
        Push file on dut via SCP

        :param src_path: str
        :param src_path: Source path of file

        :type dest_path: str
        :param dest_path: Destination path on DUT

        :type timeout: int
        :param timeout: Timeout in second

        :rtype: bool
        :return: Boolean indicating operation succeed or not
        """
        return self._base.get_ssh_api().ssh_push_file(src_path=src_path, dest_path=dest_path, timeout=timeout)

    def _pull_file(self, src_path, dest_path, timeout=30):
        """
        Push file on dut via SCP

        :param src_path: str
        :param src_path: Source path of file

        :type dest_path: str
        :param dest_path: Destination path on DUT

        :type timeout: int
        :param timeout: Timeout in second

        :rtype: bool
        :return: Boolean indicating operation succeed or not
        """
        return self._base.get_ssh_api().ssh_pull_file(src_path=src_path, dest_path=dest_path, timeout=timeout)
