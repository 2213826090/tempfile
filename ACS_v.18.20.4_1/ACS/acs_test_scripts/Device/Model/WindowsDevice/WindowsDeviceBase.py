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
:summary: This file implements the WindowsDeviceBase reference phone for ACS Dev
:since: 04 Jan 2011
:author: sfusilie, vgomberx, dgonza4x
"""

import threading
import ntpath
import time
from os import path
from time import sleep
from Device.DeviceLogger.Win8Logger.Win8Logger import Win8Logger
from Device.DeviceManager import DeviceManager
from Device.Model.DeviceBase import DeviceBase
from UtilitiesFWK.Utilities import Global, Error
import UtilitiesFWK.Utilities as Util
from UtilitiesFWK.Utilities import is_number
from ErrorHandling.DeviceException import DeviceException
from Device.DeviceController import DeviceController
from Services.CommandResult import CommandResult
from Services.CommandResultException import CommandResultException
from Services.WindowsService import WindowsService
from Services.LoggerService import LoggerService


class WindowsDeviceBase(DeviceBase):

    """
    Windows Phone Base implementation
    """

    OS_TYPE = 'WINDOWS'

    def __init__(self, config, logger):
        """
        Constructor

        :type  phone_name: string
        :param phone_name: Name of the current phone(e.g. PHONE1)
        """
        DeviceBase.__init__(self, config, logger)
        self._error = Error()
        self._device_ip = self.config.get("ipAddress", "localhost")
        self._my_name = path.basename(__file__).split(".")[0]

        self._device_logger = None
        self._init_logger()
        # mandatory parameter
        self._prss_pw_btn_time_switch_off = self.get_config("pressPowerBtnTimeSwitchOff", 10, int)
        self._prss_pwr_btn_time_switch_on = self.get_config("pressPowerBtnTimeSwitchOn", 3, int)
        self._call_setup_timeout = self.get_config("callSetupTimeout", 15, int)
        self._uecmd_default_timeout = self.get_config("defaultTimeout", 50, int)
        self._boot_timeout = self.get_config("bootTimeout", 300, int)
        self._soft_shutdown_duration = self.get_config("softShutdownDuration", 120, int)
        self._hard_shutdown_duration = self.get_config("hardShutdownDuration", 120, int)
        self._settledown_duration = self.get_config("settleDownDuration", 0, int)
        self._usb_sleep_duration = self.get_config("usbSleep", 10, int)
        # EM parameter
        self._default_wall_charger = self.get_config("defaultWallCharger", "")
        # CWS parameter
        self._wifi_adapter_name = self.get_config("WiFiAdapterName", "Wi-Fi", str)
        # windows logger parameter
        self.__logger_filters = self.get_config("kernelLogFilter", "Verbose", str)
        # user files directory
        self._userdata_path = self.get_config("userdataPath", "C:\\Users\\Public\\Documents")
        self._binaries_path = self.get_config("binPath", "C:\\Users\\Public\\Documents")
        self._cellular_network_interface = self.get_config("cellularNetworkInterface", "Mobile broadband")
        # misc var
        self._connection_lock = threading.Lock()
        self._is_acs_connected = False
        self._eqts_controller = DeviceController.DeviceController(config.device_name, self.config,
                                                                  self.get_em_parameters(), self.get_logger())
        # instantiate the Windows web service used to execute device commands
        service_port = self.get_config("servicePort", "8080", str)
        http_proxy = self.get_config("httpProxy", "", str)
        proxies = {"http": http_proxy}
        self.__windows_service = WindowsService(
            self._device_ip, service_port, proxies, logger, self._uecmd_default_timeout)

        logger_port = self.get_config("loggerPort", "8082", str)
        # instantiate the Logger web service used to inject/retrieve logs
        self.__logger_service = LoggerService(
            self._device_ip, logger_port, proxies, logger, 10)

    def initialize(self):
        """
        Initialize the environment of the target.

        :rtype: None
        """
        pass

    def get_device_os_path(self):
        """
        get a module to manipulate device path

        :rtype:  path
        :return: a module to manipulate device path
        """
        return ntpath

    # -----------------------------GETTER FUNCTIONS---------------------------

    def get_name(self):
        """
        Returns the phone class name.

        :rtype: str
        :return: the name of the phone class.
        """
        return self._my_name

    def get_uecmd_timeout(self):
        """
        Returns the timeout to use for UE Commands.

        :rtype: int
        :return: the timeout for UE Commands.
        """
        return self._uecmd_default_timeout

    def retrieve_serial_number(self):
        """"
        Returns the serial number of the device.

        :rtype: str
        :return: The serial number of the phone
        """
        return self._device_ip

    def get_sw_release(self):
        """
        Returns the software release within a C{tuple}.

        :rtype: tuple
        :return: the command status & software release
        """
        # Copying onto target
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
        return self.device_properties.kernel_version

    def get_imei(self):
        """
        Get the IMEI of the device.
        (International Mobile Equipment Identity)

        :rtype: str
        :return: Imei number
        """
        self.get_logger().warning("Deprecated method, you should use device property"
                                  " : device.device_properties.imei")
        return self.device_properties.imei

    def get_fw_version(self):
        """
        Get the Firware Version of the device.
        (International Mobile Equipment Identity)

        :rtype: str
        :return: Firmware version
        """
        self.get_logger().warning("Deprecated method, you should use device property"
                                  " : device.device_properties.fw_version")
        return self.device_properties.fw_version

    def get_cellular_network_interface(self):
        """
        Return the ip interface of celluar network

        this interface can be obtain from the command "busybox ifconfig"
        :rtype: str
        :return: telephony ip interface
        """
        return self._cellular_network_interface

    def get_device_info(self):
        """
        Get device information.

        :rtype: dict
        :return a dictionnary containing following values (key)
            - Build number (SwRelease)
            - Device IMEI (Imei)
            - Model number (ModelNumber)
            - Baseband version (BasebandVersion)
            - Kernel version (KernelVersion)
            - Firmware version (FwVersion)
            - acs agent version (AcsAgentVersion)
        """
        device_info = \
            {"SwRelease": self.device_properties.sw_release,
             "DeviceId": self.device_properties.device_id,
             "Imei": self.device_properties.imei,
             "ModelNumber": self.device_properties.model_number,
             "FwVersion": self.device_properties.fw_version,
             "BasebandVersion": self.device_properties.baseband_version,
             "KernelVersion": self.device_properties.kernel_version,
             "AcsAgentVersion": self.device_properties.acs_agent_version}
        return device_info

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
        Switch ON the device.
        if the switch on succeed, a data connection with the board will be started
        ( trough USB, ethernet... Depending of your device configuration) and we will start ACS loggers.

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
        # first check that board is booted.
        if self.is_booted():
            return_code = Global.SUCCESS
            return_message = "Device already switched on."
            self.get_logger().info(return_message)
            if not self.is_available():
                self.connect_board()
        else:
            # if no then try to boot it
            # as switch is not fully implemented then
            # at least we will connect a data cable to see if board is already booted
            self._eqts_controller.connect_usb_host_to_dut()
            sleep(self._usb_sleep_duration)
            self._eqts_controller.plug_device_power()
            self._eqts_controller.poweron_device()
            # Wait enough time for power on have effect
            self.get_logger().info("Wait boot timeout duration (%ds)..." % self._boot_timeout)
            sleep(self._boot_timeout)
            # if board is booted then try to connect acs logger.
            if self.is_booted():
                return_code = Global.SUCCESS
                return_message = "Device already switched on."
                self.get_logger().info(return_message)
                if not self.is_available():
                    self.connect_board()
            else:
                # Case: Windows Recovery mode wait the recovery time
                sleep(120)
                if self.hard_shutdown(True):
                    device_off = True
                    self._eqts_controller.poweron_device()
                    self.get_logger().info("Wait boot timeout duration (%ds)..." % self._boot_timeout)
                    sleep(self._boot_timeout)
                    if self.is_booted():
                        return_code = Global.SUCCESS
                        return_message = "Device already switched on."
                        self.get_logger().info(return_message)
                        if not self.is_available():
                            self.connect_board()
                    else:
                        return_message = "device not seen"
                        self._logger.error(self.__class__.__name__ + ".switch_on() device not booted after "+str(self._boot_timeout)+" sec")
                else:
                    return_message = "Hard shutting down the device in the recovery mode FAIL"
                    self._logger.error(self.__class__.__name__ + "Hard shutting down the device in the recovery mode FAIL after 120 sec")

        return return_code, return_message

    def switch_off(self):
        """
        Switches the board off.

        :rtype: tuple
        :return: (return code, str message)
        """
        self.get_logger().info("Switching off the device...")

        if self.is_booted():

            # Initialize variables
            device_off = False
            return_code = Global.FAILURE
            return_msg = ""

            # Let's try to perform a soft shutdown in any case
            # if it fails then try a hard shutdown
            if self.soft_shutdown(True):
                device_off = True
            elif self.hard_shutdown(True):
                device_off = True
            else:
                return_msg = "Failed to shutdown the device (soft or hard) !"
                self.get_logger().error(return_msg)

            if device_off:
                return_code = Global.SUCCESS
                return_msg = "Device successfully switched off"
                self.get_logger().info(return_msg)
        else:
            return_code = Global.SUCCESS
            return_msg = "Device already switched off"
            self.get_logger().info(return_msg)

        return return_code, return_msg

    def soft_shutdown(self, wait_for_board_off=False):
        """"
        Perform a soft shutdown and wait for the device is off

        :type wait_for_board_off: boolean
        :param wait_for_board_off: Wait for device is off or not after soft shutdown

        :rtype: boolean
        :return: If device is off or not
        """
        msg = "Soft shutting down the device..."
        self.get_logger().info(msg)
        self.inject_device_log("i", "ACS", msg)
        # Initiliaze variable
        device_off = False
        # if device is already OFF do not do soft shutdown procedure
        if self.is_available():
            # Call device soft shutdown
            if self.soft_shutdown_cmd():
                # Disconnect device before shutting down takes effect
                self.disconnect_board()
                # Wait for device is off
                if wait_for_board_off:
                    # Wait enough time for soft shutdown to have effect
                    self.get_logger().info("Wait soft shutdown duration (%ds)..." % self._soft_shutdown_duration)
                    time.sleep(self._soft_shutdown_duration)
                    if not self.is_booted():
                        # Device is switched off
                        device_off = True
                else:
                    # By pass wait for device off.
                    # We consider that the device is switched off
                    device_off = True
        return device_off

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
        self._eqts_controller.poweroff_device()
        self._eqts_controller.cut_device_power()
        # It should not be needed to wait after hard shutdown request to have effect
        # TO DO: most of the time this time sleep is not needed it shall be tuned in
        # Device_Catalog.xml for every device
        self.get_logger().info("Wait hard shutdown duration (%ds)..." % self._hard_shutdown_duration)
        time.sleep(self._hard_shutdown_duration)
        if wait_for_board_off:
            if not self.is_booted():
                # Device is switched off
                device_off = True
        else:
            # By pass wait for device off.
            # We consider that the device is switched off
            device_off = True
        return device_off

    def connect_board(self):
        """
        Opens connection to the board.

        :rtype: None
        """
        self._connection_lock.acquire()
        try:
            # to connect acs we need that board is alive and acs is not already connected
            self.get_logger().info("Connection to the device....")
            if self.get_state() == "alive":
                if not self.is_available():

                    # Update time to bench time
                    status, error_msg = self.synchronyze_board_and_host_time()
                    if status != Global.SUCCESS:
                        raise DeviceException(DeviceException.OPERATION_FAILED,
                                              "Device failed to synchronize time with host (%s)" % (error_msg,))

                    # start the logger on the remote target
                    self._start_extra_threads()
                    self._is_acs_connected = True
                    # Retrieve/update device properties
                    DeviceManager().update_device_properties(self.whoami().get('device'))
                else:
                    self.get_logger().info("Device already connected !")
            else:
                self.get_logger().info("cannot connect acs, DUT is not seen")

        except Exception as error:
            self.get_logger().debug("Error happen during device connection: %s" % str(error))
            self._is_acs_connected = False
            # Stop extra threads
            self._stop_extra_threads()

        finally:
            self._connection_lock.release()

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
                self._stop_extra_threads()
                self.get_logger().info("Disconnected!")
            else:
                self.get_logger().info("Device already disconnected !")
            self._is_acs_connected = False
        except Exception as error:  # pylint: disable=W0703
            self.get_logger().debug("Error happen during device disconnection: %s" % str(error))
        finally:
            self._connection_lock.release()

    def reboot(self, mode="MOS", wait_for_transition=True,
               transition_timeout=None, skip_failure=False,
               wait_settledown_duration=False):
        """
        Perform a software reboot on the board.
        By default will bring you to MOS and connect acs once MOS is seen.
        this reboot require that you are in a state where adb command can be run.

        :type mode: str or list
        :param mode: mode to reboot in, support MOS.
               .. warning:: it is not always possible to reboot in a mode from another mode

        :type wait_for_transition: boolean
        :param wait_for_transition: if set to true,
                                    it will wait until the wanted mode is reached

        :type transition_timeout: int
        :param transition_timeout: timeout for reaching the wanted mode
                                    by default will be equal to boot timeout set on
                                    device catalog

        :type skip_failure: boolean
        :param skip_failure: skip the failure, avoiding raising execption, using
                                this option block the returned value when it is equal to False

        :type wait_settledown_duration: boolean
        :param wait_settledown_duration: if set to True, it will wait settleDownDuration seconds
                                          after reboot for Main OS only.

        :rtype: boolean
        :return: return True if reboot action succeed depending of the option used, False otherwise
                 - if wait_for_transition not used, it will return True if the reboot action has been seen
                   by the board
                 - if wait_for_transition used , it will return True if the reboot action has been seen
                   by the board and the wanted reboot mode reached.
        """
        self.get_logger().info("Rebooting the board...")
        transition_timeout = self._init_boot_timeout(transition_timeout)
        rebooted = False

        # if device is already OFF do not do soft shutdown procedure
        if self.is_available():
            # Disconnect logging threads before sending the reboot command
            # in case the reboot takes effect whereas logging is still active
            self.disconnect_board()
            time.sleep(2)
            # Reboot the board
            try:
                self.run_uecmd("Intel.Acs.TestFmk.DeviceSystem",
                               "Intel.Acs.TestFmk.DeviceSystem.DeviceSystemActivity", "StartDutReboot", "", 0)
                # Wait for device is off
                if wait_for_transition:
                    # Wait enough end of reboot
                    self.get_logger().info("Wait Reboot duration (%d s)..." % transition_timeout)
                    endtime = time.time() + transition_timeout
                    while endtime > time.time():
                        if not self.is_booted():
                            self.get_logger().info("Device shutdown ...")
                            rebooted = True
                            break
                    if not rebooted:
                        msg = "Dut didn't shutdown in time..."
                        self.get_logger().error(msg)
                        raise DeviceException(DeviceException.PROHIBITIVE_BEHAVIOR, msg)
                    else:
                        rebooted = False
                    while endtime > time.time():
                        if self.is_booted():
                            self.get_logger().info("DUT is rebooted...")
                            rebooted = True
                            break

                    self.get_logger().info("Check if DUT is rebooted...")
                    # Reconnect board
                    if self.is_booted():
                        self.get_logger().info("DUT is rebooted...")
                        self.connect_board()
                    else:
                        msg = "Mobile is in unknown state"
                        self.get_logger().error(msg)
                        raise DeviceException(DeviceException.PROHIBITIVE_BEHAVIOR, msg)

            except DeviceException as e:
                self.get_logger().error(" Reboot FAILED !! %s" % str(e))
                if not skip_failure:
                    raise
                else:
                    rebooted = False
        else:
            msg = "Mobile is unavailable"
            if not skip_failure:
                raise DeviceException(DeviceException.PROHIBITIVE_BEHAVIOR, msg)
            rebooted = False
        return rebooted

    def _init_boot_timeout(self, boot_timeout):
        """
        Initialize boot timeout to a valid int and default value
        if not defined by user
        :param boot_timeout: Total time to wait for booting
        :type boot_timeout: object
        :return: boot_timeout
        :rtype: int
        """
        if boot_timeout is None:
            self.get_logger().warning(
                "Boot timeout is not defined, using default value from Device_Catalog (%ds)" % self._boot_timeout)
            boot_timeout = self._boot_timeout
        elif not str(boot_timeout).isdigit() or boot_timeout < 0:
            # Check if we use an integer and check if not a negative value
            self.get_logger().warning(
                "Boot timeout (%s) must be a positive value (take default value from Device_Catalog %ds)" %
                (str(boot_timeout), self._boot_timeout))
            boot_timeout = self._boot_timeout
        else:
            # Value is good: Inform user on boot timeout value taken
            boot_timeout = int(boot_timeout)
            self.get_logger().info(
                "Boot timeout defined by the user: waiting %d seconds for the device ready signal..." %
                boot_timeout)

        return boot_timeout

    # -----------------------------STATE CHECK FUNCTIONS-----------------------

    def is_available(self):
        """
        Checks if the board can be used.

        :return: availability status
        :rtype: bool
        """
        return self._is_acs_connected

    def is_booted(self):
        """
        Returns a C{bool} indicating whether the phone is booted or not.

        :rtype: bool
        :return: the I{booted} status of the phone.
        """
        return self.get_state() == "alive"

    def get_state(self, check_shell=True):
        """
        get the device state.

        :type  check_shell: boolean
        :param check_shell: kept for compatibility only, not used

        :rtype: string
        :return: device state : alive, unknown
        """
        state = "unknown"

        self.get_logger().info("Getting device state...")
        result = self.__windows_service.is_alive()
        status = result.get_verdict()
        value = result['is_alive']
        if status == Global.SUCCESS and value is True:
            state = "alive"

        return state

    def get_boot_mode(self):
        """
        get the boot mode from windows
        :attention: not fully implemented
        :rtype: string
        :return: phone state : MOS or UNKNOWN
        """
        # use get state for now to know the boot mode
        result = "UNKNOWN"
        if self.get_state() == "alive":
            result = "MOS"

        return result

    def get_acs_agent_version(self):
        """
        Gets the ACS Agent version used for current UC/Campaign in the device.
        :rtype: str
        :return: ACS Agent version
        """
        # Copying onto target
        version = "unknown"

        if self.get_state() == "alive":
            try:
                result = self.__windows_service.get_version()
                if result.get_verdict() == CommandResult.VERDICT_PASS:
                    version = result['version']
            except CommandResultException as cre:
                self._logger.debug(str(cre))
        return version

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
        # Copying onto target
        if path.isfile(src_path):
            if timeout is None:
                item_size = path.getsize(src_path)
                timeout = int(item_size / 1024) + 20

            cmd = "push \"%s\" \"%s\"" % (src_path, dest_path)

            return self.run_cmd(cmd, timeout)
        else:
            msg = "Cannot push '%s', file does not exist." % src_path
            self.get_logger().error(msg)
            return Global.FAILURE, msg

    def run_uecmd(self, module_name, class_name, method_name, args, timeout):
        """
        Executes the specified uecmd returns the result
        If the timeout is reached, return an exception

        :type  module_name: str
        :param module_name: assembly name of the uecmd to execute

        :type  class_name: str
        :param class_name: Full class name of the uecmd to execute

        :type  method_name: str
        :param method_name: method name corresponding to the uecmd to execute

        :type  args: str
        :param args: args to set as input parameter of the uecmd

        :type  timeout: int
        :param timeout: execution timeout in seconds

        :return: Instance containing the result of the uecmd
        :rtype: CommandResult
        """
        if not timeout:
            timeout = self._uecmd_default_timeout
        debug_mode = False
        try:
            result = self.__windows_service.launch_activity(
                module_name, class_name, method_name, args, timeout, debug_mode)
            if result.get_verdict() != CommandResult.VERDICT_PASS or result.has_exception():
                raise DeviceException(DeviceException.OPERATION_FAILED,
                                      str(result.get_verdict_message()))
            return result
        # catch service communication errors
        except CommandResultException as cre:
            raise DeviceException(DeviceException.INTERNAL_EXEC_ERROR, str(cre))

    def run_cmd(self, cmd, timeout,
                force_execution=False,
                wait_for_response=True,
                silent_mode=False):
        """
        Execute a shell command and return the result message
        If the timeout is reached, return an exception

        :type  cmd: string
        :param cmd: cmd to be run
        :type  timeout: integer
        :type force_execution: Boolean
        :param timeout: Script execution timeout in seconds
        :param force_execution: Force execution of command
                    without check phone connected (dangerous)
        :param wait_for_response: Wait response from adb before
                                        stating on command

        :rtype: CommandResult
        :return: A command result containing as verdict message the command output.
        """
        if not timeout:
            timeout = self._uecmd_default_timeout
        try:
            # split command into a maximum of 2 parts (binary and arguments)
            cmd_split = str(cmd).split(' ', 1)
            if len(cmd_split) == 2:
                # the command contains arguments
                result = self.__windows_service.shell_cmd(cmd_split[0], cmd_split[1], timeout)
            else:
                # command does not contain any argument
                result = self.__windows_service.shell_cmd(cmd_split[0], "", timeout)
            if result.get_verdict() != CommandResult.VERDICT_PASS or result.has_exception():
                raise DeviceException(DeviceException.OPERATION_FAILED,
                                      str(result.get_verdict_message()))
            return result
        # catch service communication errors
        except CommandResultException as cre:
            raise DeviceException(DeviceException.INTERNAL_EXEC_ERROR, str(cre))

    def get_async_result(self, task_id):
        """
        Internal method that get ouptut of an async scheduled cmd.
        it is not designed to poll the response, if the task is not finish , the call
        will be blocking.
        Warning : for now, unfinished task will raise an exception

        :type  task_id: str
        :param task_id: task id to be read

        :rtype: CommandResult
        :return: A command result containing information about the executed uecmd.
        """
        try:
            status, result = self.__windows_service.get_async_result(task_id)
            if status in [WindowsService.ASYNC_TASK_CANCELED, WindowsService.ASYNC_TASK_FAULTED]:
                txt = "Task %s is %s : %s" % (task_id, status, str(result.get_verdict_message()))
                self._logger.error(txt)
                raise DeviceException(DeviceException.OPERATION_FAILED, str(txt))
            elif status == WindowsService.ASYNC_TASK_RUNNING:
                # todo: take this case in account instead of rising an exception!
                msg = "Task %s is still running" % task_id
                self._logger.debug(msg)
                raise DeviceException(DeviceException.PROHIBITIVE_BEHAVIOR, msg)
            elif status == WindowsService.ASYNC_TASK_UNKNOWN:
                msg = "Unknown task '%s'" % task_id
                self._logger.debug(msg)
                raise DeviceException(DeviceException.INVALID_PARAMETER, msg)
            if result.get_verdict() != CommandResult.VERDICT_PASS or result.has_exception():
                raise DeviceException(DeviceException.OPERATION_FAILED,
                                      str(result.get_verdict_message()))
            return result
        # catch service communication errors
        except CommandResultException as cre:
            raise DeviceException(DeviceException.INTERNAL_EXEC_ERROR, str(cre))

    def run_async_uecmd(self, module_name, class_name, method_name, args, start_delay, repeat_delay="",
                        total_duration=""):
        """
        Starts an asynchronous uecmd and return a task id to follow the result

        :type  module_name: str
        :param module_name: uecmd module

        :type  class_name: str
        :param class_name: class of uecmd

        :type  method_name: str
        :param method_name: uecmd function

        :type  args: str
        :param args: args to pass to the uecmd

        :type  start_delay: int
        :param start_delay: time in second before starting the uecmd

        :type  repeat_delay: int
        :param repeat_delay: [optional] time in second before repeating the uecmd.
        If used, total_duration must be set too.

        :type  total_duration: int
        :param total_duration: [optional] the maximum duration time in second allowed
        to repeat the uecmd. If used, repeat_delay must be set too.

        :rtype: str
        :return: scheduled task id
        """
        # first check that both
        if is_number(repeat_delay) != is_number(total_duration):
            text = "repeat_delay (%s) and total_duration (%s) must be used together as valid integer" % (
                repeat_delay, total_duration)
            self._logger.error(text)
            raise DeviceException(DeviceException.INVALID_PARAMETER, text)

        # if the var are not numbers and different from their default values, raise an error
        if repeat_delay in ["", None]:
            repeat_delay = 0
        elif is_number(repeat_delay):
            repeat_delay = int(repeat_delay)
        else:
            text = "repeat_delay (%s) must be a valid integer" % str(repeat_delay)
            self._logger.error(text)
            raise DeviceException(DeviceException.INVALID_PARAMETER, text)
        if total_duration in ["", None]:
            total_duration = 0
        elif is_number(total_duration):
            total_duration = int(total_duration)
        else:
            text = "total_duration (%s) must be a valid integer" % str(total_duration)
            self._logger.error(text)
            raise DeviceException(DeviceException.INVALID_PARAMETER, text)

        try:
            result = self.__windows_service.launch_async_uecmd(
                module_name, class_name, method_name, args, self._uecmd_default_timeout, False,
                int(start_delay), repeat_delay, total_duration)
            if result.get_verdict() != CommandResult.VERDICT_PASS or result.has_exception():
                raise DeviceException(DeviceException.OPERATION_FAILED,
                                      str(result.get_verdict_message()))
            return result['task_id']
        # catch service communication errors
        except CommandResultException as cre:
            raise DeviceException(DeviceException.INTERNAL_EXEC_ERROR, str(cre))

    def clean_async_uecmds(self):
        """
        Method that delete all daemon files
        """
        try:
            result = self.__windows_service.clean_async_uecmd()
            if result.get_verdict() != CommandResult.VERDICT_PASS or result.has_exception():
                raise DeviceException(DeviceException.OPERATION_FAILED,
                                      str(result.get_verdict_message()))
            elif result['status'] is False:
                raise DeviceException(DeviceException.OPERATION_FAILED,
                                      "Failed to clean asynchronous uecmds")
        # catch service communication errors
        except CommandResultException as cre:
            raise DeviceException(DeviceException.INTERNAL_EXEC_ERROR, str(cre))

    # -----------------------------LOGGER RELATED FUNCTION-------------------------------------
    def _init_logger(self):
        """
        Initializes the logger.
        """
        log_port = int(self._device_config.get("logPort", "8003"))
        self._device_logger = Win8Logger(self._device_ip, log_port)
        time = Util.get_timestamp()
        file_name = self.get_phone_model() + "_" + str(log_port) + time + "_win8.log"
        file_path = path.join(self.get_report_tree().get_report_path(),
                              file_name)
        self._device_logger.set_output_file(file_path)

    def _start_extra_threads(self):
        """
        Starts extra threads required by ACS
        like logger & watchdog
        """
        status_ok = self.__enable_kernel_log()
        if status_ok:
            self._device_logger.start()

    def _stop_extra_threads(self):
        """
        Stops extra threads required by ACS
        like logger & watchdog
        """
        # stop listening to the logger
        self._device_logger.stop()
        # disable kernel traces on the logger
        self.__disable_kernel_log()

    def __enable_kernel_log(self):
        """
        Enable kernel logs on Device, but does not start the logger service.
        :rtype: bool
        :return: True if operation succeed, False otherwise
        """
        try:
            self.__logger_service.enable_kernel_traces(self.__logger_filters)
            return True
        except CommandResultException as excp:
            self._logger.warning(str(excp))
        return False

    def __disable_kernel_log(self):
        """
        Disable kernel logs on Device, but doesn't terminate the logger service.
        :rtype: bool
        :return: True if operation succeed, False otherwise
        """
        try:
            self.__logger_service.disable_kernel_traces()
            return True
        except CommandResultException as excp:
            self._logger.warning(str(excp))
        return False

    def get_device_logger(self):
        """
        Return device logger

        :rtype: object
        :return: device logger instance.
        """
        return self._device_logger

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
        priorities = {"v": "VERBOSE", "i": "INFO", "d": "DEBUG", "w": "WARNING", "e": "ERROR"}
        level = "VERBOSE"
        if priority in priorities.keys():
            level = priorities[priority]
        self._logger.debug("inject log : " + str(message))
        try:
            self.__logger_service.inject_log(message, tag, level)
            return True
        except CommandResultException as excp:
            self._logger.warning(str(excp))
        return False

    # -----------------------------DOMAIN MANDATORY FUNCTIONS--------------------

    def get_em_parameters(self):
        """
        return energy management paramater declared on device_catalog for your current device.
        there are on the <EM> </EM>
        only paramter that are used on usecase will be return like threshold or supported feature.
        :rtype : dict
        :return: return a dictionnary where key are the parameter name and value the parameter value
                 an empty field will be set to a default value
        """
        em_phone_param = {}
        em_phone_param["BATTERY"] = {}
        em_phone_param["BATTERY"]["THERMAL_CHARGING_HIGH_LIMIT"] = self.get_config(
            "battThermalChargingHighLim", 45, int)
        em_phone_param["BATTERY"]["THERMAL_CHARGING_LOW_LIMIT"] = self.get_config("battThermalChargingLowLim", 0, int)
        em_phone_param["BATTERY"]["VBATT_MOS_SHUTDOWN"] = self.get_config("minVbattMosShutdown", 3.4, float)
        em_phone_param["BATTERY"]["VBATT_MOS_BOOT"] = self.get_config("minVbattMosBoot", 3.6, float)
        em_phone_param["BATTERY"]["VBATT_COS_BOOT"] = self.get_config("minVbattCosBoot", 3.2, float)
        em_phone_param["BATTERY"]["VBATT_FLASH"] = self.get_config("minVbattFlash", 3.6, float)
        em_phone_param["BATTERY"]["VBATT_FULL"] = self.get_config("vbattFull", 4.3, float)
        em_phone_param["BATTERY"]["VBATT_OVERVOLTAGE"] = self.get_config("minVbattOverVoltage", 4.5, float)
        em_phone_param["BATTERY"]["BATTID_TYPE"] = self.get_config("BattIdType", "ANALOG", str)
        em_phone_param["BATTERY"]["BATTID_VALUE"] = self.get_config("battIdValue", 115000, int)
        em_phone_param["BATTERY"]["BP_THERM_COEF"] = self.get_config("bpThermCoef", "170897;-0.049")
        em_phone_param["BATTERY"]["BATT_ID_VALUE"] = self.get_config("battIdValue", 120000, int)
        em_phone_param["BATTERY"]["BPTHERM_VALUE"] = self.get_config("bpThermValue", 35000, int)

        em_phone_param["GENERAL"] = {}
        em_phone_param["GENERAL"]["DATA_WHILE_CHARGING"] = self.get_config("dataWhileCharging", False, "str_to_bool")
        em_phone_param["GENERAL"]["ADB_AVAILABLE_MODE"] = self.get_config("adbAvailableMode", "MOS", str)
        em_phone_param["GENERAL"]["DISCHARGE_TYPE"] = self.get_config("dischargeType", "hard", str)

        em_phone_param["GENERAL"]["PRESS_HARD_SHUTDOWN"] = self.get_config("pressPowerBtnHardShutdown", 10, float)
        em_phone_param["GENERAL"]["PRESS_SOFT_SHUTDOWN"] = self.get_config("pressPowerBtnSoftShutdown", 6, float)
        em_phone_param["GENERAL"]["PRESS_BOOT"] = self.get_config("pressPowerBtnBoot", 3, float)
        em_phone_param["GENERAL"]["GENERATED_FILE_PATH"] = self.get_config("generatedFilePath", "/sdcard*/")

        # convert fastboot combo
        fastboot_combo = self.get_config("fastbootKeyCombo", [], str)
        if isinstance(fastboot_combo, str):
            fastboot_combo = fastboot_combo.replace(" ", "").split(";")
            # remove empty element in the list
            result = []
            for element in fastboot_combo:
                if element != "":
                    result.append(element)
            fastboot_combo = result
        em_phone_param["GENERAL"]["FASTBOOT_COMBO"] = fastboot_combo

        return em_phone_param

    def get_default_wall_charger(self):
        """
        Get the default wall charger name for this device.
        Usefull to connect the right charger for a device.

        :rtype: str
        :return: wall charger name ( DCP, AC_CHGR) , must match with IO CARD charger name
        """
        return self._default_wall_charger

    def get_wifi_adapter_name(self):
        """
        Get the Wi-Fi adpater name specified in the device catalog

        :rtype: str
        :return: device adapter name
        """
        return self._wifi_adapter_name

    # -----------------------------NOT YET IMPLEMENTED-------------------------------------

    def synchronyze_board_and_host_time(self):
        """
        Synchronize the device time and the acs host time
        """
        self._logger.warning(self.__class__.__name__ + ".synchronyze_board_and_host_time() not implemented")
        return Global.SUCCESS, "not yet implemented"

    def cleanup(self, campaign_error):
        """
        Clean up the environment of the target.

        :type campaign_error: boolean
        :param campaign_error: Notify if errors occured

        :rtype: tuple of int and str
        :return: verdict and final dut state
        """
        self._logger.warning(self.__class__.__name__ + ".cleanup() not implemented")
        return True, "NO_POWER_STATE"

    def init_acs_agent(self):
        """
        Check ACS agent version, and start it if needed

        :rtype: boolean
        :return: return a boolean for compatibilty
        """
        # todo:missing part to validate agent version and check that agent is started on remote
        return True

    def get_device_controller(self):
        """
        Returns the device controller object of the device model
        :rtype: DeviceController object
        :return: device controller
        """
        return self._eqts_controller

    def use_ethernet_connection(self):
        """
        not used for windows for now.

        :rtype: bool
        :return: always return false
        """
        return False

    def soft_shutdown_cmd(self):
        """
        Soft Shutdown command, if permissions are ok (rooted)
        """
        try:
            self.run_uecmd("Intel.Acs.TestFmk.DeviceSystem",
                        "Intel.Acs.TestFmk.DeviceSystem.DeviceSystemActivity", "StartDutSoftShutDown", "", 0)
        except DeviceException as e:
            self.get_logger().error(" soft_shutdown_cmd FAILED !! %s" % str(e))
            return False
        return True
