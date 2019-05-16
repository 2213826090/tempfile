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
@summary: This file implements the SensorEnabled reference device for ACS Dev
            Others project will depend on: Clark, Lois, Diana ...
@since: 15 Sept 2014
@author: jreynaux
"""
from os import path
import threading
import time
import UtilitiesFWK.Utilities as Util
from Device.DeviceController import DeviceController
from Device.DeviceManager import DeviceManager
from Device.Model.DeviceBase import DeviceBase
from UtilitiesFWK.Utilities import Global, AcsConstants
from ErrorHandling.DeviceException import DeviceException
from Core.PathManager import Paths
from Device.Model.AndroidDevice.EmbeddedLog import EmbeddedLog
from Device.UECmd.Imp.BareMetal.Common.PyCmdsUtilities import PyCmdsUtilities


class SensorEnabledDeviceBase(DeviceBase):

    """
    Nordic Base implementation
    """

    def __init__(self, device_name, device_config):
        """
        Constructor

        @type  device_name: string
        @param device_name: Name of the current device(e.g. PHONE1)
        """
        DeviceBase.__init__(self, device_name, device_config)

        self._os_version = self.get_config("OSVersion", "Unknown").title()
        self._my_name = path.basename(__file__).split(".")[0]

        # mandatory parameter
        self._prss_pw_btn_time_switch_off = self.get_config("pressPowerBtnTimeSwitchOff", 10, int)
        self._prss_pwr_btn_time_switch_on = self.get_config("pressPowerBtnTimeSwitchOn", 3, int)

        # timeouts
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

        # _serial_number is the identifier used to communicate with the board (serialno if USB, IP if ethernet)
        # _device_id is the unique identifier of the device
        self._device_id = None

        # PUPDR parameter
        self._soft_shutdown_cmd = "system_shutdown 0"

        # misc var
        self._connection_lock = threading.Lock()
        self._is_device_connected = False

        self._eqts_controller = DeviceController.DeviceController(device_name, self._device_config,
                                                                  None, self.get_logger())

        # Embedded log feature
        self._embedded_log = EmbeddedLog.EmbeddedLog(self)

        self._py_cmds_api = None
        self._serial_port = self.get_config("serialPort", "/dev/ttyUSB0", str)
        # Default not use UART connection (no serial)
        self._retrieve_serial_trace = self.get_config("retrieveSerialTrace", "False", "str_to_bool")

    def get_em_parameters(self):
        """
        STUB for this device type

        :return: empty dict
        """
        return {}

    def initialize(self):
        """
        Initialize the environment of the target.

        @rtype: None
        """
        # do nothing for now
        pass

    def cleanup(self, campaing_error):
        """
        Clean up the environment of the target.

        :type campaign_error: bool
        :param campaign_error: Notify if errors occured

        :rtype: tuple of int and str
        :return: status and final dut state
        """
        # Disconnect the DUT
        status = False
        final_dut_state = str(self._global_config.campaignConfig.get("finalDutState"))
        self.get_logger().debug("Try to leave device in following final state : %s" % (final_dut_state,))
        try:
            dut_state = self.cleanup_final_state(final_dut_state, campaing_error)
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

    def cleanup_logs(self, campaing_error):
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
        return None

    # -----------------------------GETTER FUNCTIONS---------------------------

    def get_device_logger(self):
        return None

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

    def get_settledown_duration(self):
        """
        Return the settle down duration set in catalog.

        :rtype: int
        :return: settle duration in seconds.
        """
        return self._settledown_duration

    def get_name(self):
        """
        Returns the phone class name.

        @rtype: str
        @return: the name of the phone class.
        """
        return self._my_name

    def get_uecmd_timeout(self):
        """
        Returns the timeout to use for UE Commands.

        @rtype: int
        @return: the timeout for UE Commands.
        """
        return self._uecmd_default_timeout

    def __get_xw_revision(self, rev_type):
        """
        Extract a revision (hardware, firmware or software) using the system_get_revisions pyCmds command.

        :type rev_type: int
        :param rev_type: type of information to extract:
                - 0 for hardware revision
                - 1 for firmware revision
                - 2 for software revision

        :rtype: str
        :return: extracted revision
        """

        if rev_type < 0 or rev_type > 2:
            self._logger.debug("__system_get_revisions: bad info type ({0}). Shall be 0, 1 or 2.".format(rev_type))
            self._logger.debug("__system_get_revisions: set result as empty string".format(rev_type))
            return ""

        if rev_type == 0:
            info_str = "hardware revision"
        elif rev_type == 1:
            info_str = "firmware revision"
        else:
            info_str = "software revision"

        self._logger.debug("Getting {0} of DUT".format(info_str))
        rv = self.run_cmd(cmd="system_get_revisions", silent_mode=True)
        if len(rv) != 2:
            error_message = "Unable to get {0} (command output='{1}'). Set as Empty string".format(info_str, rv)
            self._logger.warning(error_message)
            return ""
        if len(rv[1]) < rev_type+1:
            error_message = "Not enough information in output to get {0} (command output='{1}'). Set as Empty string"\
                .format(info_str, rv)
            self._logger.warning(error_message)
            return ""

        res_rev = str(rv[1][rev_type])
        # Strip non printable characters
        res_rev = ''.join([c for c in res_rev if ord(c) > 31 or ord(c) == 9])

        self._logger.debug("{0} of DUT: {1}".format(info_str, res_rev))
        return res_rev

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

        # retrieve firmware version if not already done
        self.device_properties.fw_version = self._retrieve_fw_version()

        # retrieve board type from device parameters given to ACS command line option
        self.device_properties.board_type = self.get_config("boardType", "ndg_device", str)

        return self.get_device_info()

    def retrieve_device_id(self):
        """
        Retrieve the unique id of the device.

        :rtype: str
        :return: unique id of the device, or None if unknown
        """
        #TODO: Update accordingly
        return AcsConstants.NOT_AVAILABLE

    def _retrieve_fw_version(self):
        """
        Retrieve the firmware version (from dmesg)

        :rtype: str
        :return: the firmware version, or None if unable to retrieve it
        """
        self.get_logger().debug("Retrieving fw version ...")
        return self.__get_xw_revision(1)

    def _retrieve_sw_release(self):
        """
        Retrieve the software version

        :rtype: str
        :return: the sw release, or None if unable to retrieve it
        """
        self.get_logger().debug("Retrieving sw release ...")
        return self.__get_xw_revision(2)

    def _retrieve_kernel_version(self):
        """
        Retrieve the kernel version (from uname)

        :rtype: str
        :return: the kernel version, or None if unable to retrieve it
        """
        #TODO: Update accordingly
        return AcsConstants.NOT_AVAILABLE

    def _retrieve_model_number(self,):
        """
        Retrieve the model number

        :rtype: str
        :return: the model number, or None if unable to retrieve it
        """
        self.get_logger().debug("Retrieving model number ...")
        return self.__get_xw_revision(0)

    def __get_system_sdinfo(self):
        """
        Get system sdinfo returned by dut.
            BLE Link layer version number
            BLE company ID
            BLE SoftDevice version number
            The Temperature returned by the SoftDevice
        : rtype: tuple
        :return: Tuple of four element containing
        """
        # command: system_sdinfo
        # BLE Link layer version number : 7 (0x7)
        # BLE company ID : 89 (0x59)
        # BLE SoftDevice version number : 78 (0x4e)
        # The Temperature returned by the SoftDevice : 115 (0x73)
        output = (AcsConstants.NOT_AVAILABLE, AcsConstants.NOT_AVAILABLE,
                 AcsConstants.NOT_AVAILABLE, AcsConstants.NOT_AVAILABLE)

        cmd_str = "system_sdinfo"
        status, rv_output = self.run_cmd(cmd_str, self._uecmd_default_timeout, silent_mode=True)

        if status == Global.SUCCESS:
            if rv_output not in (None, "") and type(rv_output) is tuple:
                output = rv_output
        else:
            self.get_logger().error("Impossible to retrieve system sdinfo !")

        return output

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

            self._py_cmds_api = PyCmdsUtilities(self._logger, self._serial_port)

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
            if self._py_cmds_api and self.get_state() == "alive":
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

                # CHeCK UART CONNECTION
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
                        error_msg = "Device failed to synchronize time with host (%s)" % (error_msg)
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
        # TODO: Check if fix
        return Global.SUCCESS, "ok"

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
        current_state = "unknown"
        self.get_logger().info("Getting device state ...")

        status, output = self.run_cmd("test_link", force_execution=True)
        if status == Global.SUCCESS and output == 51966:
            current_state = "alive"

        self._logger.debug("Device on \"{0}\" state.".format(current_state))
        return current_state

    def get_boot_mode(self):
        """
        get the boot mode from ssh

        :rtype: string
        :return: device state : MOS, ROS, POS, DNX, COS or UNKNOWN
        """
        # TODO: Fix boot mode
        self._logger.debug("Getting boot mode ...")
        current_mode = "MOS"
        return current_mode

    def reboot(self, mode="MOS", wait_for_transition=True,
               transition_timeout=None, skip_failure=False,
               wait_settle_down_duration=False):
        """
        Perform a software reboot on the board.
        By default will bring you to MOS and connect acs once MOS is seen.
        this reboot require that you are in a state where adb command can be run.

        :type mode: str or list
        :param mode: mode to reboot in, support MOS, COS, POS, ROS. It can be a list of these modes
               (ie ("COS","MOS"))
               .. warning:: it is not always possible to reboot in a mode from another mode
                eg: not possible to switch from ROS to COS

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

        :type wait_settle_down_duration: boolean
        :param wait_settle_down_duration: if set to True, it will wait settleDownDuration seconds
                                          after reboot for Main OS only.

        :rtype: boolean
        :return: return True if reboot action succeed depending of the option used, False otherwise
                 - if wait_for_transition not used, it will return True if the reboot action has been seen
                   by the board
                 - if wait_for_transition used , it will return True if the reboot action has been seen
                   by the board and the wanted reboot mode reached.
        """
        # Mode in which to reboot(0 = main, 2 = wireless charging, 3 = recovery, 6 = OTA)
        reboot_dict = {"MOS": "0",
                       "POS": "6",
                       "ROS": "3",
                       "COS": "2"}
        s, o = self.run_cmd("system_reset {0}".format(reboot_dict[mode]))
        return True if s == Global.SUCCESS else False

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

        if settledown_duration is None:
            # settle down duration
            settledown_duration = self.get_settledown_duration()

        # Unload old pycmds
        # Check if not None in cas of not yet loaded (skipBootOnPowerCycle)
        if self._py_cmds_api is not None:
            self._logger.debug("Unload old pyCmds module api")
            self._py_cmds_api.unload_pycmds()

        self._logger.debug("Wait boot timeout ({0}s)".format(boot_timeout))
        time.sleep(boot_timeout)

        if not self._retrieve_serial_trace:
            self._logger.warning('No UART connection used, unable to check device state, seem to be ready')
            return Global.SUCCESS

        # Reload to ensure using the right version
        if self._py_cmds_api is None:
            self._logger.debug("pyCmds api was not yet initiated, do it now ...")
            self._py_cmds_api = PyCmdsUtilities(self._logger, self._serial_port)

        self._logger.debug("Wait settledown duration ({0}s)".format(settledown_duration))
        time.sleep(settledown_duration)

        return_code = Global.FAILURE
        return_message = "Board still not ready after %d" % boot_timeout

        if self._check_state("alive", boot_timeout):
            return_code = Global.SUCCESS
        else:
            self._logger.error(return_message)

        return return_code

    def _check_state(self, state, timeout):
        """
        Check the if the DUT enter in given B{boot} mode before given I{timeout}

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

        while (time.time() - start_time) < timeout:
            time.sleep(1)
            dut_state = self.get_state()
            if state in dut_state:
                state_found = True
                seconds = time.time() - start_time
                msg = "Device successfully switch in %s state after %.2fs" % (state, seconds)
                self.get_logger().info(msg)
                return state_found

        self.get_logger().error(msg)
        return state_found

    # -----------------------------PUSH/PULL/DUT EMBEDDED CODE EXECUTION--------------------

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
        status = Global.FAILURE
        output = "Cannot run the cmd, device not connected!"

        if not self._retrieve_serial_trace:
            output = 'No UART connection used, unable to run pyCmds command type'
            self._logger.error(output)
            return status, output

        if self._py_cmds_api is None:
            self._logger.debug("Initialization of pycmds api")
            self._py_cmds_api = PyCmdsUtilities(self._logger, self._serial_port)

        if self.is_available() or force_execution:
            # Try via SSH
            if not silent_mode:
                self._logger.debug('Try to run command via pyCmds primary:')
            status, output = self._py_cmds_api.do_py_cmd_command(cmd=cmd, silent_mode=silent_mode)

        elif not silent_mode:
            self.get_logger().warning("Cannot run '%s', device not connected!" % (str(cmd),))

        # convert bool to ACS standard status
        if status:
            status = Global.SUCCESS

        return status, output

    # ----------------------------- DEVICE PROPERTIES --------------------

    @property
    def retrieve_serial_trace(self):
        """
        Return whether use serial trace or not
        :rtype: bool
        :return: serial trace or not
        """
        return self._retrieve_serial_trace

    # ----------------------------- SPECIFIC DEVICE CODE --------------------

    def get_watchdog_log_time(self):
        return self.get_config("WatchDogLogCycle", 5, float)

    @staticmethod
    def format_cmd(cmd, silent_mode=False):
        """
        Insert serial number in adb command if needed (multi-device mode)
        :type  cmd: string
        :param cmd: cmd to be run
        :return: modified command
        :rtype: String
        """
        return cmd
