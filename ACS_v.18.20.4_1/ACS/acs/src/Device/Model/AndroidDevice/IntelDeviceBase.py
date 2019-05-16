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
:summary: This file implements the Intel device
:summary: DeviceBase -> AndroidDeviceBase -> IntelDeviceBase
:since: 05/08/2013
:author: cbonnard
"""

import time

from Device.Model.AndroidDevice.AndroidDeviceBase import AndroidDeviceBase
from Device.Model.AndroidDevice.EmbeddedLog import EmbeddedLog
from UtilitiesFWK.Utilities import Global
from UtilitiesFWK.Utilities import Verdict
import UtilitiesFWK.Utilities as Util


class IntelDeviceBase(AndroidDeviceBase):

    """
        Intel device implementation
    """

    # boot modes, as returned by adb shell getprop ro.bootmode
    BOOT_STATES = {"MOS": "main",
                   "ROS": "fota-recovery",
                   "COS": "charger"}

    def __init__(self, config, logger):
        """
        Constructor

        :type  phone_name: string
        :param phone_name: Name of the current phone(e.g. PHONE1)
        """
        self._provisioning_mode_set = False

        AndroidDeviceBase.__init__(self, config, logger)
        self._enableAdbRoot = self.get_config("enableAdbRoot", "True", "str_to_bool")
        self._enableIntelImage = self.get_config("enableIntelImage", "True", "str_to_bool")
        self._check_device_boot_mode = self.get_config("checkDeviceBootMode", "True", "str_to_bool")

        self._logger.info("Intel device initialized")

        if not self._enableIntelImage:
            self._logger.info("Reference image forced")
        if not self._enableAdbRoot:
            self._logger.info("Adb root disabled")

        # Embedded log feature
        self._embedded_log = EmbeddedLog.EmbeddedLog(self)

        # Start PTI and/or serial logs if required
        # Before establishing first connection to the device, enable PTI and/or Serial logging
        # to capture a maximum of traces. AP and BP logs will be handled further down the line
        if self._embedded_log:
            self._embedded_log.stop("PTI")
            self._embedded_log.stop("SERIAL")
            time.sleep(1)
            self._embedded_log.start("PTI")
            self._embedded_log.start("SERIAL")

        # Flashing configuration elements (This device class only support OS flashing procedure)
        self._soc_serial_number = self.get_config("socSerialNumber")

        # Flag indicating CrashModule has been successfully initialized
        # Will be used, to reinitialize Crash Modules if necessary
        self._init_crash_modules = False

    @property
    def write_logcat_enabled(self):
        """
        Do we need to write logcat from device to local host
        :return: Boolean
        """
        # Used to change the default value in case of Intel devices
        # AP logs are used instead of DUT logcat file
        return self.get_config("writeLogcat", "False", "str_to_bool")

    def _init_crash(self):
        """
        Init the crash log process for the device.

        Usually call after a first successful connection to the device

        :return: Boolean
        """

        crash_modules = self.get_device_modules("CrashModule")
        # if no crash module, crash modules initialization is ok
        status = True

        # init crash module
        if crash_modules:
            crash_logs_folder = self.get_report_tree().get_subfolder_path(subfolder_name="CRASH_LOGS",
                                                                          device_name=self._device_name)
            for crash_module in self.device_modules.CrashModule:
                status_crash_module = crash_module.init(self._serial_number, crash_logs_folder)
                if status_crash_module:
                    # log an information for Crash Info class instantiation
                    self.get_logger().info("Crash module loaded")
                    # in order to update the last known events
                    crash_module.list()

                    # Disable Crash Log Over the Air on device
                    status_crash_module = crash_module.disable_clota()
                else:
                    self.get_logger().error("Crash module not loaded properly")

                # Final status is the combination of all crash module initialization status
                status &= status_crash_module
        else:
            # log a warning for the user as Crash Info class is not well instantiated
            self.get_logger().warning("No available crash module")

        # Store last Crash modules initialization status
        self._init_crash_modules = status

        return status

    def _init_debug(self):
        """
        Init the debug process for the device.

        Usually call after a first successful connection to the device

        :return: Boolean
        """
        # init debug modules
        debug_modules = self.get_device_modules("DebugModule")
        if not debug_modules:
            self.get_logger().warning("No debug info will be collected")
        else:
            for debug_module in self.device_modules.DebugModule:
                self.retrieve_tc_debug_log = debug_module.init(self._serial_number)

    def _start_extra_threads(self):
        """
        Starts extra threads required by ACS
        like logger & watchdog and embedded logs
        :return: None
        """
        AndroidDeviceBase._start_extra_threads(self)

        # Start embedded log thread
        if self._embedded_log:
            self._embedded_log.start("APPLICATION")
        if self._device_log_file is not None:
            self._device_log_file.start()

    def _stop_extra_threads(self):
        """
        Stops extra threads required by ACS
        like logger & watchdog and embedded logs
        :return: None
        """
        AndroidDeviceBase._stop_extra_threads(self)

        # Stop embedded log thread
        if self._embedded_log:
            self._embedded_log.stop("APPLICATION")
        self.stop_device_log()

    def init_device_connection(self, skip_boot_required, first_power_cycle, power_cycle_retry_number):
        """
        Init the device connection.
        Call for first device connection attempt or to restart device before test case execution
        :param skip_boot_required:
        :param first_power_cycle:
        :param power_cycle_retry_number:
            """
        (connection_status,
         error_msg,
         power_cycle_total_count,
         boot_failure_count,
         connection_failure_count) = AndroidDeviceBase.init_device_connection(self,
                                                                              skip_boot_required,
                                                                              first_power_cycle,
                                                                              power_cycle_retry_number)

        return connection_status, error_msg, power_cycle_total_count, boot_failure_count, connection_failure_count

    def _get_device_boot_mode(self):
        """
            get the boot mode from adb

            :rtype: str
            :return: device state : MOS, ROS, COS or UNKNOWN
        """

        # checkDeviceBootMode is useful when device does not support ro.bootmode property
        if not self._check_device_boot_mode:
            current_state = AndroidDeviceBase._get_device_boot_mode(self)

        else:
            current_state = "UNKNOWN"
            output = self.run_cmd("adb shell getprop ro.bootmode", 10, True)

            if output[0] != Global.FAILURE:
                mode = (output[1].strip()).lower()
                for key in self.BOOT_STATES:
                    if mode in self.BOOT_STATES[key]:
                        current_state = key
                        break
            if current_state == "MOS":
                # In case of MOS we also check sys.boot_completed value to be sure that the device is fully booted
                current_state = AndroidDeviceBase._get_device_boot_mode(self)
        return current_state

    def get_boot_mode(self):
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
            if current_state.lower() == "unknown":
                # Fastboot command help find if device is in DnX or POS
                # For now we do not differentiate POS and DNX:
                # if DNX or POS -> POS
                # because DNX is tricky to detect and no req has been ask to do so
                current_state = self._check_pos_mode()
            time.sleep(3)

        self._logger.debug("device boot mode is %s" % current_state)

        return current_state

    def get_crash_events_data(self, tc_name):
        """
        Request the device to get failure logs

        :type tc_name: str
        :param tc_name: Test Case name

        :return: additional logs
        :rtype: String array
        """

        # In case of intel devices, if CrashToolUploaderModule is installed on the device
        # we can retrieve crash events data on the device
        results = list()

        # Init again all crash modules, if not already loaded before, else do nothing
        if not self._init_crash_modules:
            # log a warning for the user as Crash Info class is not well instantiated
            self.get_logger().warning("No crash modules available for the device, try to initialize them")
            self._init_crash()

        # If crash module loaded, retrieve crash events
        if self._init_crash_modules:
            for crash_module in self.device_modules.get('CrashModule', []):
                results.extend(crash_module.list())

        return sorted(results)

    def get_reporting_device_info(self):
        """
        Collect all device build info on the device for reporting

        :return: device and build infos
        :rtype: dict
        """

        # Add device / build information for ACS Live Reporting tool
        # No need to test status for AndroidDeviceBase.get_reporting_device_info(self) as always PASS
        _, report_device_info = AndroidDeviceBase.get_reporting_device_info(self)

        # For intel devices CrashToolUploaderModule is installed and retrieve device and build infos
        # These information are analysed and displayed by TCR reporting tool
        status_crashtool_device_info, crash_device_info = self._get_crash_device_info()
        if status_crashtool_device_info != Verdict.FAIL:
            report_device_info.update({"TCR": crash_device_info})

        # Return the status from crash device info as it is the device info reference for Intel Devices
        # and status for android device info is always PASS
        return status_crashtool_device_info, report_device_info

    def _get_crash_device_info(self):
        """
        Request the device info collected by crash tool

        :return: device and build infos
        :rtype: String
        """

        # In case of intel devices, if CrashToolUploaderModule is installed on the device
        # we can retrieve device and build information
        final_device_info_output = {}
        final_status_device_info = Verdict.PASS

        # Init again all crash modules, if not already loaded before, else do nothing
        if not self._init_crash_modules:
            # log a warning for the user as Crash Info class is not well instantiated
            self.get_logger().warning("No crash modules available for the device, try to initialize them")
            self._init_crash()

        # If crash module loaded, retrieve device info
        if self._init_crash_modules:
            for crash_module in self.device_modules.get('CrashModule', []):
                status_device_info, device_info_output = crash_module.get_device_info()
                if status_device_info != Verdict.PASS:
                    final_status_device_info = status_device_info
                final_device_info_output.update(device_info_output)

        return final_status_device_info, final_device_info_output

    def retrieve_debug_data(self, verdict=None, tc_debug_data_dir=None):
        """
        Fetch debug data (crashes, events, core dumps, anr, tombstones, ...), if any
        It will collect:
            - debug information according to debug module if available
            - crash information according to crash module if available
        :param verdict: Verdict of the test case
        :param tc_debug_data_dir: Directory where data will be stored
        """

        # In case of intel devices, if CrashToolUploaderModule is installed on the device
        # we can retrieve crash events data on the device

        # Init again all crash modules, if not already loaded before, else do nothing
        if not self._init_crash_modules:
            # log a warning for the user as Crash Info class is not well instantiated
            self.get_logger().warning("No crash modules available for the device, try to initialize them")
            self._init_crash()

        # If crash module loaded, retrieve crash info
        if self._init_crash_modules:
            # execute crash module fetch
            for crash_module in self.device_modules.get('CrashModule', []):
                crash_module.fetch(verdict=verdict)

        # If debug modules is installed
        # We can dump data for additional crash information
        debug_modules = self.get_device_modules("DebugModule")
        if not debug_modules:
            self.get_logger().warning("No debug dump will be done")
        else:
            # execute debug module dump
            for module in self.device_modules.get("DebugModule", []):
                if module.dump(verdict, tc_debug_data_dir):
                    self._logger.error("Error collecting debug info.")

    def clean_debug_data(self):
        """
        Remove intermediate debug log files and folders that should not be
        present in the final report folder
        """

        debug_modules = self.get_device_modules("DebugModule")
        if not debug_modules:
            self.get_logger().warning("No debug clean will be done")
        else:
            # execute debug module clean
            for module in self.device_modules.get("DebugModule", []):
                module.cleanup()

    def connect_board(self):
        """
        Connect to the device
        """
        connected = AndroidDeviceBase.connect_board(self)

        if self._embedded_log:
            # Route logcat to PTI if required
            self._embedded_log.enable_log_on_pti()
        return connected

    def setup(self):
        """
        Set up the environment of the target after connecting to it

        :rtype: (bool, str)
        :return: status and message
        """

        status, message = AndroidDeviceBase.setup(self)

        # Debug logging module initialization
        self._init_debug()
        # Crash and device info logging initialization
        status = self._init_crash()

        return status, message

    def cleanup(self, campaign_error):
        """
        Clean up the environment of the target.

        :type campaign_error: boolean
        :param campaign_error: Notify if errors occured

        :rtype: tuple of int and str
        :return: verdict and final dut state
        """
        for crash_module in self.device_modules.get('CrashModule', []):
            crash_module.enable_clota()
        status, dut_state = AndroidDeviceBase.cleanup(self, campaign_error)

        return status, dut_state

    def cleanup_logs(self, campaing_error):
        """
        clean any log of the device

        :type campaing_error: boolean
        :param campaing_error: Notify if errors occured
        """

        # Stop embedded logs
        if self._embedded_log:
            self._embedded_log.stop("APPLICATION")
            self._embedded_log.erase_log("APPLICATION")

            if campaing_error:
                self._embedded_log.retrieve_log("MODEM")
            self._embedded_log.stop("MODEM")

            self._embedded_log.stop("PTI")
            self._embedded_log.stop("SERIAL")

            # Delete embedded_log object here as it is the end of the
            # campaign. There is no need to start again AP &/or BP logging
            # in case a switch_on is required to leave the DUT powered on
            del self._embedded_log
            self._embedded_log = None

    def _pos_line_on(self):
        """
        Enter in the POS mode (used for flashing procedure) by trigging the provisionning line over the board

        :rtype: int (Global.SUCCESS, Global.FAILURE, Global.BLOCKED)
        :return: result of the switch state request

        """
        return_code = Global.FAILURE
        usb_host_action_result = False
        self.get_logger().info("Enable provisionning line: Trig POS mode")

        # Prepare power of device
        self._eqts_controller.plug_device_power()

        # Enable provisioning mode if requested and pos line wired
        if not self._provisioning_mode_set:
            self._provisioning_mode_set = self._eqts_controller.enable_provisioning_line()

        if not self._provisioning_mode_set and not self._use_adb_over_ethernet:
            # If not provisioning mode enabled, we will wake up the device
            # and "catch" provisioning os through usb insertion
            usb_host_action_result = self._eqts_controller.connect_usb_host_to_dut()

        if not usb_host_action_result:
            # We need to do it there because we may have only the power
            # control (no usb, no provisioning line), we have to try to start
            # the device
            if self._eqts_controller.poweron_device():
                return_code = Global.SUCCESS
        else:
            return_code = Global.SUCCESS

        if self._provisioning_mode_set and not self._use_adb_over_ethernet:
            # Plug usb cable as it has never been done if we use the provisioning mode
            if self._eqts_controller.connect_usb_host_to_dut():
                return_code = Global.SUCCESS

        return return_code

    def _pos_line_off(self):
        """
        Disable the provisionning line over the board. Stay in POS mode (Stay in flash mode)

        :rtype: int (Global.SUCCESS, Global.FAILURE, Global.BLOCKED)
        :return: result of the switch state request
        """
        self.get_logger().info("Disable provisionning line: stay in POS mode")
        return_code = Global.FAILURE
        if self._provisioning_mode_set:
            # Flash mode must be on now
            # Disable provisioning mode
            if self._eqts_controller.disable_provisioning_line():
                self._provisioning_mode_set = False
                return_code = Global.SUCCESS
        else:
            return_code = Global.SUCCESS

        return return_code

    def _flash_switch_off(self):
        """
        Switch off the device before doing a Flash procedure
        This can be done either via the power supply or via IO card or both

        :rtype: int (Global.SUCCESS, Global.FAILURE, Global.BLOCKED)
        :return: result of the switch state request
        """
        if not self.hard_shutdown(wait_for_board_off=True):
            return_code = Global.FAILURE
        else:
            return_code = Global.SUCCESS

        return return_code

    def _flash_device_switch_on(self):
        """
        Switch on the device before doing a Flash procedure
        This can be done either via the power supply or via IO card or both

        :rtype: int (Global.SUCCESS, Global.FAILURE, Global.BLOCKED)
        :return: result of the switch state request
        """

        return_code, error_message = self.switch_on()

        if return_code != Global.SUCCESS:
            self.get_logger().error("_flash_device_switch_on : %s" % error_message)

        return return_code

    def _finalize_os_boot(self, timer, boot_timeout, settledown_duration):
        """
            After connection to the device, define actions that need to be done to declare device as booted
            :type timer: int
            :param timer: time already spent to boot device (in seconds)
            :type  boot_timeout: int
            :param boot_timeout: boot timeout maximum value (in seconds)
            :type settledown_duration: int
            :param settledown_duration: time to wait after boot procedure to have device ready (in seconds)
            :rtype: int
            :return: result of the flash procedure (Global.SUCCESS, Global.FAILURE, Global.BLOCKED)
            :rtype: str
            :return: Output message status
        """

        begin_time = time.time()
        end_time = begin_time + float(boot_timeout)
        state = self.get_boot_mode()

        while state == "UNKNOWN" and time.time() < end_time:
            state = self.get_boot_mode()

        # Compute time device takes to pass in MOS state
        timer = timer + time.time() - begin_time

        if state == "MOS":
            self.get_logger().info("Boot complete in %3.1f seconds!", timer)
            # Wait for device to be fully booted
            full_boot_duration = time.time()
            full_boot = self._wait_for_full_boot(boot_timeout - timer)
            full_boot_duration = time.time() - full_boot_duration
            if full_boot:
                self.get_logger().info("Device is booted and ready to use!")
                self._settledown(settledown_duration)

                if self._screenshot_enable:
                    self.screenshot("BOOT")

                total_boot = timer
                total_boot += full_boot_duration
                # FIXME: Do not display boot stats on the same line, in B2B boot, this can be difficult to read
                return_message = "Boot time: %ds; " % full_boot_duration
                if settledown_duration:
                    return_message += "Settle down duration: %ds; " % settledown_duration
                    total_boot += settledown_duration
                return_message += "Total boot time %3.1fs" % total_boot
                self.get_logger().info(return_message)

                status = Global.SUCCESS

            else:
                # the device is not fully booted
                return_message = "Device has failed to perform a full boot after %d seconds !" % timer
                return_message += "Device is currently in %s mode !" % str(state)
                self.get_logger().warning(return_message)
                status = Global.FAILURE

        else:
            # the device has failed to boot, it is not in MOS state
            return_message = "Device has failed to boot !"
            return_message += "Device is currently in %s mode!" % str(state)
            self.get_logger().warning(return_message)
            status = Global.FAILURE
        return status, return_message

    def _check_ethernet_connection_is_available(self):
        """
        How to declare ethernet connection as available
        """
        return self.get_state() == "alive"

    def _wait_for_full_boot(self, timeout):
        """
        Loop until adb confirms boot complete
        :param timeout:
        :return:
        """
        self.get_logger().info("Waiting for device to be fully booted ...")

        begin_time = time.time()
        end_time = begin_time + float(timeout)
        full_boot = False
        while not full_boot and time.time() < end_time:
            if self.get_property_value("sys.boot_completed") == "1":
                self.get_logger().info("Device fully booted")
                full_boot = True
                # Wait 1 s before retrying
            time.sleep(1)

        return full_boot

    def _retrieve_fw_version(self, properties):
        """
        Retrieve the firmware version (from adb shell getprop)

        :type properties: dict
        :param properties: a dictionary containing all system prop (gotten through getprop)
        :rtype: str
        :return: the firmware version, or None if unable to retrieve it
        """
        key = "sys.ifwi.version"
        fw_version = None

        if key in properties:
            fw_version = properties[key]

        if fw_version in (None, ""):
            adb_cmd_str = "adb shell cat /sys/class/dmi/id/bios_version"

            status, status_msg = self.run_cmd(adb_cmd_str, self._uecmd_default_timeout, True)

            if status == Global.SUCCESS:
                if status_msg not in (None, "") and "no such file or directory" not in status_msg.lower():
                    fw_version = status_msg

            if fw_version in (None, ""):
                self.get_logger().warning("Fail to retrieve firmware version of the device")
                fw_version = None

        return fw_version

    def get_soc_number(self):
        """
        Return the soc number of the device
        ice
        """
        return self._soc_serial_number

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

        usb_host_action_result = False
        return_code = Global.FAILURE

        if self._is_phone_booted:
            return_code = Global.SUCCESS
            return_message = "Device already switched on."
            self.get_logger().info(return_message)
            if not self.is_available():
                self.connect_board()

        else:
            self._acs_agent.is_started = False

            # Handle entry parameter boot_timeout
            remaining_time = boot_timeout = self._init_boot_timeout(boot_timeout)

            # Ask for device state
            device_boot_mode = self.get_boot_mode()

            # We do not succeed to get the device state, try to switch on
            if device_boot_mode == "UNKNOWN":
                self._eqts_controller.plug_device_power()
                self._eqts_controller.poweron_device()

                # Plug USB
                if not self._use_adb_over_ethernet:
                    self.get_logger().info("Wait firmware boot time (%ds)..." % self._fw_boot_time)
                    time.sleep(self._fw_boot_time)

                    usb_host_action_result = self._eqts_controller.connect_usb_host_to_dut()

                if usb_host_action_result:
                    # Wait the time usb is ready
                    self.get_logger().info("Wait usb sleep duration (%ds)..." % self._usb_sleep_duration)
                    time.sleep(self._usb_sleep_duration)

                # After all switch on actions, get the new device state during boot timeout
                device_boot_mode = self.get_boot_mode()
                start_time = time.time()
                end_time = start_time + boot_timeout
                while device_boot_mode == "UNKNOWN" and time.time() < end_time:
                    device_boot_mode = self.get_boot_mode()
                    time.sleep(1)
                # update remaining boot timeout
                remaining_time -= time.time() - start_time

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
                                                    transition_timeout=int(remaining_time),
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
        else:
            self._settledown(settledown_duration)

        # When device booted, can retrieve device info
        return return_code, return_message

    def cleanup_final_state(self, final_dut_state, campaign_error):
        """
        Set final state of the device
        :param campaign_error:
        :param final_dut_state:
        :return: dut state
        """

        if final_dut_state == Util.DeviceState.COS:
            # Specific actions for Intel device
            self.cleanup_logs(campaign_error)
            if self.get_boot_mode() != "COS":
                # Leave the DUT in charging mode
                if not self.reboot("COS", wait_for_transition=True, skip_failure=True):
                    # Trying to power off the DUT if COS fails
                    self.get_logger().warning("Unable to configure the device in charging mode ! "
                                              "Trying to switch off the device.")
                    if self.switch_off()[0] != Global.SUCCESS:
                        final_dut_state = Util.DeviceState.UNKNOWN
                    else:
                        final_dut_state = Util.DeviceState.OFF
            else:
                self.get_logger().info("Board already in charging mode.")

        else:
            final_dut_state = AndroidDeviceBase.cleanup_final_state(self, final_dut_state, campaign_error)

        return final_dut_state
