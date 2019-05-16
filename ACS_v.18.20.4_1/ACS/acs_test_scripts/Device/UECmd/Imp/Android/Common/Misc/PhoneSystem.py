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
:summary: This file implements the Miscellaneous UEcmd for Android phone
:since: 11/04/2011
:author: dgonzalez
"""

from acs_test_scripts.Device.UECmd.Imp.Android.Common.BaseV2 import BaseV2
from acs_test_scripts.Device.UECmd.Interface.Misc.IPhoneSystem import IPhoneSystem
from acs_test_scripts.Device.UECmd.UECmdDecorator import need
from UtilitiesFWK.Utilities import Global
import re
import time
from UtilitiesFWK.Utilities import run_local_command, str_to_bool_ex, str_to_bool, internal_shell_exec
from Queue import Empty
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.AcsToolException import AcsToolException
import subprocess


class PhoneSystem(BaseV2, IPhoneSystem):

    """
    :summary: PhoneSystem UEcommands operations for Android platforms
    using an C{Intent} based communication to the I{DUT}.
    """
    SUPPORTED_ACCESSIBILITY = {
        "DELEGATE_SERVICE": "android.accessibilityservice.delegate/android.accessibilityservice.delegate.DelegatingAccessibilityService"}
    SUPPORTED_SLEEP_MODES = ["s3", "s0i1", "s0i3", "lpmp3"]
    SUPPORTED_WAKEUP_SOURCES = \
        {"USB": "USB",
         "HSI": "HSI",
         "GPIO": "GPIO",
         "IRQ 67": "PWR_BTN",
         "IRQ 14": "RTC"}
    MESSAGE_DICT_SOFTWARE = {"BOOT_MODE": "androidboot.mode=",
                             "COS_MODE": "androidboot.mode=charger",
                             "BOOT_REASON": "androidboot.wakesrc=",
                             "GRACEFUL_SHUTDOWN": "android.intent.action.ACTION_REQUEST_SHUTDOWN",
                             "BATT_INS_WAKESRC": "androidboot.wakesrc=03",
                             "SHT_CONFIRM": "shutdown, confirm="}

    def __init__(self, phone):
        """
        Constructor.
        """
        BaseV2.__init__(self, phone)
        IPhoneSystem.__init__(self, phone)
        self._logger = phone.get_logger()
        self.component = "com.intel.acs.agent/.PhoneSystem"
        self.icategory = "intel.intents.category.PHONESYSTEM"
        self.__vibra_card = "-c2"
        self.__vibra_uecmd_called = False

        self.__aplog_path = None
        # On GB and ICS wake screen with keyevent 19
        self._screenup_keyevent = 19

        self._pmu_kernel_file = None
        try:
            self._pmu_kernel_file = phone.get_config("PMUStates")
        except AttributeError:
            # Parameter does not exist for ref phone
            pass

        self.__audio_file = None
        self._setting_module = "acscmd.system.SystemSettingModule"
        self._filesystem_module = "acscmd.filesystem.FileSystemModule"
        self._camera_module = "acscmd.multimedia.CameraModule"
        self._display_module = "acscmd.display.DisplayModule"
        self._ringtone_module = "acscmd.system.RingtoneModule"
        self._audio_output_module = "acscmd.audio.AudioOutputModule"
        self._pupdr_module = "acscmd.pupdr.PupdrModule"
        # scheduled task list for asynchronous command
        self.__task_list = []

        self._message_dict = {}

        # Get system module to get hardware specific config
        self._system_module = self._device.get_device_module("SystemModule")
        self._system_module.init()

        # set global message dict
        # it will be based on :
        # - hardware dict information from system module
        # - software dict information from ue cmd class
        self._message_dict = self._system_module.system_properties.message_dict
        # update message dict according to software information
        self._message_dict.update(self.MESSAGE_DICT_SOFTWARE)

    def _convert_memory(self, memory_str):
        """
        Returns a memory value (given in kilobytes) from memory str.
        And return it as integer

        :type memory_str: str
        :param memory_str: a memory value as str (including G, K or M at tail)
        :rtype: int
        :return: memory value in kB
        """
        memory = None
        try:
            if "G" in memory_str:
                # Memory in Gb to convert to Kb
                memory = int(memory_str[:-1]) * 1024 * 1024
            elif "M" in memory_str:
                # Memory in Mb to convert to Kb
                memory = int(memory_str[:-1]) * 1024
            elif "K" in memory_str:
                # Memory in Kb to extract
                memory = int(memory_str[:-1])
            else:
                # Memory in Byte to convert to Kb
                memory = int(memory_str) / 1024

        except ValueError:
            self._logger.error("Error during catching memory string, \"" +
                               str(memory_str[:-1]) + "\" is not valid input !")

        return memory

    def get_property_value(self, key):
        """
        Returns property value by executing "adb shell getprop" command.

        :type key: String
        :param key: Key corresponding to the value you want to retrieve.

        :return: The associated value or None if the key hasn't been found.
        """
        value = self._device.get_property_value(key)

        return value

    def get_single_sim_gsm_property_value(self, android_property):
        """
        If a dual sim phone is used (for instance sofia 3GR) the gsm android property are a comma separated list: each value are associated to one sim
        for instance gsm.sim.operator.numeric= 00203,00101
        For single sim test using these platforms, only one sim must be plugged in phone,
        in this case the property looks like: gsm.sim.operator.numeric= ,00101 or gsm.sim.operator.numeric= 00101,
        This fonction retrieve the property for the active sim

        :type property: str
        :param property: the android property to retrieve

        :rtype: str
        :return: the gsm property value for the active sim card
        """
        # Retrieve the property from phone
        property_value = self.get_property_value(android_property)
        # Check if it is a dual sim DUT by checking if value embed,
        if ',' in property_value:
            property_value = property_value.split(',')
            # The following line is the PYTHON equivalent of C code:
            # property_value = property_value[0]==''?property_value[0]:property_value[1]
            # or if property_value[0]=='':
            #        property_value = property_value[0]
            #    else:
            #        property_value = property_value[1]
            property_value = 1 and property_value[0] or property_value[1]
        return property_value

    def get_phone_memory_size(self):
        """
        Returns the phone memory size.

        :rtype: int
        :return: Total memory size
        """
        cmd = "adb shell df /data"
        output = str(self._exec(cmd))

        str_list = output.split()
        try:
            memory_str = str_list[6]
            memory = self._convert_memory(memory_str)
        except IndexError:
            self._logger.error("Unable to locate memory size on returned result")
            memory = None

        if memory is None:
            raise DeviceException(DeviceException.OPERATION_FAILED, "Unable to fetch memory size")
        return memory

    def get_phone_memory_used(self):
        """
        Returns the used phone memory.

        :rtype: int
        :return: Used memory size
        """
        cmd = "adb shell df /data"
        output = str(self._exec(cmd))

        str_list = output.split()
        try:
            memory_str = str_list[7]
            memory = self._convert_memory(memory_str)
        except IndexError:
            self._logger.error("Unable to locate memory used on returned result")
            memory = None

        if memory is None:
            raise DeviceException(DeviceException.OPERATION_FAILED, "Unable to fetch memory size")
        return memory

    def get_phone_memory_left(self):
        """
        Returns the free phone memory.

        :rtype: int
        :return: Left memory size
        """
        cmd = "adb shell df /data"
        output = str(self._exec(cmd))

        str_list = output.split()
        try:
            memory_str = str_list[8]
            memory = self._convert_memory(memory_str)
        except IndexError:
            self._logger.error("Unable to locate memory left on returned result")
            memory = None

        if memory is None:
            raise DeviceException(DeviceException.OPERATION_FAILED, "Unable to fetch memory size")
        return memory

    def get_phone_sdcardext_size(self):
        """
        Returns the phone sdcardext size.

        :rtype: int
        :return: Total sdcardext size
        """
        cmd = "adb shell df %s" % self._device.get_sdcard_path()
        output = str(self._exec(cmd))

        str_list = output.split()
        try:
            sdcardext_str = str_list[6]
            sdcardext = self._convert_memory(sdcardext_str)
        except IndexError:
            self._logger.error("Unable to locate sdcardext size on returned result")
            sdcardext = None

        if sdcardext is None:
            raise DeviceException(DeviceException.OPERATION_FAILED, "Unable to fetch sdcardext size")
        return sdcardext

    def get_phone_sdcardext_used(self):
        """
        Returns the used phone sdcardext.

        :rtype: int
        :return: Used sdcardext size
        """
        cmd = "adb shell df %s" % self._device.get_sdcard_path()
        output = str(self._exec(cmd))

        str_list = output.split()
        try:
            sdcardext_str = str_list[7]
            sdcardext = self._convert_memory(sdcardext_str)
        except IndexError:
            self._logger.error("Unable to locate sdcardext used on returned result")
            sdcardext = None

        if sdcardext is None:
            raise DeviceException(DeviceException.OPERATION_FAILED, "Unable to fetch sdcardext size")
        return sdcardext

    def get_phone_sdcardext_left(self):
        """
        Returns the free phone sdcardext.

        :rtype: int
        :return: Left sdcardext size
        """
        cmd = "adb shell df %s" % self._device.get_sdcard_path()
        output = str(self._exec(cmd))

        str_list = output.split()
        try:
            sdcardext_str = str_list[8]
            sdcardext = self._convert_memory(sdcardext_str)
        except IndexError:
            self._logger.error("Unable to locate sdcardext left on returned result")
            sdcardext = None

        if sdcardext is None:
            raise DeviceException(DeviceException.OPERATION_FAILED, "Unable to fetch sdcardext size")
        return sdcardext

    def set_fm_power(self, mode):
        """
        Sets the FM power to on/off.

        .. warning:: Cannot be implemented on Android using Binder (adb services)

        :type mode: str or int
        :param mode: can be ('on', '1', 1) to enable
                            ('off', '0', 0) to disable

        :return: None
        """
        output = "set_fm_power can't be implemented on Android"
        self._logger.warning(output)

    def set_phone_lock(self, mode):
        """
        When phone is in idle mode, force the phone to stay locked/unlocked.

        :type mode: int
        :param mode: phone lock state in idle mode. Can be:
            -{0}: unlocked
            -{1}: lock
        """

        lock_mode = str_to_bool_ex(mode)
        if lock_mode is None:
            self._logger.error("set_phone_lock : Parameter mode %s is not valid" % str(mode))
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "Parameter 'mode' is not valid !")

        # Set the phone lock
        module = "acscmd.display.DisplayModule"
        function = "lockScreen" if lock_mode else "unlockScreen"
        self._internal_exec_v2(module, function)

    def set_phone_screen_lock_on(self, lock_on):
        """
        set phone screen lock on
        :type lock_on: integer
        :param lock_on: phone screen light state in idle mode. Can be:
        -{0}: unlocked display always ON
        -{1}: lock display ON
        :rtype: None
        """
        if lock_on in ("on", "1", 1):
            lock_on = 1
            self._logger.info("Lock the phone screen ON.")

        elif lock_on in ("off", "0", 0):
            lock_on = 0
            self._logger.info("Unlock the phone screen ON (can now turn OFF).")

        else:
            self._logger.error("set_phone_screen_lock_on : Parameter lock_on %s is not valid" % str(lock_on))
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "Parameter lock_on is not valid !")

        # Set the phone lock
        module = "acscmd.display.DisplayModule"
        function = "setLockScreenLightON"
        cmd_args = "--ei lockOn %d" % lock_on
        self._internal_exec_v2(module, function, cmd_args)

    def display_on(self):
        """
        set phone screen on
        :param none
        :rtype: None
        """
        self.set_phone_screen_lock_on("1")

    def display_off(self):
        """
        Try to set phone screen off
        :param None
        :rtype: None
        """
        self.set_phone_screen_lock_on("0")

    @need('speaker or headset or bluetooth')
    def switch_audio_output(self, output):
        """
        Route the audio route to a specific output

        :type output: str
        :param output: Possible output speaker, headset, bluetooth
        """
        if output in ("headset", "speaker", "bluetooth"):
            self._logger.info("Setting audio output to %s" % output)
        else:
            msg = "switch_audio_output : Parameter output %s is not valid" % str(output)
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
        function = "switchAudioOutput"
        cmd_args = " --es output %s " % output
        self._internal_exec_v2(self._audio_output_module, function, cmd_args)

    def get_sleep_mode(self):
        """
        get current sleep mode from pmu kernel file /sys/module/mid_pmu/parameters/s0ix for gingerbread
        get current sleep mode from pmu kernel file /sys/module/pmu/parameters/s0ix for ICS

        :rtype: String
        :return: current sleep mode
                 for possible mode value, ref to PhoneSystem.SUPPORTED_SLEEP_MODES
        """

        wakelocks_num = str(self._exec(str(
            "adb shell cat /proc/wakelocks | awk \'{print $1 \" \" $5}\' |grep -v \" 0\" |grep -v -c active"
        ))).rstrip()
        self._logger.debug("current wakelocks num is " + wakelocks_num)
        if wakelocks_num == '0':
            # if no wake locks, will enter s3 mode
            mode = 's3'
        else:
            mode = str(self._exec(str("adb shell cat %s") % self._pmu_kernel_file)).rstrip()
            # by default, will enter S0i3 mode
            if mode == '':
                mode = 's0i3'

        self._logger.debug("Current sleep mode: " + mode)

        if mode not in PhoneSystem.SUPPORTED_SLEEP_MODES:
            error_msg = "Unsupported sleep mode (%s)" % mode
            self._logger.info(error_msg)

        return mode

    def clear_pwr_lock(self):
        """
        Clear any 'acs' sleep lock for platform

        :return: None
        """
        self._logger.debug("Clear acs wake lock")
        self._exec("adb shell echo 'acs' > /sys/power/wake_unlock")

    def sleep_mode(self, enable):
        """
        Enable/Disable sleep mode

        :return: None
        """

        if enable in [1, '1', 'on']:
            self._logger.info("Sleep mode on")
            self._exec(
                "adb shell am start -a android.intent.action.MAIN "
                "-n com.intel.auto.display/.StandBy")
        else:
            self._logger.info("Sleep mode off")
            self._logger.warning('UECmd sleep_mode off support not yet supported')

    def sleep_mode_for(self, enable, alarm_time):
        """
        Enable/Disable sleep mode for alarm_time

        :return: None
        """

        if enable in [1, '1', 'on']:
            self._logger.info("Sleep mode on, alarm trigger in " + str(alarm_time) + " second(s)")
            self._exec(
                "adb shell am startservice -a android.intent.action.MAIN "
                "-n com.intel.auto.powerservice/.PowerEnableService "
                "--ei TIME_TO_SLEEP " + str(alarm_time))
        else:
            self._logger.warning('UECmd sleep_mode_for off support not yet supported')

    def get_sleep_value(self, mode, item):
        """
        Returns item value for selected mode
        Depends on /sys/kernel/debug/mid_pmu_states structure

        :type mode: String
        :param mode: Sleep mode to clear,
                     for possible mode value, ref to PhoneSystem.SUPPORTED_SLEEP_MODES

        :type item: String
        :param item: the value type got for current sleep mode.
                     for possible item value, Depends on /sys/kernel/debug/mid_pmu_states structure

        :return: Float
        """

        sleep_matched_value = 0

        try:
            mid_pmu_states_before = self._exec(
                str("adb shell cat /sys/kernel/debug/mid_pmu_states"), force_execution=True)

            regex_string = ".*%s(?:(?:\s+)(\S+)){%d}" % (mode, item)
            regex = re.compile(regex_string)
            sleep_matched = regex.search(mid_pmu_states_before)

            if sleep_matched:
                sleep_matched_value = sleep_matched.groups()[0]
                sleep_matched_value = float(sleep_matched_value)

        except (KeyboardInterrupt, SystemExit):
            raise
        except:
            raise DeviceException(DeviceException.PHONE_OUTPUT_ERROR, "Can't retrieve " + mode + " states")

        return sleep_matched_value

    def get_sleep_time(self, mode):
        """
        Returns sleep time in seconds for selected sleep mode
        mode possible value

        :type mode: String
        :param mode: Sleep mode to clear,
                     for possible mode value, ref to PhoneSystem.SUPPORTED_SLEEP_MODES

        :return: Float
        """
        self._logger.info("Get " + mode + " sleep time")
        return float(self.get_sleep_value(mode, 1))

    def get_sleep_residency(self, mode):
        """
        Returns sleep residency in % for selected sleep mode
        mode possible value

        :type mode: String
        :param mode: Sleep mode to clear,
                     for possible mode value, ref to PhoneSystem.SUPPORTED_SLEEP_MODES

        :return: Float
        """
        self._logger.info("Get " + mode + " residency")
        return float(self.get_sleep_value(mode, 2))

    def get_sleep_wakeup_count(self, mode):
        """
        Returns wakeup count for selected sleep mode

        :return: int
        """
        self._logger.info("Get " + mode + " wakeup count")
        return int(self.get_sleep_value(mode, 3))

    def reset_sleep_residency(self):
        """
        Reset residency counters

        """
        self._logger.info("Reset residency counters")
        self._exec(str("adb shell echo clear > /sys/kernel/debug/mid_pmu_states"), force_execution=True)

    def get_wakeup_source(self):
        """
        Returns last source which wakes up the DUT

        :rtype: String
        :return: the wakeup source (usb, power_button...)
        """
        wakeup_source = "UNKNOWN"

        cmd = "adb shell dmesg -c"
        kernel_output_lines = self._device.run_cmd(cmd, 5, True)[1].split("\n")
        regex_string = ".*wakeup from(?P<wakeup_src>(.*))"

        # Read kernel logs
        for kernel_output_line in kernel_output_lines:
            # Get the last wakeup source
            matches_str = re.compile(regex_string).search(kernel_output_line)
            if matches_str is not None:
                wakeup_source = matches_str.group("wakeup_src").strip().replace(".", "")
                if wakeup_source in PhoneSystem.SUPPORTED_WAKEUP_SOURCES.keys():
                    wakeup_source = PhoneSystem.SUPPORTED_WAKEUP_SOURCES[wakeup_source]

        if wakeup_source == "UNKNOWN":
            self._logger.warning("Wake up source not found !")

        return wakeup_source

    def check_ipc_sleep_mode(self, ipc_sleep_cmd, time_period, target_rate=50):
        """
        Check wether the IPC enters in sleep mode for a given time period

        :type ipc_sleep_cmd: String
        :param ipc_sleep_cmd: The IPC sleep shell command

        :type time_period: int
        :param time_period: The period of time to do the check (in seconds)

        :type target_rate: int
        :param target_rate: The percentage of the given time period where the
                                IPC residency should be checked.

        :raise DeviceException.TIMEOUT_REACHED: If the time is up and IPC
                                                    is not in sleep
        :raise AcsConfigException.INVALID_PARAMETER:
                                            In case of invalid input parameter
        :rtype: tuple
        :return: operation status & output log in case of success
        """
        self._logger.info("Checking IPC is in sleep for %d seconds (target: %d %%)..."
                          % (time_period, target_rate))

        if target_rate < 0 or target_rate > 100:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "The given target rate parameter is invalid (%s) !" % (str(target_rate)))

        initial_sleep_state = "D0"
        time_count = 1
        ipc_sleep_count = 0

        while time_count <= time_period:

            current_sleep_state = self.get_ipc_sleep_state(ipc_sleep_cmd)
            if current_sleep_state <> initial_sleep_state:
                ipc_sleep_count += 1

            # Update loop
            time_count += 1
            time.sleep(1)

        # Count the IPC sleep residency
        ipc_sleep = ipc_sleep_count / float(time_period) * 100

        if ipc_sleep < target_rate:
            err_msg = "The IPC sleep residency state %s, computed for a period of %s, is lower than the expected rate %s" \
                % (str(ipc_sleep), str(time_period), str(target_rate))
            raise DeviceException(DeviceException.TIMEOUT_REACHED, err_msg)
        else:
            output_message = "The IPC sleep residency state %s, computed for a period of %s, is higher than the expected rate %s" \
                % (str(ipc_sleep), str(time_period), str(target_rate))
            self._logger.info(output_message)

            return Global.SUCCESS, output_message

    def get_ipc_sleep_state(self, ipc_sleep_cmd):
        """
        Gets the IPC sleep state on the DUT.

        :raise DeviceException.INTERNAL_EXEC_ERROR: In case of error

        :rtype: String
        :return: The IPC state
        """
        # Check IPC command parameter
        if ipc_sleep_cmd is None:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "Command to check IPC state is not set !")

        output = self._exec(ipc_sleep_cmd, self._uecmd_default_timeout, False)

        if output is not None:
            if "D0i3" in output:
                current_ipc_state = "D0i3"
            elif "D3" in output:
                current_ipc_state = "D3"
            elif "D0" in output:
                current_ipc_state = "D0"
            else:
                raise AcsConfigException(AcsConfigException.OPERATION_FAILED, "Current IPC state does not match any expected state !")
            self._logger.debug("Current IPC state is: %s" % current_ipc_state)
            return current_ipc_state
        else:
            raise DeviceException(DeviceException.INTERNAL_EXEC_ERROR,
                                  "Unable to get current IPC state")

    def set_screen_timeout(self, timeout):
        """
        Sets the screen timeout.

        :type timeout: int
        :param timeout: the screen timeout in seconds. Use 0 to set maximum delay

        :return: None
        """

        if not isinstance(timeout, int) or timeout < 0:
            self._logger.error("set_screen_timeout : Bad timeout value :%s", timeout)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "Parameter timeout must be integer superior or equal to 0")

        if timeout == 0:
            self._logger.debug("set_screen_timeout maximum delay required. Set to 1800s")
            timeout = 1800

        self._logger.info("Set screen timeout to %s seconds" % timeout)

        function = "setScreenOffTimeout"
        cmd_args = "--ei timeout %s" % timeout
        self._internal_exec_v2(self._display_module, function, cmd_args)

    def get_screen_timeout(self):
        """
        Gets the screen timeout.

        :rtype: int
        :return: screen timeout in seconds
        """
        function = "getScreenOffTimeout"
        output = self._internal_exec_v2(self._display_module, function)
        if "screen_off_timeout" in output:
            timeout = int(output["screen_off_timeout"]) / 1000
            self._logger.info("screen timeout is : %s seconds" % timeout)
            return int(timeout)
        else:
            self._logger.error("get_screen_timeout : Cannot get uecmd output")
            raise AcsToolException(AcsToolException.PHONE_OUTPUT_ERROR, "Cannot get uecmd output")

    def wake_screen(self):
        """
        Wakes up the phone screen

        :return: None
        """
        self._logger.debug("Waking up phone screen")

        # Stimulate phone to wake it up
        if not self.get_screen_status():
            self._exec("adb shell input keyevent %d" % self._screenup_keyevent, force_execution=True)

    def sleep_screen(self):
        """
        Sleep down the phone screen

        :return: None
        """
        self._logger.info("Sleep down phone screen")

        # Stimulate phone to sleep it down
        if self.get_screen_status():
            self._exec("adb shell input keyevent %d" % self._screenup_keyevent)

    def check_file_exist(self, file_path):
        """
        Check if the file exists on the target file system

        :type file_path: str
        :param file_path: Full path of the file to be checked

        :return: None
        """
        self._logger.info("Checking that file %s exists" % file_path)
        function = "checkFileExist"
        cmd_args = "--es file_path %s" % file_path
        self._internal_exec_v2(self._filesystem_module, function, cmd_args)

    def close_welcome_screen(self):
        """
        close the welcome screen
        function must be implemented for LLP
        """
        pass

    def check_file_exist_from_shell(self, file_path):
        """
        Check if the file exists on the target file system from shell
        you need root access at least to do this.

        :type file_path: str
        :param file_path: Full path of the file to be checked

        :rtype: boolean
        :return: True if exist , False otherwise
        """
        cmd = "adb shell test -e %s && echo 'file exists' " \
            "|| echo 'file does not exist'" % file_path
        output = self._exec(cmd, force_execution=True)
        if output.lower().find("file does not exist") != -1:
            return False
        else:
            return True

    def check_directory_exist_from_shell(self, dir_path):
        """
        Check if the directory exists on the target file system from shell
        you need root access at least to do this.

        :type dir_path: str
        :param dir_path: Full path of the directory to be checked

        :rtype: boolean
        :return: True if exist , False otherwise
        """

        cmd = "adb shell ls %s " % dir_path
        output = self._exec(cmd)
        if output.lower().find("no such file or directory") != -1:
            return False
        else:
            return True

    def create_directory_from_shell(self, dir_path):
        """
        Create the directory running adb shell command

        :type dir_path: str
        :param dir_path: Full path of the directory which needs to be created
        """

        # Check if the directory already exists
        if (self.check_directory_exist_from_shell(dir_path) == True):
            self._logger.debug("Directory already exists, no further actions needed")
        else:
            self._logger.debug("Create directory: %s" % (dir_path))
            # Create the directory
            cmd = "adb shell mkdir -p %s" % (dir_path)
            output = self._exec(cmd)
            # Raise an error if the command failed
            if output.lower().find("read-only file system") != -1:
                self._logger.warning("write_file error: no write permission")
                raise DeviceException(DeviceException.OPERATION_FAILED, "write_file error: no write permission")
            self._logger.debug("The directory '%s' has been created" % (dir_path))

    def get_file_size(self, file_path):
        """
        Get the file size in bytes of the file given in parameter.
        Return size as long
        Return -1 if the file does not exist
        Return -2 if the filesize has not been successfully parsed

        :type file_path: str
        :param file_path: Full path of the file

        :rtype: long
        :return: size of the file in bytes. \
                 return -1 if file does not exist.
                 return -2 if the file size has not been successfully parsed
        """
        self._logger.info("Getting size of %s" % file_path)
        function = "getFileSize"
        cmd_args = "--es file_path %s" % file_path
        output = self._internal_exec_v2(self._filesystem_module, function, cmd_args)
        size = output.get("fileSize")
        if size is not None:
            return long(size)
        else:
            return -2

    def set_silent_vibrate(self, silent_enable, vibrate_mode):
        """
        Set silent mode enable/disable and vibrate mode for phone

        :type silent_enable: str
        :param silent_enable: sound silent mode enable: True|False
        :type vibrate_mode: String
        :param vibrate_mode: vibrate mode: always|never|silent|notsilent

        """
        self._logger.info("Setting silent mode: %s, vibrate mode: %s" %
                          (silent_enable, vibrate_mode))

        function = "setSilentAndVibrateMode"
        cmd_args = "--es silent_mode_enable %s --es vibrate_mode %s" % (silent_enable, vibrate_mode)
        self._internal_exec_v2(self._ringtone_module, function, cmd_args)

    @need('vibrator')
    def get_silent_vibrate(self):
        """
        Get silent mode enable/disable and vibrate mode from phone

        :rtype: str
        :return: obtained silent_enable and vibrate_mode result
        """
        self._logger.info("Getting silent_enable and vibrate_mode for phone")

        function = "getSilentAndVibrateMode"
        cmd_args = ""
        output = self._internal_exec_v2(self._ringtone_module, function, cmd_args)

        if ('silent_enable' not in output) or ('vibrate_mode' not in output):
            raise AcsToolException(AcsToolException.OPERATION_FAILED,
                                   "get silent mode and vibrate mode for phone failed")

        silent_enable = output["silent_enable"]
        vibrate_mode = output["vibrate_mode"]

        return silent_enable + "+" + vibrate_mode

    @need('speaker or headset')
    def get_audio_output(self):
        """
        Get audio output from phone

        :rtype: str
        :return: obtained audio_output: speaker|headset|bluetooth
        """
        self._logger.info("Getting Audio Output for phone")

        function = "getAudioOutput"
        cmd_args = ""
        output = self._internal_exec_v2(self._audio_output_module, function, cmd_args)

        if 'audio_output' not in output:
            raise AcsToolException(AcsToolException.PHONE_OUTPUT_ERROR, "get audio output for phone failed")

        audio_output = output["audio_output"]

        return audio_output

    def __configure_alsa_mixer(self):
        """
        configure the alsa_mixer option.
        Necessary to start vibrate.

        it will also find a working
        """
        self.__find_vibration_card()

        self._logger.debug("Trying to configure alsa_mixer")
        cmd = "adb shell alsa_amixer [CARDTOKEN] sset 'Vibra1 Boost Time' '32';" + \
            "alsa_amixer [CARDTOKEN] sset 'Vibra1 Duty Cycle' '50';" + \
            "alsa_amixer [CARDTOKEN] sset 'Vibra1 Brake' 'On';" + \
            "alsa_amixer [CARDTOKEN] sset 'Vibra1 Direction' 'Forward';" + \
            "alsa_amixer [CARDTOKEN] sset 'Vibra1 On Time' 'Infinite';" + \
            "alsa_amixer [CARDTOKEN] sset 'Vibra1 Off Time' '0.00';" + \
            "alsa_amixer [CARDTOKEN] sset 'Vibra1 Cycle Count' 'Infinite';" + \
            "alsa_amixer [CARDTOKEN] sset 'Vibra1 Enable Mux' 'SPI'"
        cmd = cmd.replace("[CARDTOKEN]", self.__vibra_card)
        self._exec(cmd)

    @need('vibrator', False)
    def get_vibration_state(self):
        """
        get the vibration state.

        :rtype: boolean
        :return: true if it is vibrating, false otherwise.
        """
        result = None
        if self._system_module.system_properties.vibrator_path:
            self._logger.info("Trying to get vibration state")

            cmd = "adb shell cat %s" % self._system_module.system_properties.vibrator_path
            output = self._exec(cmd).strip()
            if not self.is_shell_output_ok(output):
                self._logger.error("failed to get vibrator state : %s" % output)
                raise DeviceException(DeviceException.OPERATION_FAILED, "failed to get vibrator state , see log for detail")

            if output == "1":
                result = True
            elif output == "0":
                result = False
            else:
                self._logger.error("unknown vibrator state : %s" % output)
                raise DeviceException(DeviceException.PHONE_OUTPUT_ERROR, "unknown vibrator state, see log for detail")
        else:
            result = self._get_vibration_state_using_alsa()

        return result

    @need('vibrator', False)
    def set_vibration(self, mode):
        """
        Start to vibrate the phone
        :type mode: str or int
        :param mode: can be ('on', '1', 1) to enable
                            ('off', '0', 0) to disable
        :return: None
        """
        if self._system_module.system_properties.vibrator_path != "":
            self._logger.info("Trying to set vibration to %s " % mode)
            if mode in ("on", "1", 1):
                mode_code = "1"
            elif mode in ("off", "0", 0):
                mode_code = "0"
            else:
                self._logger.error("Parameter mode %s is not valid" % mode)
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "Parameter mode is not valid !")

            cmd = "adb shell echo %s > %s" % (mode_code, self._system_module.system_properties.vibrator_path)
            output = self._exec(cmd)

            if not self.is_shell_output_ok(output):
                self._logger.error("failed to turn vibrator %s : %s" % (mode,
                                                                        output))
                raise DeviceException(DeviceException.OPERATION_FAILED,
                                      "failed to turn vibrator %s , see log for detail" % mode)
        else:
            self._set_vibration_using_alsa(mode)

    def _get_vibration_state_using_alsa(self):
        """
        get the vibration state.

        :rtype: boolean
        :return: true if it is vibrating, false otherwise.
        """
        self._logger.info("Trying to get vibration state")
        if not self.__vibra_uecmd_called:
            self.__find_vibration_card()
            self.__vibra_uecmd_called = True

        cmd = "adb shell alsa_amixer %s sget 'Vibra1 Start' ''  ?" % self.__vibra_card
        output = self._exec(cmd)
        if output.find("Simple mixer control") != -1:
            if output[output.index("Item0"):-1].find("On") != -1:
                return True
            elif output[output.index("Item0"):-1].find("Off") != -1:
                return False
            else:
                self._logger.error("unknown vibrator state : %s" % output)
                raise DeviceException(DeviceException.INVALID_PARAMETER, "unknown vibrator state, see log for detail")

        self._logger.error("failed to get vibrator state : %s" % output)
        raise DeviceException(DeviceException.PHONE_OUTPUT_ERROR, "failed to get vibrator state, see log for detail")

    def _set_vibration_using_alsa(self, mode):
        """
        Start to vibrate the phone
        :type mode: str or int
        :param mode: can be ('on', '1', 1) to enable
                            ('off', '0', 0) to disable
        :return: None
        """
        self._logger.info("Trying to set vibration to %s " % mode)
        if mode in ("on", "1", 1):
            mode = "On"
        elif mode in ("off", "0", 0):
            mode = "Off"
        else:
            self._logger.error("Parameter mode %s is not valid" % mode)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "Parameter mode is not valid !")

        self.__configure_alsa_mixer()
        cmd = "adb shell alsa_amixer %s sset 'Vibra1 Start' '%s'" % (self.__vibra_card, mode)
        output = self._exec(cmd)
        self.__vibra_uecmd_called = True

        if output.find("Simple mixer control") == -1 or output[output.index("Item0"):-1].find(mode) == -1:
            self._logger.error("failed to turn vibrator %s : %s" % (mode, output))
            raise DeviceException(DeviceException.OPERATION_FAILED,
                                  "failed to turn vibrator %s , see log for detail" % mode)

    def __find_vibration_card(self):
        """
        Get the right vibra card number to control.

        :rtype: str
        :return: the name of working card for alsa_amixer vibra (c1,c2...)
        """
        self._logger.debug("Trying to find a working card")
        for card_number in range(10):
            cmd = "adb shell alsa_amixer -c%s" % card_number
            output = self._exec(cmd)
            if output.find("Vibra1") != -1:
                self.__vibra_card = ("-c%s" % card_number)
                return self.__vibra_card
        self._logger.error("No controlable card that use Vibra has been found!")
        raise DeviceException(DeviceException.INVALID_PARAMETER, "No controlable card that use Vibra1 has been found!")

    def log_dmesg(self, target, infinyloop, behavior=None,
                  behavior_value=None):
        """
        Log dmesg message into a file.

        :type target: str
        :param target: file where to log info , create it if it doesnt exit

        :type infinyloop: boolean
        :param infinyloop: if it is True then method will loop infinitely

        :type behavior: str
        :param behavior: change the behavior of the method :
                         - "scheduled" to schedule the method execution
                         - "stop" to stop the execution

        :type behavior_value: int
        :param behavior_value: the value link to the behavior:
                         - time to wait for "scheduled" option
                         - pid given by a former scheduled method execution

        :rtype: str
        :return: Pid of a scheduled operation or None
        """
        self._logger.warning("PhoneSystem.py - Function log_dmesg() not implemented.")

    def delete(self, filename, raise_exception_on_failure=True):
        """
        Deletes the file.

        :type filename: str
        :param filename: file to delete

        :type raise_exception_on_failure: Boolean
        :param raise_exception_on_failure: Raise exception on failure or not

        :rtype: boolean
        :return: True if the file has been well deleted, False otherwise
        """
        self._logger.info("delete file: %s " % filename)

        cmd = "adb shell rm -r %s " % filename
        output = self._exec(cmd, raise_error=raise_exception_on_failure)

        if output.lower().find("no such file or directory") != -1 or \
           output.lower().find("rm failed for %s, Read-only file system" % filename) != -1:
            if raise_exception_on_failure:
                msg = "file %s does not exist" % filename
                self._logger.error(msg)
                raise DeviceException(DeviceException.INVALID_PARAMETER, msg)
            return False
        return True

    def get_cpu_freq(self, cpu_id=0):
        """
        Get the cpu frequence

        :type  cpu_id: int
        :param cpu_id: cpu ID to check frequency

        :rtype: int
        :return: return the cpu frequence for each processor.
        """
        self._logger.info("get CPU frequency")
        cmd = "adb shell cat /proc/cpuinfo | grep cpu\ MHz"
        output = self._exec(cmd)
        output = output.splitlines()
        if len(output) >= (cpu_id + 1):
            return float(output[cpu_id].split(":")[1].strip())
        else:
            tmp_txt = "get_cpu_freq: cpu id %s not found" % cpu_id
            self._logger.error(tmp_txt)
            raise DeviceException(DeviceException.PHONE_OUTPUT_ERROR, tmp_txt)

    def get_max_cpu_freq(self, cpu_id=0):
        """
        Gets the Max frequency allowed

        :type  cpu_id: int
        :param cpu_id: cpu ID to check frequency
        :rtype: int
        :return: The CPU frequency in Hz
        """
        self._logger.info("get max CPU frequency")

        cmd = "adb shell cat " \
            "/sys/devices/system/cpu/cpu%d/cpufreq/scaling_max_freq" \
            % cpu_id
        output = self._exec(cmd, force_execution=True).strip()

        if output.isdigit():
            return int(output)
        else:
            self._logger.error("get_max_cpu_freq : unable to get CPU max frequency")
            raise DeviceException(DeviceException.PHONE_OUTPUT_ERROR,
                                  "get_max_cpu_freq : unable to get CPU max frequency")

    def get_backlight_level(self):
        """
        Gets the backlight level info

        :rtype: int
        :return: The backlight level
        """
        self._logger.info("get Backlight level")

        if self._system_module.system_properties.backlight_path:

            backlight_value_path = self._system_module.system_properties.backlight_path

            cmd = "adb shell cat %s" % backlight_value_path
            output = self._exec(cmd, force_execution=True).strip()
            self._logger.info("get_backlight_level: %s" % output)

            if output.isdigit():
                return int(output)
            else:
                self._logger.error("get_backlight_level : unable to get backlight level")
                raise DeviceException(DeviceException.PHONE_OUTPUT_ERROR,
                                      "get_backlight_level : unable to get backlight level")
        else:
                self._logger.error("get_backlight_level : backlight path is not define in device config file")
                raise AcsConfigException(AcsConfigException.READ_PARAMETER_ERROR, "get_backlight_level : backlight path is not define in device config file")

    def set_backlight_level(self, brightness):
        """
        Sets the display brightness

        :type  brightness: str containing int
        :param brightness: brightness to set
        """
        self._logger.info("set Backlight level")

        # Param control
        if not brightness.isdigit():
            self._logger.error("set_backlight_level not digit: %s" % brightness)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "set_backlight_level : brightness parameter should be a digit")
        if self._system_module.system_properties.backlight_path:

            backlight_value_path = self._system_module.system_properties.backlight_path

            cmd = "adb shell echo %s > %s" % (brightness, backlight_value_path)
            output = self._exec(cmd).strip()
            self._logger.info("set_backlight_level: %s" % output)

            if output.lower().find("read-only file system") != -1:
                self._logger.error("set_backlight_level error: no write permission")
                raise DeviceException(DeviceException.OPERATION_FAILED, "set_backlight_level error: no write permission")
            else:
                self._logger.debug("set_backlight_level returns: " + output)
        else:
                self._logger.error("set_backlight_level : backlight path is not define in device config file")
                raise AcsConfigException(AcsConfigException.READ_PARAMETER_ERROR, "set_backlight_level : backlight path is not define in device config file")

    def write_file(self, target_file, text, mode='overwrite', force_execution=False):
        """
        Writes given text into given file( can create the file if doesnt exist)

        :type target_file: str
        :param target_file: path of the file , eg: /ect/hello
        :type text: str
        :param text: text to write into the file
        :type mode: str
        :param mode: Optional open mode :
                     "overwrite" to erase contains and add text
                     "add" to add text to current contain
        :param force_execution: optional force execution of commend
        :type force_execution: bool

        :return: None
        """
        self._logger.info("write '%s' into file %s " %
                         (text, target_file))
        modified_text = ""
        text = str(text)
        if text.find("\n") != -1:
            text = text.splitlines()
            for line in text:
                modified_text += line + "\\n"
        else:
            modified_text = text

        if mode == 'add':
            cmd = "adb shell echo -e '%s' >> %s" % (modified_text, target_file)
        else:
            cmd = "adb shell echo -e '%s' > %s" % (modified_text, target_file)

        result = self._device.run_cmd(cmd, 5, force_execution)

        if result[1].lower().find("read-only file system") != -1:
            self._logger.warning("write_file error: no write permission")
            raise DeviceException(DeviceException.OPERATION_FAILED, "write_file error: no write permission")

    def read_file(self, target_file, behavior=None, behavior_value=None):
        """
        Returns the contain of given file.

        :type target_file: str
        :param target_file: path of the file , eg: /ect/hello

        :type behavior: str
        :param behavior: change the behavior of the method :
                         - "scheduled" to schedule the method execution
                         - "read" to read the output of a scheduled method

        :type behavior_value: int
        :param behavior_value: the value link to the behavior:
                         - time to wait for "scheduled" option
                         - pid given by a former scheduled method execution

        :rtype: str
        :return: Pid of a scheduled operation or file contents
        """
        self._logger.info("read  file %s" % target_file)
        cmd = "adb shell cat %s" % target_file
        output = self._exec(cmd)

        if output.lower().find("No such file or directory") != -1:
            tmp_txt = "file %s is not existing" % target_file
            self._logger.error(tmp_txt)
            raise DeviceException(DeviceException.INVALID_PARAMETER, tmp_txt)
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
        self._device.pull(remotepath, localpath, timeout)

    def create_subfolder(self, subfolder_name):
        """
        Create the given subfolder in the campaign report folder

        :type: String
        :param: Subfolder name

        :rtype: String
        :return: The path of the created subfolder
        """
        return self._device.get_report_tree().create_subfolder(subfolder_name)

    def copy(self, target, destination, wait_until_end=True):
        """
        Copy a file.

        :type target_file: str
        :param target_file: path of the file to copy , eg: /ect/hello

        :type target_file: str
        :param target_file: path of the tharget , eg: /ect/hellocopy

        :type wait_until_end: boolean
        :param wait_until_end: set to true to daemonize this command
        """
        self._logger.info("copy file %s into %s" %
                         (target, destination))

        cmd = "adb shell cp -r %s %s" % (target, destination)
        output = self._exec(cmd, wait_for_response=wait_until_end, force_execution=True)
        if output.lower().find("No such file or directory") != -1:
            tmp_txt = "file %s is not existing" % target
            self._logger.error(tmp_txt)
            raise DeviceException(DeviceException.INVALID_PARAMETER, tmp_txt)

    def set_display_brightness(self, brightness):
        """
        Sets the display brightness
        :type: brightness: int
        :param brightness: brightness to set, in percentage from 0 to 100

        :rtype: list
        :return: operation status & output log
        """
        self._logger.info("Setting screen brightness to : %s " % brightness)

        actual_brightness = None
        dpst_file = self._system_module.system_properties.dpst_file
        if dpst_file:
            cmd = "echo 'DSP DISABLE END' > %s && cat %s" % (dpst_file, dpst_file)

            output = self._exec("adb shell %s" % cmd)
            if output != "DPST Enabled:No":
                self._logger.error("Disabling DPST failed : %s" % output)

        output = self._internal_exec_v2(self._display_module,
                                        "setScreenBrightness",
                                        "--ei brightness %s " %
                                        brightness)

        if "actual_brightness" in output:
            actual_brightness = output["actual_brightness"]

        if dpst_file:
            cmd = "echo 'DSP ENABLE END' > %s && cat %s" % (dpst_file, dpst_file)
            output = self._exec("adb shell %s" % cmd)
            if output != "DPST Enabled:Yes":
                self._logger.error("Enabling DPST failed : %s" % output)

            self._exec("adb shell setprop coreu.dpst.aggressiveness 3")

        if not actual_brightness:
            raise DeviceException(DeviceException.PHONE_OUTPUT_ERROR,
                                  "Failed to set brightness : '%s'" % output)
        return actual_brightness

    def generate_ap_crash(self):
        """
        Generate an AP crash onto the phone
        """
        self._exec("adb shell dd if=/dev/zero of=/dev/kmem", 3)

    def set_brightness_mode(self, mode):
        """
        Sets the mode of the brightness

        :type mode: str
        :param mode: mode to set, 'manual' or 'automatic'
        """
        self._logger.info("Setting brightness mode to : %s " % mode)
        self._internal_exec_v2(self._display_module,
                               "setScreenBrightnessMode",
                               "--es mode %s" % mode)

    def get_date(self, seconds=False):
        """
        Return the phone date.

        :rtype: String
        :return: date
        """
        cmd = 'adb shell date +"%Y %m-%d %H:%M:%S"'
        output = self._exec(cmd)
        return output.strip()

    @need('torchlight')
    def set_torchlight(self, mode):
        """
        set flash in torchlight mode on/off

        :type mode: str or int
        :param mode: can be ('on', '1', 1) to enable
                            ('off', '0', 0) to disable
        :return: None
        """
        if mode in ("on", "1", 1):
            mode = 1
            self._logger.info("Trying to turn on flash as torchlight")

        elif mode in ("off", "0", 0):
            mode = 0
            self._logger.info("Trying to turn off flash")

        else:
            self._logger.error("set_torchlight: Parameter mode %s is not valid" % str(mode))
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "mode is not valid !")
        self._internal_exec_v2(self._camera_module, "setTorchLight", "--ei mode %s" % mode)

    def set_user_alarm_in_x_min(self, offset):
        """
        Set a new user alarm into the phone.

        :type offset: int
        :param offset: nb of minutes in which the alarm should ring.
                        The value should be between 1 and 59

        :return: None
        """
        # Input control
        if offset <= 0 or offset > 59:
            err_msg = "offset (%d) should be in [1,59]" \
                % offset
            self._logger.error(err_msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, err_msg)

        self.set_phone_lock(0)

        # Get the time of the board
        cmd = "adb shell date"
        output = self._exec(cmd).strip('\r\n ')
        date1 = time.strptime(output, "%a %b %d %H:%M:%S %Z %Y")

        # Start UI to set user alarm
        self._exec("adb shell am start -n com.android.deskclock/.SetAlarm")

        # Get a second time the time of the board
        output = self._exec(cmd).strip('\r\n ')
        date2 = time.strptime(output, "%a %b %d %H:%M:%S %Z %Y")

        # If the minute counter increases during the UI start
        # we need to restart is once.
        if date1.tm_min != date2.tm_min:
            output = self._exec(cmd).strip('\r\n ')
            date1 = time.strptime(output, "%a %b %d %H:%M:%S %Z %Y")

            # Start UI to set user alarm
            self._exec("adb shell am start -n com.android.deskclock/.SetAlarm")

            # Get a second time the time of the board
            output = self._exec(cmd).strip('\r\n ')
            date2 = time.strptime(output, "%a %b %d %H:%M:%S %Z %Y")

            if date1.tm_min != date2.tm_min:
                err_msg = "set_user_alarm_in_x_min(): " \
                    + "Unable to start UI user alarm set"
                self._logger.error(err_msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, err_msg)

        # Increase the minutes of the alarm
        # KEYCODE_DPAD_RIGHT
        self._exec("adb shell input keyevent 22")
        for _i in range(offset):
            # KEYCODE_DPAD_UP
            self._exec("adb shell input keyevent 19")
            # KEYCODE_DPAD_CENTER
            self._exec("adb shell input keyevent 23")

        # Check if we need to increase hour of the alarm
        if date1.tm_min + offset >= 60:
            # KEYCODE_DPAD_RIGHT
            self._exec("adb shell input keyevent 21")
            self._exec("adb shell input keyevent 21")
            self._exec("adb shell input keyevent 21")
            # KEYCODE_DPAD_UP
            self._exec("adb shell input keyevent 19")
            # KEYCODE_DPAD_CENTER
            self._exec("adb shell input keyevent 23")

        # Validate the inputs
        # KEYCODE_DPAD_DOWN
        # KEYCODE_DPAD_CENTER
        self._exec("adb shell input keyevent 20")
        self._exec("adb shell input keyevent 20")
        # KEYCODE_DPAD_CENTER
        self._exec("adb shell input keyevent 23")
        # KEYCODE_BACK
        self._exec("adb shell input keyevent 4")

    def get_screen_status(self, behavior=None, behavior_value=None):
        """
        Gets the screen status.

        :type behavior: str
        :param behavior:
                        - "scheduled" to schedule the operation
                        - "read" to read the output from a scheduled operation
                        - None to act like normaly

        if behavior is equal to "scheduled":
        schedule the method to be launch after x seconds

                :type behavior_value: int
                :param behavior_value: time in second to wait
                                               before executing the method
                :rtype: str
                :return: pid of scheduled operation

        if behavior is equal to "read":
        read the output of a previous scheduled operation

                :type behavior_value: int
                :param behavior_value: pid

                :rtype: bool
                :return: screen status was on or off

        if behavior is equal to None:

                :type behavior_value: None
                :param behavior_value: not used

                :rtype: bool
                :return: screen status on or off
        """
        if behavior == "scheduled":
            self._logger.info("Scheduled get screen status")
            function = "scheduledGetScreenStatus"
            task_id = self._internal_exec_v2(self._display_module, function, " --ei delay %s" % behavior_value)["output"]
            self.__task_list.append(task_id)
            return task_id

        elif behavior == "read":
            self._logger.info("Scheduled get screen status result")
            if behavior_value in self.__task_list:
                output = self.get_async_result(str(behavior_value))
                return str_to_bool(output["output"])

            else:
                raise DeviceException(DeviceException.OPERATION_FAILED, "cant execute scheduled getscreenstatus : unknown taskid %s" % behavior_value)

        else:
            function = "getScreenStatus"
            output = self._internal_exec_v2(self._display_module, function)

            if "screen_status" in output:
                screen_status = bool(int(output["screen_status"]))
                self._logger.info("screen status is : %s" % screen_status)
                return screen_status
            else:
                msg = "get_screen_status : Cannot get uecmd output"
                self._logger.error(msg)
                raise AcsToolException(AcsToolException.PHONE_OUTPUT_ERROR, msg)

    def get_screen_resolution(self):
        """
        Gets the screen resolution.

        :rtype: str
        :return: screen resolution
        """
        function = "getScreenResolution"
        output = self._internal_exec_v2(self._display_module, function)

        if "width" not in output or "height" not in output:
            raise DeviceException(DeviceException.PHONE_OUTPUT_ERROR,
                                  "output keys (width, height) not found: %s" % (str(output)))
        else:
            width = output.get("width")
            height = output.get("height")
            screen_resolution = "{0}x{1}".format(width, height)
            self._logger.info("screen resolution is : %s" % screen_resolution)
            return screen_resolution

    def stress_cpu(self, mode):
        """
        Activate/deactivate cpu stress.

        :type mode: str or int
        :param mode: can be ('on', '1', 1) to enable
                            ('off', '0', 0) to disable
        :return: None
        """
        if mode in ("on", "1", 1):
            cmd = "adb shell dd if=/dev/zero of=/dev/null bs=100000"
            self._logger.info("Trying to stress cpu")

        elif mode in ("off", "0", 0):
            cmd = "adb shell killall dd"
            self._logger.info("Trying to stop cpu stress")

        else:
            self._logger.error("stress_cpu: Parameter mode %s is not valid" % str(mode))
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "mode is not valid !")
        self._exec(cmd, wait_for_response=False)

    def stress_emmc(self, mode):
        """
        Activate/deactivate emmc stress.

        :type mode: str or int
        :param mode: can be ('on', '1', 1) to enable
                            ('off', '0', 0) to disable
        :return: None
        """
        if mode in ("on", "1", 1):
            # search the message in the aplog
            if self.__aplog_path is None:
                self.__aplog_path = self.__get_aplog_path()

            cmd = "adb shell cp %s/ /data/local/stress" % self.__aplog_path
            self._logger.info("Trying to stress emmc")

        elif mode in ("off", "0", 0):
            cmd = "adb shell killall cp"
            self._logger.info("Trying to stop emmc stress")

        else:
            self._logger.error("stress_cpu: Parameter mode %s is not valid" % str(mode))
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "mode is not valid !")
        self._exec(cmd, wait_for_response=False)

    def check_message_in_log(self, key, min_time=None, max_time=None, check_result=False):
        """
        check in aplog if a message linked to a key in message_dict
        is present in the aplog before the time

        :type key: str
        :param key: the message to find
        :type min_time: in float
        :param min_time: the min time limit in second to search the message
        :type max_time: in float
        :param max_time: the max time limit in second to search the message
        :type check_result: boolean
        :param check_result: if check result is True,
                                the function return the result of
                                the message between the message and a space

        :rtype: STRING
        :return: return a tuple (string_result, time_result)
                if check_result = False
                    string_result is "TRUE" or "FALSE"
                if check_result = True
                    string_result is the result after the message in aplog
        """
        def __check_message_in_output(aplog, message):
            """
            this function search the message in an aplog

            :type aplog: String
            :param aplog: the aplog name

            :type message: String
            :param message: message to search for

            :rtype: table of String
            :return: the result of the research
            """
            # search the message in the aplog
            if not self._device.is_available():
                self._exec("adb root", force_execution=True, timeout=10)
                time.sleep(10)
            # get the messages in the aplogs
            cmd = "adb shell grep -i \"%s\" %s/%s" % (message, self.__aplog_path, aplog)
            output = self._exec(cmd, force_execution=True, timeout=10)
            return output

        def __change_line_date_to_epoch_time(date):
            """
            internal function
            """
            if date:
                striped_result = date.strip()
                splitted_result = striped_result.split(".")[0].split(" ")
                formated_time = time.strftime("%Y ") + " " + splitted_result[0] + " " + splitted_result[1]
                time_result = time.mktime(time.strptime(formated_time, "%Y %m-%d %H:%M:%S"))
                return time_result
            else:
                raise DeviceException(DeviceException.INTERNAL_EXEC_ERROR, "Unable to retrieve date from device")

        # init all parameters and variables
        # check if the message is present in the init() of the PhoneSystem Uecmd
        if key not in self._message_dict:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "key %s is not present in message_dict" % key)

        message = self._message_dict[key]
        # check if the user has used the defaults times parameters
        if min_time is None:
            min_time = time.time() - 60
        if max_time is None:
            max_time = time.time()
        # init variables
        aplog_name = "aplog"
        aplog_nb = 0
        result = None
        # logs info
        str_t_min = time.strftime("%Y %m-%d %H:%M:%S",
                                  time.localtime(min_time))
        str_t_max = time.strftime("%Y %m-%d %H:%M:%S",
                                  time.localtime(max_time))
        self._logger.info("[CHECK APLOG] searching for tag '%s' from [%s] to [%s] in logs" %
                          (message, str_t_min, str_t_max))

        # if the board is not connected, it may be not rooted
        if self.__aplog_path is None:
            self.__aplog_path = self.__get_aplog_path()

        # check the message in all the aplog
        while result is None:
            output = __check_message_in_output(aplog_name, message)
            # if ADB is present, it means there is no message in the aplog
            if "no response from adb" in output.lower() or output == "":
                self._logger.info("[CHECK APLOG] no result for the message '%s'" % message +
                                  " in the %s" % aplog_name)
                # increase the aplog number to check
                aplog_nb += 1
                aplog_name = "aplog." + str(aplog_nb)
                continue

            # if the following message is present, it means there is no more aplog
            if "no such file or directory" in output.lower():
                self._logger.info("[CHECK APLOG] no result for the message '%s'" % message +
                                  " in all aplog")
                return "FALSE", None

            # reverse the list to look for more recent event
            output_split = output.split("\n")
            output_split.reverse()
            # check the time in each line of the ouput
            if output_split is not None:
                for line in output_split:
                    epoch_time = __change_line_date_to_epoch_time(line)
                    if min_time < epoch_time < max_time:
                        result = line
                        self._logger.info("[CHECK APLOG] found the message '%s' " % message +
                                          "at time : %s" % time.strftime("%Y %m-%d %H:%M:%S",
                                                                         time.localtime(epoch_time)))
                        break
            # increase the aplog number to check
            aplog_nb += 1
            aplog_name = "aplog." + str(aplog_nb)
        # check the result or send the true response
        if check_result:
            temp = result.split(message)
            temp2 = temp[1].split(" ")
            result = temp2[0].strip()
            self._logger.info("[CHECK APLOG] result : '%s'" % result)
            return result, epoch_time
        else:
            return "TRUE", epoch_time

    def select_pwoff_menu(self, confirm):
        """
        On power off menu, select power off and confirm the choice
        work only if the screen display the power off menu
        :type confirm: str
        :param confirm: "ok" or "cancel"
        """
        if confirm.lower() not in ["ok", "cancel"]:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "parameter confirm %s is invalid" % confirm)

        # open menu, go to power off
        poweroff_config = self._system_module.system_properties.poweroff_menu
        # try to be a ref position
        for key in poweroff_config.get("reference_position", []):
            self._exec("adb shell input keyevent {0}".format(key), force_execution=True)

        for key in poweroff_config.get(confirm, []):
            self._exec("adb shell input keyevent {0}".format(key), force_execution=True)

    def read_phone_device(self, mode):
        """
        read phone device model in MOS, or POS
        :type mode: str
        :param mode: "MOS" or "POS"
        """
        self._logger.info("READ device ID")
        # Build the adb devices command
        if mode == "MOS":
            largs = ['adb', 'devices']
        elif mode == "POS":
            largs = ['fastboot', 'devices']
        elif mode == "ROS":
            largs = ['adb', 'devices']
        else:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "parameter mode %s is invalid" % mode)

        # run command
        (process, q) = run_local_command(largs)
        time.sleep(1)
        # search ID in the result
        temp = ''
        while True:
            try:
                line = q.get(timeout=.1)
            except Empty:
                break
            else:
                if "device" in line and not "List" in line:
                    temp = line
                elif "fastboot" in line:
                    temp = line
        if temp == '':
            return "UNKNOWN"
        # format result
        result = temp.split("\t")
        self._logger.debug("result device:%s" % result[1])
        # terminate process if it wasn't
        # pylint: disable=E1101
        if process.poll is None:
            process.terminate()
        return result[1].strip().upper()

    def freeze_board(self, freeze_type):
        """
        simulate a board freeze.

        :type freeze_type: str
        :param freeze_type: "SOFT" or "HARD"
        """
        self._logger.info("trying to %s freeze board" % freeze_type)

        # simulate the freeze
        if freeze_type == "SOFT":
            cmd = "adb shell echo 1 > /d/emmc_ipanic"
        elif freeze_type == "HARD":
            cmd = "adb shell echo 1 > /d/watchdog/security_watchdog/trigger"
        else:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "the freeze type %s doesn't exist" % freeze_type)
        # umount /d in order to avoid device busy error
        self._exec("adb shell umount /d")
        time.sleep(5)
        # mount /d
        self._exec("adb shell mount -t debugfs none /d")
        time.sleep(5)
        # freeze the board
        self._exec(cmd, timeout=5, wait_for_response=False)

    def __get_aplog_path(self):
        """
        return the right aplogs path

        :rtype: str
        :return: aplogs path , raise error else
        """
        potential_folder = ["/data/logs", "/logs"]
        result = None
        for folder in potential_folder:
            txt = self._exec("adb shell ls %s" % folder, self._uecmd_default_timeout,
                             force_execution=True).lower()
            if txt.find("aplog") != -1 and self.is_shell_output_ok(txt):
                result = folder
                break

        if result is None:
            raise DeviceException(DeviceException.OPERATION_FAILED, "Failed to find aplogs folder")

        return result

    def change_language_country_informations(self, language="", country=""):
        """
        Change the country and language informations of the board.
        :type language: str
        :param language: the language (fr, en ...)
        :type country: str
        :param country: the country (FR, US ..)
        """
        if language:
            self._logger.info("Change language of the phone to : %s" % language)
            language = language.lower()
            cmd = """setprop persist.sys.language %s""" % language
            (error_code, _error_msg) = self._device.run_cmd("adb shell %s" % cmd, 10)
            if not error_code == Global.SUCCESS:
                raise DeviceException(DeviceException.OPERATION_FAILED,
                                      "Failed to change language property of the phone")
        if country:
            self._logger.info("Change language of the phone to : %s" % country)
            language = language.lower()
            cmd = """setprop persist.sys.country %s""" % country.upper()
            (error_code, _error_msg) = self._device.run_cmd("adb shell %s" % cmd, 10)
            if not error_code == Global.SUCCESS:
                raise DeviceException(DeviceException.OPERATION_FAILED,
                                      "Failed to change country property of the phone")
        # apply the new settings
        (error_code, _error_msg) = self._device.run_cmd("adb shell stop", 10)
        if not error_code == Global.SUCCESS:
            raise DeviceException(DeviceException.OPERATION_FAILED, "Failed to change country property of the phone")
        time.sleep(5)
        (error_code, _error_msg) = self._device.run_cmd("adb shell start", 10)
        if not error_code == Global.SUCCESS:
            raise DeviceException(DeviceException.OPERATION_FAILED, "Failed to change country property of the phone")
        time.sleep(10)

    def set_mock_location(self, value):
        """
        Set the security value for android to mock location settings

        :type value: boolean
        :param value: the value to set (True or False)
        """

        if value in [False, 0, "0"]:
            val = 0
        else:
            val = 1
        module = 'acscmd.system.SystemSettingModule'
        function = "setMockLocation"
        cmd_args = "--ei value %d" % val
        # Set the mock location settings
        self._internal_exec_v2(module, function, cmd_args, is_system=True)

    def set_accessibility_services(self, value):
        """
        Set the accessibility  to the specified service

        :type value: str
        :param value: the value to set (Str)
        """

        if value in [False, 0, "0"]:
            val = "0"
            self._logger.info("Deactivate accessibility")
            # desactivate the whole accesibility
        elif value in self.SUPPORTED_ACCESSIBILITY:
            val = self.SUPPORTED_ACCESSIBILITY[value]
            self._logger.info("Activate accessibility on %s" % val)
        else:
            val = str(value)
            self._logger.info("Activate accessibility on %s" % val)

        module = 'acscmd.system.SystemSettingModule'
        function = "setAccessibility"
        cmd_args = "--es value %s" % val
        # Set the accessibility services settings
        self._internal_exec_v2(module, function, cmd_args, is_system=True)


    def freeze_boot_board(self, end_freeze_wait=900, crash_board_cmd="none"):
        """
        Freeze board during the boot.
        During the boot of the board, this UEcmd writes in the watchdog in order to freeze the board.
        It writes between the OFF state and the MOS.

        :type end_freeze_wait: int
        :param end_freeze_wait : Freeze wait time : 15 minutes
        :type crash_board_cmd: str
        :param crash_board_cmd : Command to crash the board, 2 choices are possible (security watchdog or kernel (stress)) : none
        """
        # Initialize the parameters
        check_exception = False
        root_board_cmd = "adb root"
        check_board_cmd = "adb get-state"
        wait_device_cmd = "adb wait-for-device"

        if crash_board_cmd == "SECURITY":
            cmd_crash = "adb shell echo 1 > /d/watchdog/security_watchdog/trigger"
        elif crash_board_cmd == "KERNEL":
            cmd_crash = "adb shell echo 1 > /d/emmc_ipanic"
        else:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "the freeze cmd %s doesn't exist" % crash_board_cmd)

        # Wait the detection of the device
        self._device.run_cmd(wait_device_cmd, 60, True, True)

        # Root the device
        self._device.run_cmd(root_board_cmd, 10, True, True)

        # Crash the board
        cpt_crash = 0
        while cpt_crash <= 2:
            self._device.run_cmd(cmd_crash, 0, True, False)
            time.sleep(0.75)
            cpt_crash += 1

        # Wait the reboot of the board
        time.sleep(3)
        timelimit_freeze = time.time() + end_freeze_wait
        while timelimit_freeze > time.time():
            check_board_present = self._device.run_cmd(check_board_cmd, 1, True, False)
            if check_board_present[1] == "unknown":
                check_exception = True
                break

        # If the timeout is reached, raise an exception
        if not check_exception:
            raise DeviceException(DeviceException.OPERATION_FAILED, "Freeze timeout is reached !")

    def allow_install_non_market_apps(self, value, use_agent=False):
        """
        Allow to install non market applications or not

        :type value: bool
        :param value: the value to set (True or False)
        :type use_agent: bool
        :param use_agent: tell if command will use android api or call adb
        """
        if not value:
            val = 0
        else:
            val = 1
        if use_agent:
            raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED, "Implementation using API is not done")

        else:
            cmd = "adb shell sqlite3 /data/data/com.android.providers.settings/databases/settings.db " \
                  "\"insert into global (name,value) values ('install_non_market_apps', %d)\"" % val
            output = self._exec(cmd)
            if output:
                raise DeviceException(DeviceException.OPERATION_FAILED, "Unable to set install non market application"
                                                                        " setting to %s" % str(val))

    def set_verify_application(self, value):
        """
        Set the security value for android to verify application
        This won't do anything in Android version lower than JB-MR1
        """
        self._logger.debug("Application verification feature is not available on this platform : "
                           "ignoring set_verify_application command")
        pass

    def disable_antimalware_request(self):
        """
        Disable request for antimalware check
        """
        self._logger.debug("Anti-malware feature is not available on this platform : "
                           "ignoring disable_antimalware_request command")
        pass

    def set_stay_on_while_plugged_in(self, value, use_agent=True):
        """
        Set the value for android to stay awake when plugged to host.
        :type value: int or str
        :param value: the value to set (between 0 to 3) or in (true|false|usb|ac|wireless)
        :type use_agent: bool
        :param use_agent: tell if command will use android api or call adb
        """
        if use_agent:
            if value in [False, "false", 0, "0"]:
                val = 0
            elif value in [True, "true", 3, "3"]:
                val = 3
            elif value == "usb":
                val = 2
            elif value == "ac":
                val = 1
            elif value == "wireless":
                val = 4
            else:
                val = int(value)

            if not val in range(0, 4):
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                         "invalid value : %s, must be an integer between 0 to 3")

            module = 'acscmd.system.SystemSettingModule'
            function = "setStayOnWhilePluggedIn"
            cmd_args = "--ei value %d" % val

            # Set the stay on while plugged in settings
            self._internal_exec_v2(module, function, cmd_args, is_system=True)
        else:
            cmd = "adb shell svc power stayon %s" % str(value)
            output = self._exec(cmd)
            if output:
                raise DeviceException(DeviceException.OPERATION_FAILED, "Unable to set stay awake setting to %s"
                                                                        % str(value))

    def get_phone_time(self):
        """
        Get the time on the phone represented as the number of seconds elapsed
        since the 01-01-1970.
        :rtype: str
        :return: time of the phone.
        """
        return_code, return_msg = self._device.run_cmd('adb shell date +%s', 5)
        # Checking the output contains the expected key.
        if return_code != Global.SUCCESS:
            raise DeviceException(DeviceException.OPERATION_FAILED, "Failed to get phone time")
        return str(return_msg).strip()

    def get_file_permissions(self, file_name, permissions=None):
        """
        Get the user rights on the phone represented as the number of seconds elapsed
        :type file_name: str
        :param file_name: name of the file (the absolute pat of the file)
        :type permissions: str
        :param permissions: the permissions to check ( ex: ".rwxrwxrwx")

        :return: str.
        """

        cmd = "adb shell ls -l %s | sed 's/[[:space:]]\+/ /g' |cut -d' ' -f1" % file_name

        # Sending the command.
        output = self._exec(cmd).strip()
        if permissions is not None:
            if permissions == output:
                self._logger.info("the permissions of the file %s match %s" % (file_name, permissions))
                return True
            else:
                self._logger.info("the permissions of the file %s are %s, they are different from %s" %
                                  (file_name, output, permissions))
                return False

        else:
            self._logger.info("the permissions of the file %s are : %s" % (file_name, permissions))
            return output

    def rename(self, oldname, newname):
        """
        Rename a file.

        :type new_name: str
        :param new_name: the new name
        :type old_name: str
        :param old_name: the old name
        """

        self._logger.info("rename file %s into %s" % (oldname, newname))

        cmd = "adb shell mv %s %s" % (oldname, newname)
        output = self._exec(cmd)
        if output.lower().find("failed on '%s' - Read-only file system" % oldname) != -1:
            txt = " '%s' - Read-only file system" % oldname
            self._logger.error(txt)
            raise DeviceException(DeviceException.INVALID_PARAMETER, txt)

    def get_screen_dimensions(self):
        """
        Get the screen dimensions
        """
        function = "getScreenResolution"
        output = self._internal_exec_v2(self._display_module, function)
        # Checking the output contains the expected key.
        if "width" not in output or "height" not in output:
            raise DeviceException(DeviceException.PHONE_OUTPUT_ERROR,
                                  "output keys (width, height) not found: %s" % (str(output)))

        return sorted([int(output.get("width")), int(output.get("height"))], reverse=True)

    def get_auto_time(self):
        """
        Gets the auto time parameter status on device

        :rtype: bool
        :return: If parameter is set or not.
        """
        self._logger.info("Get Auto Time value from dut ...")

        method = "getAutoTime"
        output = self._internal_exec_v2(self._setting_module, method, is_system=True)

        self._logger.debug("Auto Time value from dut is: %s"
                           % output["auto_time"])

        if output["auto_time"] == "1":
            return True
        else:
            return False

    def set_auto_time(self, mode):
        """
        Sets the auto time parameter status on device

        :type mode: int
        :param mode: The mode in what the auto time to be set (0|1)

        :rtype: None
        """
        if mode in ("on", "1", 1):
            mode = 1
            self._logger.info("Trying to set Auto Time ON")

        elif mode in ("off", "0", 0):
            mode = 0
            self._logger.info("Trying to set Auto Time OFF")

        else:
            self._logger.error("set_auto_time: Parameter mode %s is not valid" % str(mode))
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "mode is not valid !")

        self._logger.info("set Auto Time value to {0} on dut ...".format(mode))

        method = "setAutoTime"
        cmd_args = "--ei mode {0}".format(mode)
        self._internal_exec_v2(self._setting_module, method, cmd_args, is_system=True)

    def get_current_time(self):
        """
        Get the current date and time, readably formatted

        :rtype: tuple of str
        :return: A tuple containing date and time (y, m, d, h, m, s)
        """
        self._logger.info("Getting current time on DUT")

        cmd = "adb shell date '+%Y.%m.%d.%H.%M.%S'"
        cmd = self._device.format_cmd(cmd)

        self._logger.info("Getting the current date")
        (_result, output) = internal_shell_exec(cmd, 10)

        date = output.split(".")
        if len(date) < 6:
            raise DeviceException(DeviceException.INTERNAL_EXEC_ERROR,
                                  "Error during date retrieval ({0})".format(output))
        datetime = tuple(date)[:6]
        self._logger.debug("Current DUT time is: {0}".format(str(datetime)))

        return datetime

    def configure_time(self, year="1970", month="01", day="01",
                       hour="00", minutes="00", seconds="00"):
        """
        Configure the DUT date and time with given values.

        :type year: str
        :param year: The year to be set on DUT

        :type month: str
        :param month: The month to be set on DUT

        :type day: str
        :param day: The day to be set on DUT

        :type hour: str
        :param hour: The hour to be set on DUT

        :type minutes: str
        :param minutes: The minutes to be set on DUT

        :type seconds: str
        :param seconds: The seconds to be set on DUT

        :rtype: None
        """
        formatted_date = str(year + month + day)
        formatted_time = str(hour + minutes + seconds)

        cmd = "adb shell date -s {0}.{1}".format(formatted_date, formatted_time)
        cmd = self._device.format_cmd(cmd)

        self._logger.info("Getting the current date")
        internal_shell_exec(cmd, 10)

    def get_cpu_info(self, requested=list()):
        """
        Get cpu information from the parameter requested list

        :type requested: list
        :param requested: The list of parameter to read in cpuinfo
        """

        if not requested:
            return

        pattern = "-e '" + "' -e '".join(requested) + "'"
        result = self._exec("adb shell grep %s /proc/cpuinfo|head -n %d" %
                            (pattern, len(requested)), 3)

        lines = result.splitlines()
        ret = [None] * len(requested)
        for line in lines:
            name = line.split(":")[0].strip()
            value = line.split(":")[1].strip()

            if requested.count(name):
                pos = requested.index(name)
                ret[pos] = value

        return ret

    def is_acs_agent_ready(self):
        """
        Returns a boolean indicating whether the ACS Agent is started or not.
        :rtype: boolean
        :return: True if AcsAgentService is started (ie: it can run commands), False otherwise
        """
        function = "isServiceStarted"
        target_type = "framework.ServiceStatus"

        self._logger.debug("Sending intents to check if ACS agent V2 is ready ... ")
        output = self._internal_exec_v2(target_type, function, timeout=self._device.get_uecmd_timeout())
        started = (output["result"] == '1')
        if not started:
            error_msg = "Error when trying to start ACS Agent, DUT is not connected or adb command issue"
            self._logger.error(error_msg)

        status = "STARTED" if started else "NOT STARTED"
        self._logger.debug("AcsAgentService status: " + status)

        return started

    def unlock_phone_with_swipe(self, lock_x, lock_y, timeout):
        """
        Unlock the screen with use swipe. Give lock icon coordinate and method generate adb command.
        :type lock_x: int
        :param lock_x: x coordinate in pixel
        :type lock_y: int
        :param lock_y: y coordinate in pixel
        :type timemout: int
        :param timeout: timeout
        :rtype: None
        :return: None
        """
        self._logger.info("Trying to unlock the phone with swipe")
        # Unlock the phone with add 300 pixels at lock icon coordinate.
        lock_swipe_x = lock_x + 300
        self._exec("adb shell input touchscreen swipe %s %s %s %s %s" % (lock_x, lock_y, lock_swipe_x, lock_y, timeout))

    def remove_sidetone(self):
        """
        Disables the sidetone by setting a very low gain value in the sidetone loop

        :rtype: None
        """
        cmd = "adb shell lpeexplorer -i -w " + \
              "cmd=33,task=1,type=1,len=20,flag=1," + \
              "plist=255:255:255:255:33:0:12:0:1:0:255:136:103:0:96:250:96:250:50:0 -k"

        self._logger.info("Disable sidetone")
        self._exec(cmd)

    def disable_wifi_scan_always_enable(self):
        """
        Disable wifi_scan_always_enabled option
        """

        # if the xml file exists, we update it and exit
        xmlglobal = "/data/system/users/0/settings_global.xml"
        if self.update_xml_setting(xmlglobal, "wifi_scan_always_enabled", 0, "setting"):
            return

        # Get the last inserted row id
        cmd = "adb shell sqlite3 /data/data/com.android.providers.settings/databases/settings.db" \
              " \"select * from SQLITE_SEQUENCE where name='global';\""
        output = self._exec(cmd)

        # New row id of wifi_scan_always_enabled variable
        row_id = int(output.split("|")[1]) + 1

        # Change value of wifi_scan_always_enabled variable
        cmd = "adb shell sqlite3 /data/data/com.android.providers.settings/databases/settings.db" \
              " \"insert into global values(" + str(row_id) + ", 'wifi_scan_always_enabled', 0);\""
        self._exec(cmd)

    def home_page_menu(self):
        """
        Go to home page
        """
        # Go to home page menu
        self._exec("adb shell am start -c android.intent.category.HOME -a android.intent.action.MAIN")

    def settings_menu(self):
        """
        Enter Settings menu or exit from Settings menu
        """
        self._exec("adb shell am start " + "-n com.android.settings/.Settings")

    def security_menu_settings(self):
        """
        Enter Security menu setting or exit from Security menu settings
        """
        self._exec("adb shell am start " + "-n com.android.settings/.SecuritySettings")

    def perform_graceful_shutdown(self, delay=0):
        """
        Perform a graceful shutdown

        :type delay:int
        :param delay: time in second to wait before performing a shutdown
        """
        cmd = "gracefulShutdown"
        args = "--ei delay %s" % str(delay)
        self._internal_exec_v2(self._pupdr_module, cmd, args, is_system=True)

    def check_process(self, procstr):
        """
        Confirms that the given process is alive
            RETURNS True when the process is alive, False otherwise.
        """

        # Check that the process is alive
        cmd = 'adb shell ps | grep {0}'.format(procstr)
        std_out = self._exec(cmd)

        if procstr in std_out:
            return True
        else:
            return False

    def turn_off_location(self):
        """
        Sets the location mode to off
        """

        # if the xml file exists, we update it and exit
        xmlsecure = "/data/system/users/0/settings_secure.xml"
        if self.update_xml_setting(xmlsecure, "location_providers_allowed", "", "setting"):
            return

        # else, update the database
        loc_db = "/data/data/com.android.providers.settings/databases/settings.db"
        cmd_del = "delete from secure where name='location_providers_allowed'"
        cmd_ins = "insert into secure (name, value) values ('location_providers_allowed', '')"
        self._exec("adb shell sqlite3 %s \"%s\"" % (loc_db, cmd_del))
        self._exec("adb shell sqlite3 %s \"%s\"" % (loc_db, cmd_ins))

    def recent_apps_menu(self):
        """
        Go to recent apps
        """
        # Go to recent apps menu
        self._exec("adb shell input keyevent KEYCODE_APP_SWITCH")
