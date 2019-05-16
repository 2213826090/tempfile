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
:summary: This script implements the interface to unitary actions for
miscellaneous features.
:since: 04/08/2010
:author: vgombert
"""
from ErrorHandling.DeviceException import DeviceException

# pylint: disable=W0613


class IPhoneSystem():

    """
    Abstract class that defines the interface to be implemented
    by device system operations handling sub classes.

    All method that shall be redefined in sub-classes raise a
    I{DeviceException} error.
    """
    # pylint: disable=W0613

    def __init__(self, device):
        """
        Initializes this instance.

        Nothing to be done in abstract class.
        """
        pass

    def set_fm_power(self, mode):
        """
        Sets the FM power to on/off.

        :type mode: str or int
        :param mode: can be ('on', '1', 1) to enable
                            ('off', '0', 0) to disable

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_phone_memory_size(self):
        """
        Returns the phone memory size.

        :rtype: int
        :return: Total memory size
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_phone_memory_used(self):
        """
        Returns the used phone memory.

        :rtype: int
        :return: Used memory size
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_phone_memory_left(self):
        """
        Returns the free phone memory.

        :rtype: int
        :return: Left memory size
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_phone_sdcardext_size(self):
        """
        Returns the phone sdcardext size.

        :rtype: int
        :return: Total sdcardext size
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_phone_sdcardext_used(self):
        """
        Returns the used phone sdcardext.

        :rtype: int
        :return: Used sdcardext size
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_phone_sdcardext_left(self):
        """
        Returns the free phone sdcardext.

        :rtype: int
        :return: Left sdcardext size
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def start_audio_daemon(self):
        """
        Starts audio daemon.

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def stop_audio_daemon(self):
        """
        Stops audio daemon.

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def load_voice_module(self):
        """
        Load voice module.

        :rtype: int
        :return: loaded module number
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def unload_voice_module(self, module_number):
        """
        Unload voice module.

        :type module_number: int
        :param module_number: the module number eg : 11,12... or all to unload all loaded voice modules

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def switch_audio_output(self, sink_name):
        """
        Switchs the audio output.

        :param sink_name: type of the output , can be "headset" or "speaker"

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def mount_device(self, device, folder, behavior=None, behavior_value=None):
        """
        Mounts given device into folder (create it if it doesnt exist).

        :type device: str
        :param device: path of the device to mount , often on /dev
        :type folder: str
        :param folder: folder where to mount the device , eg : /dev/mnt

        :type behavior: str
        :param behavior: change the behavior of the method :
                         - "scheduled" to schedule the method execution
                         - "read" to read the output of a scheduled method

        :type behavior_value: int
        :param behavior_value: the value link to the behavior:
                         - time to wait for "scheduled" option
                         - pid given by a former scheduled method execution

        :rtype: str
        :return: Pid of a scheduled operation or None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def unmount_device(self, device, behavior=None, behavior_value=None):
        """
        Unmounts given device.

        :type device: str
        :param device: path of the device to unmount
                       or path of a folder where has been moutn a device

        :type behavior: str
        :param behavior: change the behavior of the method :
                         - "scheduled" to schedule the method execution
                         - "read" to read the output of a scheduled method

        :type behavior_value: int
        :param behavior_value: the value link to the behavior:
                         - time to wait for "scheduled" option
                         - pid given by a former scheduled method execution

        :rtype: str
        :return: Pid of a scheduled operation or None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

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
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

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
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def move(self, target_file, destination, behavior=None, behavior_value=None):
        """
        Moves target to given destination.

        :type target_file: str
        :param target_file: path of the file , eg: /ect/hello
        :type destination: str
        :param destination: where to move the file , eg: /tmp

        :type behavior: str
        :param behavior: change the behavior of the method :
                         - "scheduled" to schedule the method execution
                         - "read" to read the output of a scheduled method

        :type behavior_value: int
        :param behavior_value: the value link to the behavior:
                         - time to wait for "scheduled" option
                         - pid given by a former scheduled method execution

        :rtype: str
        :return: Pid of a scheduled operation or None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

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
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def log_dmesg(self, target, infinyloop, behavior=None, behavior_value=None):
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
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def clean_daemon_files(self):
        """
        Method that delete all daemon files

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_sleep_mode(self):
        """
        get current sleep mode from pmu kernel file /sys/module/mid_pmu/parameters/s0ix for gingerbread
        get current sleep mode from pmu kernel file /sys/module/pmu/parameters/s0ix for ICS

        :rtype: String
        :return: current sleep mode
                 for possible mode value, ref to PhoneSystem.SUPPORTED_SLEEP_MODES
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def clear_pwr_lock(self):
        """
        Clear any 'acs' sleep lock for platform

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def sleep_mode(self, enable):
        """
        Enable/Disable sleep mode

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def sleep_mode_for(self, enable, alarm_time):
        """
        Enable/Disable sleep mode for alarm_time

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

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
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

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
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_sleep_time(self, mode):
        """
        Returns sleep time in seconds for selected sleep mode
        mode possible value

        :type mode: String
        :param mode: Sleep mode to clear,
                     for possible mode value, ref to PhoneSystem.SUPPORTED_SLEEP_MODES

        :return: Float
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_sleep_residency(self, mode):
        """
        Returns sleep residency in % for selected sleep mode
        mode possible value

        :type mode: String
        :param mode: Sleep mode to clear,
                     for possible mode value, ref to PhoneSystem.SUPPORTED_SLEEP_MODES

        :return: Float
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_sleep_wakeup_count(self, mode):
        """
        Returns wakeup count for selected sleep mode

        :return: int
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def reset_sleep_residency(self):
        """
        Reset residency counters

        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_wakeup_source(self):
        """
        Returns last source which wakes up the DUT

        :rtype: String
        :return: the wakeup source (usb, power_button...)
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

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
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_ipc_sleep_state(self, ipc_sleep_cmd):
        """
        Gets the IPC sleep state on the DUT.

        :raise DeviceException.INTERNAL_EXEC_ERROR: In case of error

        :rtype: String
        :return: The IPC state
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_screen_timeout(self, timeout):
        """
        Sets the screen timeout.

        :type timeout: int
        :param timeout: the screen timeout in seconds. Use 0 to set maximum delay

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_screen_timeout(self):
        """
        Gets the screen timeout.

        :rtype: int
        :return: screen timeout in seconds
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def wake_screen(self):
        """
        Wakes up the device screen

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def sleep_screen(self):
        """
        Sleep the device screen

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def check_file_exist(self, file_path):
        """
        Check if the file exists on the target file system

        :type file_path: str
        :param file_path: Full path of the file to be checked

        :rtype: boolean
        :return: True if exist , False otherwise
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def check_file_exist_from_shell(self, file_path):
        """
        Check if the file exists on the target file system from shell
        you need root access at least to do this.

        :type file_path: str
        :param file_path: Full path of the file to be checked

        :rtype: boolean
        :return: True if exist , False otherwise
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

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
                 return -2 if the filesize has not been successfully parsed
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def remove_file(self, filepath):
        """
        Remove file from device

        :param filepath: Name and path of the file to delete
        :type filename: str
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def create_custom_size_user_data(self, filename, size_ko, random_data=False, folder=None):
        """
        Create a file directly into the device (no file to upload) with a specific size.

        :param filename: Name of the file to create
        :type filename: str

        :param size_ko: Size of the file to create, in kilo-octet (integer only)
        :type size_ko: int

        :param random_data: generate a file with random data in it.
        Default to False because it takes time to generate a file with random data.
        :type random_data: bool

        :param folder: destination folder on DUT where the file will be created.
        By default, it creates new files in ACS ftpdir_path of the device.
        :type folder: str
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_phone_screen_lock_on(self, mode):
        """
        set phone screen lock on
        :type lock_on: integer
        :param lock_on: phone screen light state in idle mode. Can be:
        -{0}: unlocked display always ON
        -{1}: lock display ON
        :rtype: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def display_on(self):
        """
        set device screen on
        :param none
        :rtype: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def display_off(self):
        """
        set device screen off
        :param None
        :rtype: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_silent_vibrate(self, silent_enable, vibrate_mode):
        """
        Set silent mode enable/disable and vibrate mode for device

        :type silent_enable: str
        :param silent_enable: sound silent mode enable: True|False
        :type vibrate_mode: String
        :param vibrate_mode: vibrate mode: always|never|silent|notsilent

        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_silent_vibrate(self):
        """
        Get silent mode enable/disable and vibrate mode from device

        :rtype: str
        :return: obtained silent_enable and vibrate_mode result
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_audio_output(self):
        """
        Get audio output from device

        :rtype: str
        :return: obtained audio_output: speaker|headset
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_vibration(self, mode):
        """
        Start to vibrate the device
        :type mode: str or int
        :param mode: can be ('on', '1', 1) to enable
                            ('off', '0', 0) to disable
        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_vibration_state(self):
        """
        get the vibration state.

        :rtype: boolean
        :return: true if it is vibrating, false otherwise.
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_max_cpu_freq(self, cpu_id=0):
        """
        Gets the Max frequency allowed

        :type  cpu_id: int
        :param cpu_id: cpu ID to check frequency
        :rtype: int
        :return: The CPU frequency in Hz
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_backlight_level(self):
        """
        Gets the backlight level info

        :rtype: int
        :return: The backlight level
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_backlight_level(self, brightness):
        """
        Sets the display brightness

        :type  brightness: str containing int
        :param brightness: brightness to set
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

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
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def create_subfolder(self, subfolder_name):
        """
        Create the given subfolder in the campaign report folder

        :type: String
        :param: Subfolder name

        :rtype: String
        :return: The path of the created subfolder
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

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
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_display_brightness(self, brightness):
        """
        Sets the display brightness
        :param brightness: brightness to set, in percentage

        :rtype: list
        :return: operation status & output log
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def generate_ap_crash(self):
        """
        Generate an AP crash onto the phone
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_brightness_mode(self, mode):
        """
        Sets the mode of the brightness
        :param mode: mode to set, 'manual' or 'automatic

        :rtype: list
        :return: operation status & output log
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_date(self, seconds):
        """
        Return the phone date.

        :type seconds: boolean
        :param seconds: True to return date in seconds.

        :rtype: String
        :return: date
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_user_alarm_in_x_min(self, offset):
        """
        Set a new user alarm into the phone.

        :type offset: int
        :param offset: nb of minutes in which the alarm should ring.
                        The value should be between 1 and 59
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_torchlight(self, mode):
        """
        set flash in torchlight mode on/off

        :type mode: str or int
        :param mode: can be ('on', '1', 1) to enable
                            ('off', '0', 0) to disable
        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

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
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_screen_resolution(self):
        """
        Gets the screen resolution.

        :rtype: str
        :return: screen resolution
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def stress_cpu(self, mode):
        """
        Activate/deactivate cpu stress.

        :type mode: str or int
        :param mode: can be ('on', '1', 1) to enable
                            ('off', '0', 0) to disable
        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_cpu_freq(self, cpu_id=0):
        """
        Get the cpu frequence

        :type  cpu_id: int
        :param cpu_id: cpu ID to check frequency

        :rtype: int or int list
        :return: return the cpu frequence for each processor.
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def stress_emmc(self, mode):
        """
        Activate/deactivate emmc stress.

        :type mode: str or int
        :param mode: can be ('on', '1', 1) to enable
                            ('off', '0', 0) to disable
        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

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
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def select_pwoff_menu(self, confirm):
        """
        Open pwoff_menu, select power off and confirm the choice
        work only if the screen display the power off menu
        :type confirm: str
        :param confirm: "ok" or "cancel"
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def freeze_board(self, freeze_type):
        """
        simulate a board freeze.

        :type freeze_type: str
        :param freeze_type: "SOFT" or "HARD"
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_boot_wake_source(self):
        """
        get the boot wake source from adb

        :rtype: tuple of str str
        :return: (wake source number, wake source reason)
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_stay_on_while_plugged_in(self, value):
        """
        Set the value for android to stay awake when plugged to host

        :type value: int
        :param value: the value to set (between 0 to 3)
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def change_language_country_informations(self, language="", country=""):
        """
        Change the country and language informations of the board.  Need a reboot of the board to be applied
        :type language: str
        :param language: the language (fr, en ...)
        :type country: str
        :param country: the country (FR, US ..)
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def activate_mock_location(self):
        """
        Activate the mock location. Need a reboot of the board to be applied
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def activate_accessibility(self):
        """
        Enable the accessibility. Need a reboot of the board to be applied
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def disable_lockscreen(self, wait_settle_down_duration=False, reboot=False):
        """
        Disable the lockscreen. Need a reboot of the board to be applied
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def freeze_boot_board(self, end_freeze_wait=900, crash_board_cmd="none"):
        """
        simulate a board freeze during the boot.

        :type wait_end_freeze_timetout: int
        :param wait_end_freeze_timetout : Boot wait time : 900 seconds
        :type crash_board_cmd: str
        :param crash_board_cmd : Command to crash the board, 2 choices are possible (security watchdog or kernel (stress)) : none
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_phone_time(self):
        """
        Get the time on the phone represented as the number of seconds elapsed
        since the 01-01-1970.

        :rtype: str
        :return: time of the phone.
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_file_permissions(self, file_name, permissions=None):
        """
        Get the user rights on the phone represented as the number of seconds elapsed
        :type file_name: str
        :param file_name: name of the file (the absolute pat of the file)
        :type permissions: str
        :param permissions: the permissions to check ( ex: ".rwxrwxrwx")

        :return: str.
        """

        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def rename(self, oldname, newname):
        """
        Rename a file.

        :type new_name: str
        :param new_name: the new name
        :type old_name: str
        :param old_name: the old name
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_screen_dimension(self):
        """
        Get the screen dimension
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_auto_time(self):
        """
        Gets the auto time parameter status on device

        :rtype: bool
        :return: If parameter is set or not.
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_auto_time(self, mode):
        """
        Sets the auto time parameter status on device

        :type mode: int
        :param mode: The mode in what the auto time to be set (0|1)

        :rtype: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_rtc_mode(self):
        """
        Gets the rtc mode parameter status on device

        :rtype: bool
        :return: If rtc is in local (true) or utc (false)
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_rtc_mode(self, mode):
        """
        Sets the rtc mode parameter status on device

        :type mode: int
        :param mode: The mode in what the rtc mode to be set (0 utc|1 local)

        :rtype: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_current_time(self):
        """
        Get the current date and time, readably formatted

        :rtype: tuple of str
        :return: A tuple containing date and time (y, m, d, h, m, s)
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

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
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def is_acs_agent_ready(self):
        """
        Check that ACS Agent is ready to use
        :rtype: boolean
        :return: True if ACS agent is ready, Fasle otherwise
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

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
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def remove_sidetone(self):
        """
        Disables the sidetone by setting a very low gain value in the sidetone loop

        :rtype: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def home_page_menu(self):
        """
        Go to home page
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def settings_menu(self):
        """
        Enter Settings menu or exit from Settings menu
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def security_menu_settings(self):
        """
        Enter Security menu setting or exit from Security menu settings
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def turn_off_location(self):
        """
        Sets the location mode to off
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def check_process(self, procstr):
        """
        Confirms that the given process is alive
            RETURNS True when the process is alive, False otherwise.
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)
