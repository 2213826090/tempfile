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
:summary: This script implements the interface of system uecmd.
:since: 08/24/2010
:author: wchen61
"""
from ErrorHandling.DeviceException import DeviceException

# pylint: disable=W0613


class ISystem():

    """
    Abstract class that defines the interface to be implemented
    by device system operations handling sub classes.

    All method that shall be redefined in sub-classes raise a
    I{DeviceException} error.
    """

    def __init__(self, device):
        """
        Initializes this instance.

        Nothing to be done in abstract class.
        """
        pass

    def system_tune_sound(self, soundtype):
        """
        check the tune sound of different types

        :type soundtype: str
        :param soundtype: the type of sound to be check

        :rtype: list
        :return: operation status & output log
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def adjust_specified_stream_volume(self, sound_type, volume_percent_level):
        """
        set the volume level on the specified stream

        :type sound_type: str
        :param sound_type: the sound stream

        :type volume_percent_level: int
        :param volume_percent_level: Volume settings percentage between 0% and 100%

        :rtype: str
        :return: operation status
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_specified_stream_volume(self, sound_type):
        """
        get the volume of audio stream

        :type  sound_type: str
        :param sound_type: the sound stream

        :rtype: int
        :return: volume value in percent
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def retrieve_system_information(self, informationtype, value):
        """
        check the system information

        :type informationtype: str
        :param informationtype: the type of information to be retrieved
        :type value: int or str
        :param value: the value expect to get

        :rtype: list
        :return: operation status & output log
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_date_and_time(self, date_and_time):
        """
        Set date and time on the device

        :type date_and_time: datetime.datetime
        :param date_and_time: Date and time in python datetime structure

        :rtype: list
        :return: operation status & output log.
                 Returns date and time in following format in case of success: "%d/%m %H:%M:%S"
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_date_and_time(self):
        """
        Get date and time on the device

        :rtype: list
        :return: operation status & date.
                 Returns date and time in following format in case of success: "%Y-%m-%d %H:%M:%S"
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_mode_duration(self, mode):
        """
        Get the duration passed in the given mode (up time, idle time, ...) since device startup

        :type mode: str
        :param mode: mode we want to retrieve the duration

        :rtype: int
        :return: time (in seconds) passed in the given mode.
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_timezone(self, timezone):
        """
        set device timezone

        :type timezone: str
        :param timezone: Timezone to set on the device

        :rtype: list
        :return: operation status & output log. Returns the timezone in case of success.
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def skip_wizard(self):
        """
        Remove the wizard

        :rtype: boolean
        :return: True if reboot is mandatory, False otherwise
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def configure_launcher(self):
        """
        Configure the launcher
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def clear_cache(self):
        """
        Clear the buffer and cache prior
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def pid_of(self, process_name):
        """
        Get the pid of a process by its name.
        :rtype: str
        :return: the pid of the process or empty str if process as not been
        found.
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_dumpsys_focus(self):
        """
        get the Windows dumpsys information on Focus (open window in UI)
        :rtype: bool
        :return: True
        :rtype: str
        :return: focus
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def check_Activity(self):
        """
        Check if activity_name is launch.
        :type activity_name: str
        :param activity_name: Name of research activity.
        :rtype: bool
        :return: True if window name is find, False if doesn't find.
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def open_Dialer(self):
        """
        Open dialer application.
        :rtype: None
        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def kill_Dialer_app(self):
        """
        Kills the dialer application.
        :rtype: None
        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def kill_Dialer_app_if_already_launch(self):
        """
        Kills the dialer application if already launch.
        :rtype: None
        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_filesystem_rw(self):
        """
        Set the file system in read/write mode.
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def install_device_executable(self, bin_path, destination, timeout=0):
        """
        Install a binary as device executable file

        :type bin_path: str
        :param bin_path: file to be installed

        :type  destination: str
        :param destination: destination on device

        :type  timeout: int
        :param timeout: operation timeout

        :rtype: list
        :return: Output status and output log
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def remove_device_files(self, device_directory, filename_regex):
        """
        Remove file on the device

        :type device_directory: str
        :param device_directory: directory on the device

        :type  filename_regex: str
        :param filename_regex: regex to identify file to remove

        :rtype: list
        :return: Output status and output log
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def remove_device_dir(self, device_directory):
        """
        Remove file on the device

        :type device_directory: str
        :param device_directory: directory on the device

        :rtype: list
        :return: Output status and output log
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_installed_launchers(self, open_chooser=False):
        """
        Retrieve a list of installed and enabled launchers on device

        :type open_chooser: bool
        :param open_chooser: Indicating if have to force opening
                            of a launcher windows chooser.

        :rtype: list
        :return: The list of installed launcher
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_package_state(self, package_name):
        """
        Get the given android package state

        :type package_name: str
        :param package_name: The android packege name to check.
                            As android ComponentName format (eg. com.android.launcher)

        :rtype: UeCmdTypes.PACKAGE_STATE
        :return: The package state corresponding to given package, default unknown
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_package_state(self, package_name, state):
        """
        :type package_name: str
        :param package_name: The android packege name to check.
                            As android ComponentName format (eg. com.android.launcher)

        :type state: UeCmdTypes.PACKAGE_STATE
        :param state: The package state to set

        :raise AcsConfigException: If a parameter name is not set or state invalid
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def run_shell_executable(self, exe, args, io_redirection=0, timeout=20):
        """
        Execute a shell script or binary.  This also sets permissions to
        rwxr-xr-x on the file to be sure it can execute.

        :type exe: str
        :param exe: Path and filename of script or binary to execute

        :type args: str
        :param args: Command line arguments to use with the executable

        :type io_redirection: int
        :param io_redirection: Determines what to do with stdout and stderr I/O.
            0 = do not redirect it; let it be returned in output_msg and check
                for common error messages.
            1 = redirect both to /dev/null (lost forever)
            2 = redirect stdout to stdout.log and stderr to stderr.log in the same
                directory as the executable

        :type timeout: int
        :param timeout: Number of seconds to wait for 'adb shell' to return.

        :raise AcsConfigException: If exe or args are not specified
        :raise DeviceException: If set_file_permission or run_cmd fails.
        :rtype: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def check_device_file_install(self, source_file_path, destination_file_path):
        """
        Check if the file installation has succeeded on a device

        :type file_path: str
        :param file_path: file to be installed

        :type  destination: str
        :param destination: destination on device

        :rtype: tuple (bool, str)
        :return: (Output status = True if file is installed on the device else
                 False, output error message)
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def install_scripts_from_lib(self, relative_source_path, timeout=0):
        """
        Copy all files from a specified subdirectory in acs_test_scripts/Lib/ShellScripts/<OS>,
        to a standard location on the DUT.  <OS> is "Android", "Windows" or "Linux", depending
        on what the device is running.

        The directory on the DUT is based on a hardcoded location, depending on the OS:
            Android: /data/<relative_source_path>
            Windows: TBD/<relative_source_path>
            Linux: TBD/<relative_source_path>

        :type relative_source_path: str
        :param relative_source_path: relative path to the files under
                                     acs_test_scripts/Lib/ShellScripts/<OS>

        :type  timeout: int
        :param timeout: operation timeout

        :raise AcsConfigException: If relative_source_path does not exist under
                                   acs_test_scripts/Lib/ShellScripts/<OS>
        :raise DeviceException: If push fails
        :rtype: str
        :return: absolute path where files were put on the DUT
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def disable_google_voice_hotword(self):
        """
        Disables the Google voice hotword feature.

        @return: True if the service is disabled, False otherwise.
        @rtype: Boolean
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def disable_voiceinteraction(self):
        """
        Disables the Google voiceinteraction service (related to hotword feature)

        @return: True if the service is disabled, False otherwise.
        @rtype: Boolean
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def disable_voicerecognition(self):
        """
        Disables the Google voicerecognition service (related to hotword feature)

        @return: True if the service is disabled, False otherwise.
        @rtype: Boolean
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def wait_for_device(self, timeout):
        """
        Wait for device
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def clear_logcat(self):
        """"
        Clear all the logcats on the DUT.
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)


    def wait_process_loaded(self, app_name, timeout, refresh=1):
        """
        Wait a process finish its load

        :type app_name: str
        :param app_name: app name to check

        :type  timeout: int
        :param timeout: operation timeout

        :type  refresh: int
        :param refresh: frequency of refresh process state check
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def start_log_monitoring(self, device_logger=None, target_messages=None):
        """
        Start to monitor error log conditions
        :type device_logger : LogCatLogger
        :param device_logger : Android device logger
        :type target_messages : List
        :param target_messages : List of messages to be monitored
        :rtype : None
        :return : None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def stop_log_monitoring(self, device_logger=None, target_messages=None):
        """
        Stop to monitor error log messages
        :type device_logger : LogCatLogger
        :param device_logger : Android device logger
        :type target_messages : List
        :param target_messages : List of messages to stop monitoring
        :rtype : None
        :return : None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def check_logged_errors(self, device_logger=None, target_messages=None):
        """
        Verify logs whether error log messages were triggered or not
        :type device_logger : LogCatLogger
        :param device_logger : Android device logger
        :type target_messages : List
        :param target_messages : List of messages to be checked
        :rtype : None
        :return : None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def check_logged_start_msgs(self, device_logger=None, target_messages=None):
        """
        Verify logs to check start of IP or APP
        :type device_logger : LogCatLogger
        :param device_logger : Android device logger
        :type target_messages : List
        :param target_messages : List of messages to be checked
        :rtype : None
        :return : None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_available_memory(self, memory_top=None):
        """
        Set the amount of available memory
        :type memory_top : Integer
        :param memory_top : Top address of available memory
        :return : None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def restore_available_memory(self):
        """
        Restore the amount of available memory
        :return : None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_number_of_cores(self):
        """
        Get the number of cores
        @return: number of cores
        @rtype: Integer
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_available_scaling_frequencies(self):
        """
        Set the amount of available memory
        @return: List of supported frequencies
        @rtype: Integer Array
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_core_scaling_frequency(self, core_id=None, target_mode=None, target_frequency=None):
        """
        Set maximum scaling frequency of a core
        :type core_id : Integer
        :param core_id : Core ID
        :type target_mode : String
        :param target_mode : Target mode max or min
        :type target_frequency : Integer
        :param target_frequency : Target frequency
        :return : None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)
