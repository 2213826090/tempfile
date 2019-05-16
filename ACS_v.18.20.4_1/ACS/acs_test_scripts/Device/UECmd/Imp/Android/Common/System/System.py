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
:summary: This file implements the System UEcmd for Android device
:since: 08/16/2011
:author: wchen61
"""

import os
import re
import time
import posixpath
import hashlib
from datetime import datetime
from acs_test_scripts.Device.UECmd.UECmdTypes import PACKAGE_STATE
from acs_test_scripts.Device.UECmd.Imp.Android.Common.BaseV2 import BaseV2
from acs_test_scripts.Device.UECmd.Interface.System.ISystem import ISystem
import UtilitiesFWK.Utilities as Util
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException
import subprocess

DEFAULT_FORMAT = "%m%d%H%M%Y.%S"

class System(BaseV2, ISystem):

    """
    :summary: System UEcommands operations for Android platforms using an C{Intent} based communication to the I{DUT}.
    """

    def __init__(self, device):
        """
        Constructor.
        """

        BaseV2.__init__(self, device)
        ISystem.__init__(self, device)
        self._logger = device.get_logger()

        self.retrieve_system_information_component = "com.intel.acs.agent/.SystemInfo"
        self.retrieve_system_information_category = "intel.intents.category.SYSTEMINFO"

        self._tunesound_module = "acscmd.audio.TuneSoundModule"
        self._system_info_class = "acscmd.system.SystemInformationModule"

        self._dialer_package = "com.android.dialer"
        self.dialer_component = self._dialer_package + "/.DialtactsActivity"

    def __set_device_provisioned(self):
        """
        Set device provisioned

        :rtype: bool
        :return: True if update has been done, False otherwise
        """

        # if the xml file exists, we update it and exit
        xmlglobal = "/data/system/users/0/settings_global.xml"
        xmlsecure = "/data/system/users/0/settings_secure.xml"
        if self.update_xml_setting(xmlglobal, "device_provisioned", 1, "setting"):
            self.update_xml_setting(xmlsecure, "user_setup_complete", 1, "setting")
            return True

        # else, update the database
        database = "/data/data/com.android.providers.settings/databases/settings.db"
        global_found = False
        retry = 0
        while not global_found and retry < 3:
            retry = retry + 1
            cmd = "adb shell sqlite3 %s \".tables global\" && echo 0" % database
            output = self._exec(cmd).splitlines()
            if len(output) > 1:
                tbl_provisioned = "global"
                global_found = True
            else:
                time.sleep(60)
                self._logger.info("output provisionning: " + str(output))
        if not global_found:
            tbl_provisioned = "secure"

        cmd = "adb shell sqlite3 %s \"select value from %s where "\
              "name='device_provisioned'\"" % (database, tbl_provisioned)
        is_provisioned = self._exec(cmd).strip(" \n")

        if is_provisioned == "" or is_provisioned == "0":
            request = "delete from %s where name='device_provisioned';" % \
                      tbl_provisioned
            request += "insert into %s (name, value) values ('device_provisioned', '1');" % \
                       tbl_provisioned

            request += "insert into secure (name, value) values ('user_setup_complete', '1');"
            request += "insert into secure (name, value) values ('last_setup_shown', 'eclair_1');"
            self._exec("adb shell sqlite3 %s \"%s\" && echo Ok" % (database, request))
            return True
        else:
            return False

    def __set_setup_complete_policy(self):
        """
        Set setup_complete_policy in device_policies.xml
        """
        xmlfile = "/data/system/device_policies.xml"
        cmd = "adb shell sed -i 's:<policies>:<policies setup-complete=\"true\" />:' %s" % xmlfile
        self._device.run_cmd(cmd, self._uecmd_default_timeout)

    def system_tune_sound(self, soundtype):
        """
        Check the tune sound of different types

        :type  soundtype: str
        :param soundtype: the type of sound to be check

        :rtype: list
        :return: operation status & output log
        """
        function = "tuneSound"
        cmd_args = " --es type %s" % soundtype
        result = self._internal_exec_v2(self._tunesound_module, function, cmd_args)

        if not (result["output"] is None):
            return result["output"]
        else:
            return "No error"

    def adjust_specified_stream_volume(self, sound_type, volume_percent_level):
        """
        Set the volume percent level on the specified stream

        :type  sound_type: str
        :param sound_type: the sound stream

        :type  volume_percent_level: int
        :param volume_percent_level: Volume settings percentage between 0% and 100%

        :rtype: str
        :return: operation status
        """
        if sound_type in ("Bluetooth", "VoiceCall", "Ringtone", "Alarm", "Media"):
            if 0 <= volume_percent_level <= 100:
                self._logger.info("Setting %s volume to %s percent \
                                " % (sound_type, volume_percent_level))
            else:
                msg = "adjust_specified_stream_volume : The volume settings: %s \
                    shall be a percentage between 0 and 100 percent" % volume_percent_level
                self._logger.error(msg)
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
        else:
            msg = "adjust_specified_stream_volume : Sound stream %s is not valid" % str(sound_type)
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        function = "adjustSpecifiedStreamVolume"
        cmd_args = " --es type %s --ei volume %d" % (sound_type, volume_percent_level)
        result = self._internal_exec_v2(self._tunesound_module, function, cmd_args)

        if not (result["output"] is None):
            return result["output"]
        else:
            return "No error"

    def get_specified_stream_volume(self, sound_type):
        """
        get the volume of audio stream

        :type  sound_type: str
        :param sound_type: the sound stream

        :rtype: int
        :return: volume value in percent
        """
        if sound_type not in ("Bluetooth", "VoiceCall", "Ringtone", "Alarm", "Media"):
            msg = "get_specified_stream_volume : Sound stream %s is not valid" % str(sound_type)
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        target_method = "getStreamVolume"
        cmd_args = " --es type %s" % sound_type
        output = self._internal_exec_v2(self._tunesound_module, target_method, cmd_args)
        return int(output[System.OUTPUT_MARKER])

    def retrieve_system_information(self, informationtype, value):
        """
        Check the system information

        :type  informationtype: str
        :param informationtype: the type of information to be retrieved
        :type  value: int or str
        :param value: the value expected

        :rtype: list
        :return: operation status & output log
        """
        target_method = "retrieveSystemInfo"
        cmd = "--es type {0} --es value {1}".format(informationtype, value if value else "reserve")
        output = self._internal_exec_v2(self._system_info_class, target_method, cmd).get('output')
        return output if output else "No error"

    def set_date_and_time(self, date_and_time):
        """
        Set date and time on the device

        :type date_and_time: datetime.datetime
        :param date_and_time: Date and time in python datetime structure (UTC)

        :rtype: list
        :return: operation status & output log.
                 Returns date and time in following format in case of success: "%d/%m %H:%M:%S"
        """
        USER_FORMAT = "%Y-%m-%d %H:%M:%S"

        self._logger.info("Synchronise host & device date & time to %s (UTC)" % date_and_time.strftime(USER_FORMAT))

        host_date_and_time = date_and_time.strftime(DEFAULT_FORMAT)
        cmd = "adb shell date -u {0}".format(host_date_and_time)
        return_code, return_msg = self._device.run_cmd(cmd, self._uecmd_default_timeout)

        if return_code == Util.Global.SUCCESS:
            return_code, return_msg = self._check_date_and_time(date_and_time)

        return return_code, return_msg

    def _check_date_and_time(self, date_and_time):
        """
        Check date and time on the device

        :type date_and_time: datetime.datetime
        :param date_and_time: Date and time in python datetime structure (UTC)

        :rtype: list
        :return: operation status & output log.
                 Returns date and time in following format in case of success: "%d/%m %H:%M:%S"
        """
        host_date_and_time_utc = date_and_time.strftime(DEFAULT_FORMAT)

        cmd = "adb shell date -u +{0}".format(DEFAULT_FORMAT)
        device_date_and_time_utc = self._exec(cmd)

        cmd = "adb shell date +{0}".format(DEFAULT_FORMAT)
        device_date_and_time_local = self._exec(cmd)

        try:
            host_ts_utc = datetime.strptime(host_date_and_time_utc, DEFAULT_FORMAT)
            device_ts_utc = datetime.strptime(device_date_and_time_utc, DEFAULT_FORMAT)
            device_ts_local = datetime.strptime(device_date_and_time_local, DEFAULT_FORMAT)
        except ValueError as error:
            return_msg = str(error)
            return Util.Global.FAILURE, return_msg

        # Theoretically, device_ts and host_ts are equals
        # Practically, they may have a gap of several seconds (usually 2s)
        # If the gap is upper than 10 seconds, there might be
        # a mistake.
        time_delta = device_ts_utc - host_ts_utc
        # Workaround to compute total seconds in python2.6
        # https://bitbucket.org/wnielson/django-chronograph/issue/27/python26-datatimetimedelta-total_seconds
        if hasattr(time_delta, "total_seconds"):
            duration = time_delta.total_seconds()
        else:
            duration = (time_delta.microseconds + (time_delta.seconds + time_delta.days * 24 * 3600) * 10 ** 6) / 10 ** 6

        if abs(duration) > 30:
            return_code = Util.Global.FAILURE
            return_msg = "Synchronisation issue: " \
                         "Host (UTC) '{0}', Device (UTC) '{1}', Delta '{2}'".format(host_ts_utc,
                                                                              device_ts_utc,
                                                                              time_delta)
            self._logger.warning(return_msg)
        else:
            return_code = Util.Global.SUCCESS
            return_msg = datetime.strftime(device_ts_local, "%d/%m %H:%M:%S")

        return return_code, return_msg

    def _set_toybox_date_time(self, date_and_time):
        """
        Set date and time on devices using toybox
        Some versions of toybox do not have the same format and -s option does not work
        so we just need to run "date mmddHHMM[YYYY][.SS]"

        :type date_and_time: datetime.datetime
        :param date_and_time: Date and time in python datetime structure
        """
        host_date_and_time_for_toybox = date_and_time.strftime("%m%d%H%M%Y.%S")
        set_date_cmd_toybox = "adb shell type toybox && toybox date {0}".format(host_date_and_time_for_toybox)
        _, toybox_msg = self._device.run_cmd(set_date_cmd_toybox, self._uecmd_default_timeout)
        if "not found" in toybox_msg.lower():
            self._logger.info("The device is not using toybox, cannot set the date at toybox format")
        elif "unknown command" in toybox_msg.lower():
            self._logger.info("'date' command is not provided by toybox, cannot set the date at toybox format")
        else:
            self._logger.info("'date' command is provided by toybox, we have set it with the correct format")


    def set_timezone(self, timezone):
        """
        Set device timezone.

        :type  timezone: str
        :param timezone: Timezone to set on the device

        :rtype: list
        :return: operation status & output log. Returns the timezone in case of success.
        """
        set_timezone_property_cmd = "adb shell setprop persist.sys.timezone '%s'" % timezone
        return_code, return_msg = self._device.run_cmd(set_timezone_property_cmd, self._uecmd_default_timeout)

        if return_code == Util.Global.SUCCESS:
            timezone_intent_cmd = "adb shell am broadcast -a android.intent.action.TIMEZONE_CHANGED" \
                                  " --receiver-replace-pending --es time-zone '{0}'".format(timezone)
            return_code, return_msg = self._device.run_cmd(timezone_intent_cmd, self._uecmd_default_timeout)

            timezone_property = self._exec("adb shell getprop persist.sys.timezone")

            if return_code == Util.Global.SUCCESS and timezone == timezone_property:
                return_msg = timezone_property
            else:
                return_code = Util.Global.FAILURE
                return_msg = "Cannot set timezone to {0} ! ({1})".format(timezone, return_msg)

        return return_code, return_msg

    def configure_launcher(self):
        """
        Configure the launcher
            - Remove Ok button on home screen and on apps screen
            - Remove other launchers
        """
        xmlcontent = "<?xml version='1.0' encoding='utf-8' standalone='yes' ?>\n" \
                     "<map>\n"\
                     "<boolean name='cling.allapps.dismissed' value='true' />\n" \
                     "<boolean name='cling.workspace.dismissed' value='true' />\n"\
                     "</map>"

        launcher_dir = "/data/data/com.android.launcher"
        launcher_prefs_dir = "%s/shared_prefs" % launcher_dir
        launcher_file = "%s/com.android.launcher2.prefs.xml" % launcher_prefs_dir

        cmd = "rm -r %s/databases 2> /dev/null; " \
              "mkdir -p %s && " \
              "echo \"%s\" > %s" % (launcher_dir, launcher_prefs_dir,
                                    xmlcontent, launcher_file)
        self._exec("adb shell " + cmd, 10)

    def skip_wizard(self):
        """
        Remove the wizard

        :rtype: boolean
        :return: True if reboot is mandatory, False otherwise
        """
        self._exec("adb remount", 10)

        reboot_mandatory = self.__set_device_provisioned()
        self.__set_setup_complete_policy()

        cmd = "pm disable com.google.android.setupwizard"
        self._exec("adb shell " + cmd, 10)

        return reboot_mandatory

    def clear_cache(self):
        """
        Clear the buffer and cache prior
        """
        self._exec("adb shell sync && echo 3 > /proc/sys/vm/drop_caches", 30)

    def pid_of(self, process_name):
        """
        Get the pid of a process by its name.

        :rtype: str
        :return: the pid of the process or empty str if process as not been found.
        """
        cmd = "adb shell pidof " + str(process_name)
        cmd = self._device.format_cmd(cmd)

        self._logger.info("Getting the pid of the process with name: %s" % process_name)
        (_result, pid) = Util.internal_shell_exec(cmd, 10)
        if not pid.isdigit() and not pid == "":
            raise DeviceException(DeviceException.INTERNAL_EXEC_ERROR,
                                  "The pid should be a int. Is: %s" % str(pid))
        self._logger.debug("PID of %s is: %s" % (process_name, pid))
        return pid

    def get_dumpsys_focus(self):
        """
        get the Windows dumpsys information on Focus (open window in UI)

        :rtype: str
        :return: focus contain the name of display window.
        """
        log = "get dumpsys focus : "

        self._logger.debug(log + "start")

        focus = self._exec("adb shell dumpsys window windows | grep Focus", 3)

        return str(focus)

    def check_Activity(self, activity_name):
        """
        Check if activity_name is launch.
        :type activity_name: str
        :param activity_name: Name of research activity.
        :rtype: bool
        :return: True if window name is find, False if doesn't find.
        """
        find = False
        # get the Windows dumpsys information on Focus (open window in UI)
        focus = self.get_dumpsys_focus()
        # Check if window name is in Windows dumpsys information
        if activity_name in focus:
            find = True
        return find

    def open_Dialer(self):
        """
        Open dialer application.
        """
        self._logger.info("Launch Dialer application")
        # Launch application Dialer
        self._exec("adb shell am start %s" % self.dialer_component)

    def kill_Dialer_app(self):
        """
        Kills the dialer application.
        """

        self._logger.info("Killing the Dialer application")
        cmd = "adb shell am force-stop com.android.dialer"
        self._exec(cmd)

    def kill_Dialer_app_if_already_launch(self):
        """
        Kills the dialer application if already launch.
        """

        self._logger.info("Check if Dialer is already launch.")
        app = self._exec("adb shell ps | grep dialer")
        if app != "":
            self._logger.info("Dialer is already launch, kill application.")
            self.kill_Dialer_app()
        else:
            self._logger.info("Dialer is not start.")

    def set_filesystem_rw(self):
        """
        Set the file system in read/write mode.
        """
        self._logger.info("Set filesystem to read-write (all partitions)")
        max_retry = 3
        tries = 0
        return_code = Util.Global.FAILURE
        return_msg = "Number of max retries ({0}) reached.".format(max_retry)

        while tries < max_retry and return_code != Util.Global.SUCCESS:
            return_code, return_msg = self._device.run_cmd("adb remount", 5, True)
            tries += 1

        if return_code == Util.Global.FAILURE:
            error_msg = "Failed to set filesystem to read-write: {0}".format(return_msg)
            raise DeviceException(DeviceException.FILE_SYSTEM_ERROR, error_msg)

    def _run_cmd_and_check_failure(self, cmd, timeout, force_execution=False, wait_for_response=True, silent_mode=False):
        """
        Execute the input command and return the result message.
        If output message contains 'failure', 'no such file' or 'not found', returns an error.
        If the timeout is reached, return an exception

        :type  cmd: str
        :param cmd: cmd to be run

        :type  timeout: int
        :param timeout: Script execution timeout in ms

        :type  force_execution: bool
        :param force_execution: Force execution of command without check device connected (dangerous)

        :type  wait_for_response: bool
        :param wait_for_response: Wait response from adb before stating on command

        :rtype: tuple (int, str)
        :return: Output status and output log
        """
        return_code, return_msg = self._device.run_cmd(cmd, timeout, force_execution, wait_for_response, silent_mode)

        if any([x in return_msg for x in ["Error", "Failure", "No such file", "not found"]]):
            return_code = Util.Global.FAILURE
        return return_code, return_msg

    def install_device_executable(self, bin_path, destination, timeout=0):
        """
        Install a binary as device executable file

        :type bin_path: str
        :param bin_path: file to be installed

        :type  destination: str
        :param destination: destination on device

        :type  timeout: int
        :param timeout: operation timeout

        :rtype: tuple (bool, str)
        :return: Output status and output log
        """
        status = Util.Global.FAILURE
        status_msg = "Undefined error while pushing binary on device"
        binary_filename = os.path.basename(bin_path)

        if os.path.isfile(bin_path):
            if destination is not None:
                if self._device.is_rooted():
                    # Initialize timeout if not set
                    if not timeout:
                        timeout = Util.compute_timeout_from_file_size(bin_path, self._device.min_file_install_timeout)

                    # Set filesystem into RW
                    self.set_filesystem_rw()

                    # Build push, chmod commands
                    dest_path = posixpath.join(destination, binary_filename)
                    bin_push_cmd = 'adb push "{0}" "{1}"'.format(bin_path, dest_path)
                    bin_chmod_cmd = "adb shell chmod 755 {0}".format(dest_path)
                    # Push the binary file
                    self._logger.info("Pushing %s -> %s ..." % (str(bin_path), str(dest_path)))
                    status, status_msg = self._run_cmd_and_check_failure(bin_push_cmd, timeout)
                    if status == Util.Global.SUCCESS:
                        # Modify access right on the binary file
                        status, status_msg = self._run_cmd_and_check_failure(bin_chmod_cmd, timeout)
                        if status == Util.Global.SUCCESS:
                            status_msg = "Binary has been successfully installed on the phone!"
                            # Remount /system as read only
                            status, remount_status_msg = \
                                self._run_cmd_and_check_failure("adb shell mount -o remount ro /system", 10)
                            if status != Util.Global.SUCCESS:
                                status_msg += " But cannot remount system partition in RO ({0})".format(
                                    remount_status_msg)
                                self._logger.warning(remount_status_msg)

                else:
                    status_msg = "Need to be root to install binaries, %s not installed" % (bin_path,)
                    self._logger.warning(status_msg)
            else:
                status_msg = "Impossible to push %s, no destination given" % (str(bin_path),)
        else:
            status_msg = "Impossible to install, no file given"

        return status, status_msg

    def remove_device_files(self, device_directory, filename_regex):
        """
        Remove file on the device

        :type device_directory: str
        :param device_directory: directory on the device

        :type  filename_regex: str
        :param filename_regex: regex to identify file to remove

        :rtype: tuple (bool, str)
        :return: Output status and output log
        """
        status_msg = "Cannot remove file on device folder {0}".format(device_directory)
        cmd = "adb shell find {0} -maxdepth 1 -type f -name '{1}'".format(device_directory, filename_regex)
        status, output = self._device.run_cmd(cmd, self._uecmd_default_timeout, wait_for_response=True, silent_mode=True)

        if status != Util.Global.SUCCESS:
            status_msg += " ({0})".format(output)
        elif output:
            cmd = "adb shell rm {0}".format(" ".join(output.split('\n')))
            status, msg = self._run_cmd_and_check_failure(cmd, self._uecmd_default_timeout, wait_for_response=True)
            if status == Util.Global.SUCCESS:
                status_msg = "Files : {0} have been successfully " \
                             "removed from device".format(" ".join(output.split('\n')))
        else:
            status = Util.Global.SUCCESS
            status_msg = "Nothing to remove, no file matching {0} in {1} !".format(filename_regex, device_directory)

        return status, status_msg

    def remove_device_dir(self, device_directory):
        """
        Remove directory from the device

        :type device_directory: str
        :param device_directory: directory on the device

        :rtype: tuple (bool, str)
        :return: Output status and output log
        """
        cmd = "adb remount"
        status, output = self._device.run_cmd(cmd, self._uecmd_default_timeout, wait_for_response=True, silent_mode=True)
        status_msg = "Cannot directory from the deivce {0}".format(device_directory)
        cmd = "adb shell rm -rf {0}".format(device_directory)
        self._logger.info(cmd)
        status, output = self._device.run_cmd(cmd, self._uecmd_default_timeout, wait_for_response=True, silent_mode=True)
        self._logger.info(output)
        if status != Util.Global.SUCCESS:
            status_msg += " ({0})".format(output)
        else:
            status = Util.Global.SUCCESS
            status_msg = "Nothing to remove, no directory matching {0}!".format(device_directory)

        return status, status_msg

    def click_event_not_found(self, evt, response):
        """
        In the response there must be the button event (e.g. BTN_LEFT) and "DOWN" and "UP"
        """
        return not (response and evt.strip() in response and "DOWN" in response and "UP" in response)

    def check_hid_events(self, device_name, events, timeout=None):
        """
        Finds out the device id (in the form of dev/input/eventX, given the device's name.
        Then uses getevent utility to read events from the given device.
        It compares the event reads with the passed ones, to check if the expected events are received.

        :type device_name: str
        :param device_name: name of the device. it is possible to find it out with:
                "adb shell getevent -p" (the device must be paired and connected to the device).
        :type events: []
        :param events: list of the expected events. possible values are:
            BTN_LEFT, BTN_RIGHT, BTN_MIDDLE, REL_X, REL_Y, REL_WHEEL

        :type timeout: int
        :param timeout: timeout for receiving HID events

        :return: True if events have been received, False otherwise
        """

        self._validate_hid_events_arg(events)
        device_id = self._get_hid_device_by_name(device_name)
        event_nr = 8
        # "getevent -c xxx" argument tells getevent to stop after receiving xxx events.
        # The reason is that each click of a mouse's button generates 6 events, 3 for button down and 3 for button up.
        # But for 1st event, there's 2 more ones at the top, that's why we start with 8
        for evt in events:
            cmd = "adb shell getevent -c %d -l %s" % (event_nr, device_id)
            response = self._exec(cmd, timeout)
            event_nr = 6
            self._logger.debug("looking for %s in getevent response: %s" % (evt, response))
            if self.click_event_not_found(evt, response):
                return False
        return True

    def check_hid_events_generic(self, device_name, events, serial_name, timeout=None):
        """
        Finds out the device id (in the form of dev/input/eventX, given the device's name.
        Then uses getevent utility to read events from the given device.
        It compares the event reads with the passed ones, to check if the expected events are received.

        This method is different from check_hid_events above in the fact that is does not expect for a single type of
        event (e.g. mouse button press), but it waits for any possible HID event generator action.
        :type device_name: str
        :param device_name: name of the device. it is possible to find it out with:
                "adb shell getevent -p" (the device must be paired and connected to the device).
        :type events: []
        :param events: list of the expected events. possible values are in the TEST Step description Catalog. This step
        checks for any possible events, but if any new are discovered, they can be added there.

        :type timeout: int
        :param timeout: timeout for receiving HID events

        :return: True if events have been received, False otherwise
        """

        device_id = self._get_hid_device_by_name(device_name)
        for evt in events:
            cmd = ["timeout",
                   "--preserve-status",
                   "{0}".format(timeout),
                   "adb",
                   "-s",
                   "{0}".format(serial_name),
                   "shell",
                   "getevent",
                   "-l",
                   "{0}".format(device_id)]
            self._logger.info("executing command: " + str(cmd))
            popen = subprocess.Popen(cmd, stdout=subprocess.PIPE)
            popen.wait()
            response = popen.stdout.read()
            self._logger.debug("looking for %s in getevent response: %s" % (evt, response))
            if self.click_event_not_found(evt, response):
                return False
        return True

    def _validate_hid_events_arg(self, events):
        """
        Raise an error if one of the events is invalid
        """
        for evt in events:
            if evt not in ["BTN_LEFT", "BTN_RIGHT", "BTN_MIDDLE", "REL_X", "REL_Y", "REL_WHEEL"]:
                msg = "Invalid event argument (%s)" % evt
                self._logger.error(msg)
                raise DeviceException(DeviceException.INVALID_PARAMETER, msg)

    def _get_hid_device_by_name(self, device_name):
        """
        Return the device id by its name
        """
        # Returns the list of devices
        cmd = "adb shell getevent -p"
        response = self._exec(cmd)
        reg_ex = "add device [0-9]+[:]{1} (\/dev\/input\/event[0-9]+)[\s.]+name:(.*)\"%s\"" % device_name

        found = re.search(reg_ex, response)
        if found is None:
            msg = "Bluetooth device not found (%s)" % device_name
            self._logger.error(msg)
            raise DeviceException(DeviceException.INVALID_PARAMETER, msg)

        return found.group(1)

    def get_installed_launchers(self, open_chooser=False):
        """
        Retrieve a list of installed and enabled launchers on device

        :type open_chooser: bool
        :param open_chooser: Indicating if have to force opening
                            of a launcher windows chooser.

        :rtype: list
        :return: The list of installed launcher
        """
        self._logger.debug("Getting installed launchers on device")
        target_method = "retrieveInstalledLaunchers"

        cmd = "--ez open_chooser %s" % str(open_chooser)

        output = self._internal_exec_multiple_v2(self._system_info_class, target_method, cmd)
        launcher_list = []
        for launcher in output:
            launcher_list.append(launcher["package_name"])

        self._logger.debug("Installed launchers are: %s" % str(launcher_list))
        return launcher_list

    def get_package_state(self, package_name):
        """
        Get the given android package state

        :type package_name: str
        :param package_name: The android packege name to check.
                            As android ComponentName format (eg. com.android.launcher)

        :rtype: UeCmdTypes.PACKAGE_STATE
        :return: The package state corresponding to given package, default unknown
        """
        self._logger.debug("Getting %s state ..." % package_name)
        state = PACKAGE_STATE.UNKNOWN
        if package_name in self._get_package_list(PACKAGE_STATE.ENABLED):
            state = PACKAGE_STATE.ENABLED
        elif package_name in self._get_package_list(PACKAGE_STATE.DISABLED):
            state = PACKAGE_STATE.DISABLED
        self._logger.debug("%s state is %s" % (package_name, str(state)))
        return state

    def set_package_state(self, package_name, state):
        """
        Set the given package to given state.

        :type package_name: str
        :param package_name: The android packege name to check.
                            As android ComponentName format (eg. com.android.launcher)

        :type state: UeCmdTypes.PACKAGE_STATE
        :param state: The package state to set

        :raise AcsConfigException: If a parameter name is not set or state invalid
        :raise DeviceException: If unable to change package state
        :rtype: None
        """
        if package_name in [None, ""]:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "Invalid package name !")

        if state in [PACKAGE_STATE.ENABLED, PACKAGE_STATE.DISABLED]:
            state_param = str(state)[:-1].lower()
            self._logger.debug("%s %s package ..." % (state_param, package_name))
            output = self._exec("adb shell pm %s %s" % (state_param, package_name))
        else:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "Invalid package state !")

        check_string = "%s new state: %s" % (package_name, str(state).lower())
        if check_string not in output:
            raise DeviceException(DeviceException.PHONE_OUTPUT_ERROR, "Unable to change package state")

    def _get_package_list(self, package_state=PACKAGE_STATE.ALL):
        """
        Get a list of android packages of given state

        :type package_state: UeCmdTypes.PACKAGE_STATE
        :param package_state: The package state to search , default all

        :rtype: list
        :return: List of android package corresponding to given state
        """
        if package_state == PACKAGE_STATE.ENABLED:
            option = "-e"
        elif package_state == PACKAGE_STATE.DISABLED:
            option = "-d"
        else:
            # List all
            option = ""

        cmd = "adb shell pm list packages %s" % option
        p_list = self._exec(cmd).splitlines()
        # remove 'package:' in all element of list
        p_list = map(lambda x: x.replace("package:", ""), p_list)
        self._logger.debug("%s packages are: %s" % (package_state, str(p_list)))
        return p_list

    def run_shell_executable(self, exe, args, io_redirection=0, timeout=20):
        """
        Execute a shell script or binary.  This also sets permissions to rwxr-xr-x on the file to be sure it can execute.

        :type exe: str
        :param exe: Path and filename of script or binary to execute

        :type args: str
        :param args: Command line arguments to use with the executable

        :type io_redirection: int
        :param io_redirection: Determines what to do with stdout and stderr I/O.
            0 = do not redirect it; let it be returned in output_msg and check for common error messages.
            1 = redirect both to /dev/null (lost forever)
            2 = redirect stdout to stdout.log and stderr to stderr.log in the same directory as the executable

        :type timeout: int
        :param timeout: Number of seconds to wait for 'adb shell' to return.

        :raise AcsConfigException: If exe or args are not specified
        :raise DeviceException: If set_file_permission or run_cmd fails.
        :rtype: None
        """
        if not exe or not args:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "uecmd run_shell_executable: executable or arguments were not specified")

        path = "/".join(exe.split("/")[:-1])
        filename = exe.split("/")[-1]
        file_api = self._device.get_uecmd("File")

        # Make sure we have permission to execute.
        # We'll also rely on this to check existence of the file rather than doing a separate check.
        (return_code, output_msg) = file_api.set_file_permission(exe, "rwxr-xr-x")
        if not return_code:
            msg = "uecmd run_shell_executable: Error '%s' when trying to set permissions on '%s'" % (output_msg, exe)
            self._logger.error(msg)
            raise DeviceException(DeviceException.FILE_SYSTEM_ERROR, msg)

        # See if user wants to send stdout and stderr somewhere else.
        if io_redirection == 1:
            io_suffix = "&>/dev/null"
        elif io_redirection == 2:
            io_suffix = "1>stdout.log 2>stderr.log"

        # If a path is given to the file, make that the working directory before we invoke the executable.
        if path != "":
            cmd_line = "adb shell cd {0}; ./{1} {2} {3}".format(path, filename, args, io_suffix)
        else:
            cmd_line = "adb shell ./{0} {1} {2}".format(filename, args, io_suffix)

        (return_code, output_msg) = self._run_cmd_and_check_failure(cmd_line, timeout)
        if return_code != Util.Global.SUCCESS:
            msg = "uecmd run_shell_executable: Error '%s' when running '%s'" % (output_msg, cmd_line)
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

    def check_device_file_install(self, source_file_path, destination_file_path):
        """
        Check if the file installation has succeeded on a device

        :type file_path: str
        :param file_path: file to be installed

        :type  destination: str
        :param destination: destination on device

        :rtype: tuple (bool, str)
        :return: (Output status = True if file is installed on the device else False, output error message)

        """
        status = Util.Global.FAILURE
        status_msg = "Undefined error while checking file checksum on device (source fil e= %s, destination file = %s)" \
                     % (str(source_file_path), str(destination_file_path))

        # Check source file path + destination file path
        if os.path.isfile(source_file_path):
            if destination_file_path is not None:
                # adb shell md5 /sdcard/acs_files/1MB.zip
                # 6db371d446c028e4f984254b8178ec52 /sdcard/acs_files/1MB.zip
                # checksum file_name
                file_size = os.path.getsize(source_file_path)

                _, output = self._device.run_cmd(cmd="adb shell which md5sum md5", timeout=self._uecmd_default_timeout,
                                              silent_mode=True, force_execution=True)
                if "not found" not in output and output.splitlines(): # take the first one (if exists)
                    md5_cmd = output.splitlines()[0]
                else:
                    md5_cmd = "md5"

                return_result, return_message = self._device.run_cmd(cmd="adb shell %s %s" % (md5_cmd, destination_file_path),
                                                    timeout=(self._uecmd_default_timeout + (file_size/1000000000)*60), silent_mode=True,
                                                    force_execution=True)
                if return_result != Util.Global.FAILURE and "No such file" not in return_message and "not found" not in return_message:
                    # Retrieve the checksum of the file installed on the device
                    dest_file_md5_checksum, _ = return_message.split()

                    # Compute the checksum of the file to install
                    # if file size is superior to 1Gbyte, retrieve checksum in many times
                    if file_size < 1000000000 :
                        orig_file_checksum = hashlib.md5(open(source_file_path, 'rb').read()).hexdigest()
                    else :
                        orig_file_checksum = hashlib.md5()
                        read_size = 10000000
                        with open(source_file_path, 'rb') as f:
                            data = f.read(read_size)
                            while data:
                                orig_file_checksum.update(data)
                                data = f.read(read_size)
                            orig_file_checksum = orig_file_checksum.hexdigest()

                    # Compare cumputed checksum
                    if orig_file_checksum == str(dest_file_md5_checksum):
                        status = Util.Global.SUCCESS
                        status_msg = "File already exist at : %s" % (str(destination_file_path))
                else:
                    status_msg = "File is not accessible: %s" % (str(destination_file_path))
            else:
                status_msg = "No destination file path given: %s" % (str(destination_file_path))
        else:
            status_msg = "Source file to check does not exist: %s" % (str(source_file_path))

        return status, status_msg

    def install_scripts_from_lib(self, relative_source_path, timeout=0):
        """
        Copy all files from a specified subdirectory in acs_test_scripts/Lib/ShellScripts/<OS>, to a standard
        location on the DUT.  <OS> is "Android", "Windows" or "Linux", depending on what the device is running.
        The directory on the DUT is based on a hardcoded location, depending on the OS:
            Android: /data/<relative_source_path>
            Windows: TBD/<relative_source_path>
            Linux: TBD/<relative_source_path>
        Also ensure that the scripts are executable and have Unix line endings.

        :type relative_source_path: str
        :param relative_source_path: relative path to the files under acs_test_scripts/Lib/ShellScripts/<OS>

        :type  timeout: int
        :param timeout: operation timeout

        :raise AcsConfigException: If relative_source_path does not exist under acs_test_scripts/Lib/ShellScripts/<OS>
        :raise DeviceException: If push fails
        :rtype: str
        :return: absolute path where files were put on the DUT
        """

        dest_path = self._device.get_device_os_path().join(self._device.binaries_path, relative_source_path)
        source_path = os.path.normpath(os.path.join(os.path.dirname(__file__), "../../../../../..", "Lib", "ShellScripts", "Android", relative_source_path))
        if not os.path.exists(source_path):
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "install_scripts_from_lib: Cannot find source path ({0})".format(source_path))

        # Make sure there are no DOS line endings
        for filename in os.listdir(source_path):
            path = os.path.join(source_path, filename)
            text = open(path, "U").read()
            text = text.replace("\r\n", "\n")
            open(path, "wb").write(text)

        push_cmd = 'adb push "{0}" "{1}"'.format(source_path, dest_path)
        chmod_cmd = "adb shell chmod 755 {0}/*".format(dest_path)

        # Push the files
        self._logger.info("uecmd install_scripts_from_lib: Pushing {0} -> {1} ...".format(source_path, dest_path))
        status, status_msg = self._run_cmd_and_check_failure(push_cmd, timeout)
        if status != Util.Global.SUCCESS:
            raise DeviceException(DeviceException.OPERATION_FAILED, "install_scripts_from_lib: Failed to push files ({0})".format(status_msg))
        # Ensure that scripts are executable
        status, status_msg = self._run_cmd_and_check_failure(chmod_cmd, timeout)
        if status != Util.Global.SUCCESS:
            raise DeviceException(DeviceException.OPERATION_FAILED, "install_scripts_from_lib: Failed to set permissions ({0})".format(status_msg))
        return dest_path

    def disable_google_voice_hotword(self):
        """
        Disables the Google voice hotword feature.

        @return: True if the service is disabled, False otherwise.
        @rtype: Boolean
        """
        self._logger.warning("Not implemented")
        return False

    def disable_voiceinteraction(self):
        """
        Disables the Google voiceinteraction service (related to hotword feature)

        @return: True if the service is disabled, False otherwise.
        @rtype: Boolean
        """
        self._logger.warning("Not implemented")
        return False

    def disable_voicerecognition(self):
        """
        Disables the Google voicerecognition service (related to hotword feature)

        @return: True if the service is disabled, False otherwise.
        @rtype: Boolean
        """
        self._logger.warning("Not implemented")
        return False

    def wait_for_device(self, timeout):
        """
        Wait for device
        """

        status, msg = self._device.run_cmd("adb wait-for-device",
                                           timeout, True)
        return status == 0

    def clear_logcat(self):
        """"
        Clear all the logcats on the DUT.
        """
        self._exec("adb logcat -c")
        for aplog_folder in ["/logs", "/data/logs"]:
            self._exec("adb shell rm %s/aplogs/*" % aplog_folder)
            self._exec("adb shell echo "" > %s/aplog" % aplog_folder)

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
        start = time.time()
        stop = start + timeout

        # check process is alive
        _, output = self._device.run_cmd("adb shell ps|grep '%s$'|sed 's/[[:space:]]\+/ /g' |cut -d' ' -f2" % app_name, 3)
        if not output.isdigit():
            raise DeviceException(DeviceException.OPERATION_FAILED,
                                  "Can't find '{0}' process ID".format(app_name))
        browser_pid = int(output)
        # check progress only if timeout a positive value
        in_progress = timeout > 0
        prev_load = 0

        while time.time() < stop and in_progress:
            time.sleep(refresh)

            cmd = "adb shell ps -x %s|tail -n1|sed 's/.*(u:\([0-9]*\), s:[0-9]*)/\\1/'" % browser_pid
            _, output = self._device.run_cmd(cmd, 3)
            if output.isdigit():
                cur_load = int(output)

            if prev_load + 5 < cur_load:
                prev_load = cur_load
            else:
                in_progress = False

        if in_progress:
            raise DeviceException(DeviceException.TIMEOUT_REACHED,
                                  "Timeout reached")

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
        for triglog in target_messages:
            device_logger.add_trigger_message(triglog)

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
        for triglog in target_messages:
            device_logger.remove_trigger_message(triglog)

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
        for triglog in target_messages:
            time.sleep(1)
            messages = device_logger.get_message_triggered_status(triglog)
            if messages is not None:
                if len(messages) > 0:
                    msg = "Failing test b/c the following message appeared in log: %s" % triglog
                    raise DeviceException(DeviceException.OPERATION_FAILED, msg)

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
        for triglog in target_messages:
            time.sleep(1)
            messages = device_logger.get_message_triggered_status(triglog)
            if len(messages) == 0:
                msg = "Failing test, could not find this message to indicate successful start of an app or activity: %s" % triglog
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

    def stop_app(self, package):
        """
        Stop any app using its package name
        """
        self._exec("adb shell am force-stop %s" % package)

    def set_available_memory(self, memory_top=None):
        """
        Set the amount of available memory
        :type memory_top : Integer
        :param memory_top : Top address of available memory
        :return : None
        """
        try:
            val = int(memory_top)
        except ValueError:
            msg = "Input value {0} is not an integer".format(memory_top)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        memory_top_hex = "0x%0.8X" % memory_top

        cmdline = 'mkdir /data/OSbV_backup;'
        cmdline += 'cat /d/osip/cmdline > /data/OSbV_backup/origCmdline;'
        cmdline += 'filecontent=$(</data/OSbV_backup/origCmdline);'
        cmdline += 'echo "${filecontent} mem=%s" > /data/OSbV_backup/myCmdline;' % memory_top_hex
        cmdline += 'cat /data/OSbV_backup/myCmdline > /d/osip/cmdline'
        self._exec('adb shell ' + cmdline)

        output = self._exec('adb shell "ls /data/OSbV_backup/origCmdline | wc -l"')
        if (output < 1):
            msg = "Your platform doesn't support kernel commandline parameter customization so we cannot change available memory."
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

    def restore_available_memory(self):
        """
        Restore the amount of available memory
        :return : None
        """
        output = self._exec('adb shell "ls /data/OSbV_backup/origCmdline | wc -l"')
        if (output < 1):
            msg = "Your platform doesn't have an original kernel commandline setting to be restored."
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        cmdline = 'cat /data/OSbV_backup/origCmdline > /d/osip/cmdline'
        self._exec('adb shell ' + cmdline)

    def get_number_of_cores(self):
        """
        Get the number of cores
        @return: number of cores
        @rtype: Integer
        """
        output = self._exec('adb shell find . -maxdepth 1 -type d -name "cpu[0-9]" -print  | wc -l')

        return int(output)

    def get_available_scaling_frequencies(self):
        """
        Set the amount of available memory
        @return: List of supported frequencies
        @rtype: Integer Array
        """
        output = self._exec('adb shell cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_available_frequencies')
        out_intlist = map(int, output.split(' '))

        return out_intlist

    def set_core_scaling_frequency(self, core_id=None, target_mode=None, target_frequency=None):
        """
        :type core_id : Integer
        :param core_id : Core ID
        :type target_mode : String
        :param target_mode : Target mode max or min
        :type target_frequency : Integer
        :param target_frequency : Target frequency
        :return : None
        """

        #Validate Core ID parameter
        try:
            val = int(core_id)
            number_of_cores = self.get_number_of_cores()

            if core_id > (number_of_cores - 1):
                msg = "The system has only {0} cores. You tried to set core {1}".format(number_of_cores, core_id, target_frequency)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)
        except ValueError:
            msg = "Input value {0} is not an integer".format(memory_top)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        #Validate the target frequency
        try:
            val = int(target_frequency)
            supported_frequencies = self.get_available_scaling_frequencies()

            if val not in supported_frequencies:
                msg = "The system doesn't support {0} for scaling frequency.".format(val)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)
        except ValueError:
            msg = "Input value {0} is not an integer".format(memory_top)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        #Validate the target mode
        if (target_mode != "max") and (target_mode != "min"):
            msg = "Input value {0} is invalid for frequency mode".format(target_mode)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        self._exec('adb shell cat {0} > /sys/devices/system/cpu/cpu{1}/cpufreq/scaling_{2}_freq'.format(target_frequency, core_id, target_mode))

    def get_uptime(self):
        """
        Get the device uptime
        """
        output = self._exec("adb shell cat /proc/uptime", force_execution=True)
        if output and len(output.split()):
            uptime = output.split()[0]
            if uptime.replace(".", "").isdigit():
                return float(uptime)
        return 0.
