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
:summary: This file implements the application management UECmd for Android devices.
:since: 10/12/2014
:author: vdechefd
"""

import os
import re
import tempfile
import time
from acs_test_scripts.Device.UECmd.Imp.Android.Common.BaseV2 import BaseV2
from acs_test_scripts.Device.UECmd.Interface.System.IAppMgmt import IAppMgmt
import UtilitiesFWK.Utilities as Util
from Utilities.TextFileUtilities import modify_text_file, add_line_before_pattern
from ErrorHandling.AcsToolException import AcsToolException
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException


class AppMgmt(BaseV2, IAppMgmt):

    """
    :summary: Application management UEcommands operations
              for Android platforms using an C{Intent} based communication to the I{DUT}.
    """

    def __init__(self, device):
        """
        Constructor.
        """
        BaseV2.__init__(self, device)
        IAppMgmt.__init__(self, device)
        self._logger = device.get_logger()

    def install_device_app(self, app_path, timeout=0, allow_downgrade=False):
        """
        Install a device application

        :type app_path: str
        :param app_path: file to be installed

        :type  timeout: int
        :param timeout: operation timeout

        :type  allow_downgrade: bool
        :param allow_downgrade: allow the downgrade of application

        :rtype: tuple (bool, str)
        :return: Output status and output log
        """
        status = Util.Global.FAILURE
        status_msg = "Undefined error while installing application on device"
        cmd_options = "-r"
        if allow_downgrade:
            cmd_options += " -d"

        if os.path.isfile(app_path):
            # Define the timeout from the file size
            if not timeout:
                timeout = Util.compute_timeout_from_file_size(app_path, self._device.min_file_install_timeout)

            self._logger.info("Installing {0} ...".format(app_path))
            cmd = 'adb install {0} "{1}"'.format(cmd_options, app_path)
            status, status_msg = self._run_cmd_and_check_failure(cmd, timeout)
        else:
            status_msg = "Impossible to install {0}, file does not exist".format(app_path)

        return status, status_msg

    def backup_app(self, backup_file, app_location, timeout=0):
        """
         Backup application to host file from app location on device

        :type  backup_file: str
        :param backup_file: file path of backup file

        :type  app_location: str
        :param app_location: file path of application on the device

        :type  timeout: int
        :param timeout: time before issuing command

        :rtype: str, str
        :return: status and associated error message
        """

        cmd = "adb pull {0} {1}".format(app_location, backup_file)
        status, status_msg = self._run_cmd_and_check_failure(cmd, timeout)
        if status != Util.Global.SUCCESS:
            status_msg = "App backup has failed ({0})".format(status_msg)
        return status, status_msg

    def get_path_of_device_app(self, package_name):
        """
        Retrieve the path on device of device application

        :type  package_name: str
        :param package_name: package name of the device application

        :rtype: str
        :return: device path to the application
        """
        app_location = ""
        cmd = "adb shell pm list packages -f"
        run_cmd_status, run_cmd_msg = self._device.run_cmd(cmd,
                                                           timeout=self._uecmd_default_timeout,
                                                           wait_for_response=True,
                                                           silent_mode=True)
        if run_cmd_status == Util.Global.SUCCESS and run_cmd_msg:
            for package in run_cmd_msg.split('\n'):
                expr_name = "package:(?P<app_location>.*)=(?P<package_name>.*)"
                matches_str = re.compile(expr_name).search(package)
                if matches_str is not None and package_name == matches_str.group("package_name"):
                    app_location = matches_str.group("app_location")
                    break
        else:
            app_location = ""
        return app_location

    def get_package_name_of_device_app(self, app_name):
        """
        Retrieve package name from an android application

        :type  app_name: str
        :param app_name: the apk file name

        :rtype: tuple (str, str)
        :return: package name and output log
        """
        package_name = ""
        return_msg = "Failed to get app Package Name"

        if not app_name.endswith(".apk"):
            app_name = "%s.apk" % app_name

        cmd = "adb shell pm list packages -f"
        status, status_msg = self._run_cmd_and_check_failure(cmd, timeout=self._uecmd_default_timeout, silent_mode=True)
        if status == Util.Global.SUCCESS:
            for package in status_msg.split('\n'):
                expr_name = "package:(?P<app_location>.*)=(?P<package_name>.*)"
                matches_str = re.compile(expr_name).search(package)
                if matches_str is not None and app_name in matches_str.group("app_location").split('/'):
                    package_name = matches_str.group("package_name")
                    return_msg = "Success to get app Package Name={0}".format(package_name)
                    self._logger.info(return_msg)
                    break
        else:
            package_name = ""
            return_msg = "Impossible to retrieve app Package Name (pm list packages -f)"

        return package_name, return_msg

    def clear_cache_data_of_device_app(self, package_name):
        """
        Clear package cache and data of an android application

        :type  package_name: str
        :param package_name: package name of the device application

        :rtype: tuple (bool, str)
        :return: Output status and output log
        """
        status = Util.Global.FAILURE
        status_msg = "Undefined error"

        app_location = self.get_path_of_device_app(package_name)
        if app_location == "":
            status_msg = "Invalid package name, no app on the device has this package name"
        else:
            cmd = "adb shell pm clear {0}".format(package_name)
            status, status_msg = self._run_cmd_and_check_failure(cmd, timeout=self._uecmd_default_timeout, silent_mode=True)

        return status, status_msg

    def sign_device_app(self, app_path, sign_path, sign_name, out_path, timeout):
        """
        Sign a device application

        :type app_path: str
        :param app_path: file to sign

        :type sign_path: str
        :param sign_path: folder containing signing keys

        :type sign_name: str
        :param sign_name: name of the signature (eg. filename, without extension)

        :type out_path: str
        :param out_path: signed file (must be different from app_path)

        :type  timeout: int
        :param timeout: operation timeout

        :rtype: list
        :return: Output status and output log
        """
        status = Util.Global.FAILURE
        status_msg = "Undefined error while signing application"
        app_path = os.path.abspath(app_path)
        out_path = os.path.abspath(out_path)
        sign_path = os.path.abspath(sign_path)

        if os.path.isfile(app_path):
            self._logger.info("Signing {0} with '{2}' key from {1} ...".format(app_path, sign_path, sign_name))
            cmd = 'java -jar signapk.jar {1}/{2}.x509.pem {1}/{2}.pk8 {0} {3}'\
                .format(app_path, sign_path, sign_name, out_path)
            status, status_msg = self._device.run_cmd(cmd, timeout)
            if status == Util.Global.FAILURE:
                status_msg = "An error occurred while signing {0} : {1}".format(app_path, status_msg)
        else:
            status_msg = "Impossible to sign {0}, file does not exist".format(app_path)

        return status, status_msg

    def uninstall_device_app(self, app_name, timeout=0, backup_file=None, forced=False):
        """
        Uninstall a device application

        :type app_name: str
        :param app_name: reference to the application to remove

        :type  timeout: int
        :param timeout: operation timeout

        :type  backup_file: str
        :param backup_file: if set, previous app will be backup to the specified file

        :type  forced: bool
        :param forced: if true, app uninstall will be forced, even if it is a protected or system app

        :rtype: tuple (bool, str)
        :return: Output status and output log
        """
        # Uninstall the app first
        status = Util.Global.FAILURE
        app_location = self.get_path_of_device_app(app_name)
        if not app_location:
            status = Util.Global.SUCCESS
            status_msg = "Application is not installed on the device"
        else:
            if backup_file:
                self.backup_app(backup_file, app_location, timeout)

            if forced:
                # list of all files declared for this package
                cmd = "adb shell pm dump {0} | grep -e Dir= -e Path= | tr -d '\\r'".format(app_name)
                status, status_msg = self._run_cmd_and_check_failure(cmd, timeout)
                if status == Util.Global.SUCCESS:
                    file_list = status_msg.split('\n')
                    if file_list:
                        # remount all partitions
                        cmd = "adb remount"
                        status, status_msg = self._run_cmd_and_check_failure(cmd, timeout)
                        # delete all the files declared for the package
                        for file_to_remove in file_list:
                            cmd = "adb shell rm -rf {0}".format(file_to_remove.split("=")[1])
                            status, status_msg = self._run_cmd_and_check_failure(cmd, timeout)
                            if status == Util.Global.FAILURE:
                                break
                        # reboot phone once all files are deleted
                        self._device.reboot()
                    else:
                        self._logger.info("Nothing to delete for app {0}".format(app_name))
            else:
                cmd = "adb uninstall {0}".format(app_name)
                status, status_msg = self._run_cmd_and_check_failure(cmd, timeout)

        return status, status_msg

    def launch_app(self, intent=None, action=None, flags=None, extras=None, data_uri=None, data_type=None):
        """
        Tell ActivityManager to start an application.

        :type intent: str
        :param intent: Component/.intent to send; value goes with the -n option

        :type action: str
        :param action: Intent action; value goes with the -a option

        :type flags: str
        :param flags: Flags; value goes with the -f option.

        :type extras: str
        :param extras: Extras.  May contain one or more -e* options along with data.

        :type data_uri: str
        :param data_uri: Intent data URI; value goes with the -d option.

        :type data_type: str
        :param data_type: Intent data MIME type; value goes with the -t option.

        :raise AcsConfigException: If no intent or action is specified
        :raise DeviceException: If 'am start' command fails.
        :rtype: None
        """
        if not intent and not action:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "uecmd launch_app: No intent or action was given")

        cmd_line = "adb shell am start"
        if intent:
            cmd_line = cmd_line + " -n " + intent
        if action:
            cmd_line = cmd_line + " -a " + action
        if flags:
            cmd_line = cmd_line + " -f " + flags
        if data_uri:
            cmd_line = cmd_line + " -d " + data_uri
        if data_type:
            cmd_line = cmd_line + " -t " + data_type
        if extras:
            cmd_line = cmd_line + " " + extras

        (return_code, output_msg) = self._run_cmd_and_check_failure(cmd_line, 20)
        if return_code != Util.Global.SUCCESS:
            msg = "uecmd launch_app: Error '%s' when running '%s'" % (output_msg, cmd_line)
            self._logger.error(msg)
            raise DeviceException(DeviceException.ADB_ERROR, msg)

    def stop_device_app(self, app_name, timeout=0):
        """
        Stop a device application

        :type app_name: str
        :param app_name: reference to the application to remove (package name)

        :type  timeout: int
        :param timeout: operation timeout

        :rtype: tuple (bool, str)
        :return: Output status and output log
        """
        if timeout == 0:
            timeout = self._uecmd_default_timeout

        cmd = "adb shell am force-stop %s" % app_name
        status, status_msg = self._run_cmd_and_check_failure(cmd, timeout)
        return status, status_msg

    def add_element_in_shared_prefs(self, package_name, prefs_filename, element):
        """
        Add an element in application shared preferences
        """
        path_on_device = "/data/data/" + package_name + "/shared_prefs/" + prefs_filename
        timeout = 10

        # retrieve package path on device
        result, message = self._device.run_cmd("adb shell pm path %s" % package_name, timeout)
        if result == Util.Global.SUCCESS:
            regex_package_path = ".*package:.*"
            matches_path = re.compile(regex_package_path).search(message)
            if matches_path:
                # start package thus it can generate shared prefs file
                self._device.run_cmd("adb shell am start %s" % package_name, timeout)
                time.sleep(5)

                # retrieve shared preferences from device
                tmpdest = os.path.abspath(os.path.join(tempfile.gettempdir(), prefs_filename))
                result, message = self._device.pull(path_on_device, tmpdest, timeout)

                if result == Util.Global.SUCCESS:
                    # add desired value in shared prefs
                    is_modified = modify_text_file(tmpdest, add_line_before_pattern, '</map>', element)
                    if is_modified:
                        # push the modified shared prefs on the device
                        result, message = self._device.push(tmpdest, path_on_device, timeout)

                        if result == Util.Global.SUCCESS:
                            # go home before calling force-stop, to avoid
                            # system crashes with some apps (such as Chrome M36)
                            self._device.run_cmd("adb shell input keyevent KEYCODE_HOME", timeout)

                            # stop app process, thus it will re-load shared prefs at next start
                            result, message = self._device.run_cmd("adb shell am force-stop %s" % package_name, timeout)
                            if result != Util.Global.SUCCESS:
                                raise DeviceException("Unable to stop %s application. "
                                                      "Error: %s" % (package_name, message))
                            else:
                                adb_force_stop_timeout = self._device.get_config("AdbAmForceStopTimeout", 2, int)
                                time.sleep(adb_force_stop_timeout)
                        else:
                            raise DeviceException("Unable to push %s to %s on device. "
                                                  "Error: %s" % (tmpdest, path_on_device, message))
                    else:
                        raise AcsToolException("Unable to modify shared prefs for %s" % package_name)
                else:
                    raise DeviceException("Unable to pull %s from device. Error: %s" % (path_on_device, message))
            else:
                # app is not installed => failure
                raise AcsToolException("Unable to modify shared prefs for {0}: "
                                       "app is not installed on device!".format(package_name))
        else:
            raise DeviceException("Unable to search path for %s application. Error: %s" % (package_name, message))

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

    def app_enable_disable(self, package_name, enable):
        """
        Enable/disable package

        :type package_name: str
        :param package_name: name of the package to enable/disable

        :type enable: boolean
        :param enable: True = enable package, False = disable package
        """
        if not package_name:
            self._logger.warning("app_enable_disable: No package name specified!")
            return

        tag_app_state = self.get_app_status(package_name)

        if tag_app_state == "not available":
            raise DeviceException(DeviceException.INVALID_PARAMETER, "Unknown package %s" % package_name)
        elif enable and tag_app_state == Util.TestConst.STR_DISABLE:
            self._logger.info("Enable " + package_name + " package")
            self._exec("adb shell pm enable " + package_name)
        elif not enable and tag_app_state == Util.TestConst.STR_ENABLE:
            self._logger.info("Disable " + package_name + " package")
            self._exec("adb shell pm disable " + package_name)
        elif enable and tag_app_state == Util.TestConst.STR_ENABLE:
            self._logger.info(package_name + " package is already enabled")
        elif not enable and tag_app_state == Util.TestConst.STR_DISABLE:
            self._logger.info(package_name + " package is already disabled")

    def get_app_status(self, package_name):
        """
        Get package status

        :type package_name: str
        :param package_name: name of the package

        :rtype: str
        :return: "enabled", "disabled", "not available"
        """
        output = self._exec("adb shell pm list packages " + package_name)

        if package_name not in output:
            self._logger.info(package_name + " package is not available")
            package_status = "not available"
        else:
            output = self._exec("adb shell pm list packages -e " + package_name)

            if '' in ["".join(x.split("package:" + package_name)) for x in output.splitlines()]:
                self._logger.info(package_name + " package is enabled")
                package_status = Util.TestConst.STR_ENABLE
            else:
                self._logger.info(package_name + " package is disabled")
                package_status = Util.TestConst.STR_DISABLE

        return package_status

