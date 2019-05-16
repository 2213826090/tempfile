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

:organization: INTEL NDG SW
:summary: This file implements the System UEcmd for Linux device
:since: 12/09/2014
:author: dpierrex
"""

import os
import re
import time
import posixpath
import hashlib
from datetime import datetime
from acs_test_scripts.Device.UECmd.Imp.Linux.Common.Base import Base
from acs_test_scripts.Device.UECmd.Interface.System.ISystem import ISystem
import UtilitiesFWK.Utilities as Util
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException


class System(Base, ISystem):

    """
    :summary: System UEcommands operations for Linux platforms using an ssh based communication to the I{DUT}.
    """

    def __init__(self, device):
        """
        Constructor.
        """

        Base.__init__(self, device)
        ISystem.__init__(self, device)

    def set_date_and_time(self, date_and_time):
        """
        Set date and time on the device

        :type date_and_time: datetime.datetime
        :param date_and_time: Date and time in python datetime structure

        :rtype: list
        :return: operation status & output log.
                 Returns date and time in following format in case of success: "%d/%m %H:%M:%S"
        """
        # get the date with the Busybox format
        given_date_and_time = date_and_time.strftime("%Y-%m-%d %H:%M:%S")
        set_date_cmd = "timedatectl set-time '{0}'".format(given_date_and_time)
        return_code, return_msg = self._internal_exec(set_date_cmd, self._uecmd_default_timeout)
        if return_code == Util.Global.SUCCESS:
            return_code, device_date_and_time = self._internal_exec("date +%Y-%m-%d-%H:%M:%S")

            given_ts = datetime.strptime(given_date_and_time, "%Y-%m-%d %H:%M:%S")
            device_ts = datetime.strptime(device_date_and_time, "%Y-%m-%d-%H:%M:%S")

            # Theoretically, device_ts and given_ts are equals
            # Practically, they may have a gap of several seconds (usually 2s)
            # If the gap is upper than 10 seconds, there might be
            # a mistake.
            time_delta = abs(device_ts - given_ts)
            # Workaround to compute total seconds in python2.6
            # https://bitbucket.org/wnielson/django-chronograph/issue/27/python26-datatimetimedelta-total_seconds
            if hasattr(time_delta, "total_seconds"):
                duration = time_delta.total_seconds()
            else:
                duration = (time_delta.microseconds + (time_delta.seconds + time_delta.days * 24 * 3600) * 10 ** 6) / 10 ** 6

            if duration > 10:
                return_code = Util.Global.FAILURE
                return_msg = "Date and Time seems to be bad (times are too different): " \
                             "requested '{0}', applied '{1}'".format(given_date_and_time, device_date_and_time)
                self._logger.warning(return_msg)

            else:
                return_code = Util.Global.SUCCESS
                return_msg = datetime.strftime(device_ts, "%d/%m %H:%M:%S")
        return return_code, return_msg

    def get_date_and_time(self):
        """
        Get date and time on the device

        :rtype: list
        :return: operation status & date.
                 Returns date and time in following format in case of success: "%Y-%m-%d %H:%M:%S"
        """
        return self._internal_exec("date '+%Y-%m-%d %H:%M:%S'")

    def set_timezone(self, timezone):
        """
        Set device timezone.

        :type  timezone: str
        :param timezone: Timezone to set on the device

        :rtype: list
        :return: operation status & output log. Returns the timezone in case of success.
        """
        set_timezone_property_cmd = "timedatectl set-timezone %s" % timezone
        return_code, return_msg = self._internal_exec(set_timezone_property_cmd, self._uecmd_default_timeout)

        if return_code != Util.Global.SUCCESS:
            return_code = Util.Global.FAILURE
            return_msg = "Cannot set timezone to {0} ! ({1})".format(timezone, return_msg)

        return return_code, return_msg

    def clear_cache(self):
        """
        Clear the buffer and cache prior
        """
        self._internal_exec("sync && echo 3 > /proc/sys/vm/drop_caches", 30)

    def pid_of(self, process_name):
        """
        Get the pid of a process by its name.

        :rtype: str
        :return: the pid of the process or empty str if process as not been found.
        """
        cmd = "pidof " + str(process_name)
        cmd = self._internal_exec(cmd)

        self._logger.info("Getting the pid of the process with name: %s" % process_name)
        (_result, pid) = Util.internal_shell_exec(cmd, 10)
        if not pid.isdigit() and not pid == "":
            raise DeviceException(DeviceException.INTERNAL_EXEC_ERROR,
                                  "The pid should be a int. Is: %s" % str(pid))
        self._logger.debug("PID of %s is: %s" % (process_name, pid))
        return pid

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
        :param wait_for_response: Wait response from ssh before stating on command

        :rtype: tuple (int, str)
        :return: Output status and output log
        """
        return_code, return_msg = self._device.run_cmd(cmd, timeout, force_execution, wait_for_response, silent_mode)

        if "Failure" in return_msg or "No such file" in return_msg or "not found" in return_msg:
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
                # Initialize timeout if not set
                if not timeout:
                    timeout = Util.compute_timeout_from_file_size(bin_path, self._device.min_file_install_timeout)

                # Build chmod commands
                dest_path = posixpath.join(destination, binary_filename)
                bin_chmod_cmd = "chmod +x {0}".format(dest_path)
                # Push the binary file
                self._logger.info("Pushing %s -> %s ..." % (str(bin_path), str(dest_path)))
                # status, status_msg = self._run_cmd_and_check_failure(bin_push_cmd, timeout)
                status, status_msg = self._device.push(bin_path, destination, timeout)

                if status == Util.Global.SUCCESS:
                    # Modify access right on the binary file
                    status, status_msg = self._run_cmd_and_check_failure(bin_chmod_cmd, timeout)
                    if status == Util.Global.SUCCESS:
                        status_msg = "Binary has been successfully installed on the device!"
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
        cmd = "find {0} -maxdepth 1 -type f -regex '.*/{1}'".format(device_directory, filename_regex)
        status, output = self._device.run_cmd(cmd, self._uecmd_default_timeout, wait_for_response=True, silent_mode=True)

        if status != Util.Global.SUCCESS:
            status_msg += " ({0})".format(output)
        elif output:
            cmd = "rm {0}".format(" ".join(output.split('\n')))
            status, msg = self._run_cmd_and_check_failure(cmd, self._uecmd_default_timeout, wait_for_response=True)
            if status == Util.Global.SUCCESS:
                status_msg = "Files : {0} have been successfully " \
                             "removed from device".format(" ".join(output.split('\n')))
        else:
            status = Util.Global.SUCCESS
            status_msg = "Nothing to remove, no file matching {0} in {1} !".format(filename_regex, device_directory)

        return status, status_msg

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

        #Make sure we have permission to execute.
        #We'll also rely on this to check existence of the file rather than doing a separate check.
        (return_code, output_msg) = file_api.set_file_permission(exe, "rwxr-xr-x")
        if not return_code:
            msg = "uecmd run_shell_executable: Error '%s' when trying to set permissions on '%s'"%(output_msg, exe)
            self._logger.error(msg)
            raise DeviceException(DeviceException.FILE_SYSTEM_ERROR, msg)

        #See if user wants to send stdout and stderr somewhere else.
        if io_redirection == 1:
            io_suffix = "&>/dev/null"
        elif io_redirection == 2:
            io_suffix = "1>stdout.log 2>stderr.log"

        #If a path is given to the file, make that the working directory before we invoke the executable.
        if path != "":
            cmd_line = "cd {0}; sh ./{1} {2} {3}".format(path, filename, args, io_suffix)
        else:
            cmd_line = "sh ./{0} {1} {2}".format(filename, args, io_suffix)

        (return_code, output_msg) = self._run_cmd_and_check_failure(cmd_line, timeout)
        if return_code != Util.Global.SUCCESS:
            msg = "uecmd run_shell_executable: Error '%s' when running '%s'"%(output_msg, cmd_line)
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
                     %(str(source_file_path), str(destination_file_path))

        # Check source file path + destination file path
        if os.path.isfile(source_file_path):
            if destination_file_path is not None:
                # md5sum /home/root/acs_files/1MB.zip
                # 6db371d446c028e4f984254b8178ec52 /home/root/acs_files/1MB.zip
                # checksum file_name
                return_result, return_message = self._device.run_cmd(cmd="md5sum %s" % (destination_file_path),
                                                    timeout=self._uecmd_default_timeout, silent_mode=True,
                                                    force_execution=True)
                if return_result != Util.Global.FAILURE and "No such file" not in return_message:
                    # Retrieve the checksum of the file installed on the device
                    dest_file_md5_checksum, _ = return_message.split()

                    # Compare them to the file to install
                    orig_file_checksum = hashlib.md5(open(source_file_path, 'rb').read()).hexdigest()

                    if orig_file_checksum == str(dest_file_md5_checksum):
                        status = Util.Global.SUCCESS
                        status_msg = "File already exist at : %s" % (str(destination_file_path))
                else:
                    status_msg = "File is not accessible: %s" %(str(destination_file_path))
            else:
                status_msg = "No destination file path given: %s" %(str(destination_file_path))
        else:
            status_msg = "Source file to check does not exist: %s" % (str(source_file_path))

        return status, status_msg
