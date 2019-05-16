# @PydevCodeAnalysisIgnore
# flake8: noqa
"""
@copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
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

@organization: INTEL MCG PSI
@summary: This file exposes the crash info library
"""

# Current crashinfo lib version : used to check compatibility with the crashinfo api running on device.
__version__ = "2.1"

from exceptions import Exception
from optparse import OptionParser
import os
import subprocess
import sys
import tempfile
import time
import json


def write_file(file_path, content):
    """
    Writes the given C{content} to a C{file} at the given C{file_path}.
    Returns C{True} in case of success.

    @type file_path: str
    @param file_path: the file path

    @type content: str
    @param content: the file's content

    @rtype: bool
    @return: the operation status:
        - C{True} if the operation succeeded
        - C{False} otherwise
    """
    # Initialize the status
    status = False
    # Create a context manager for IO operations so
    # that errors are properly handled.
    with open(file_path, 'w') as output_file:
        # Perform the requested IO
        output_file.write(content)
        # Update the status
        status = True
    # Return the status
    return status


class CrashInfoException(Exception):
    pass


class CrashInfo(object):

    """
    CrashInfo allows to pull crashes events from a given device and to upload
    them to Crashtool server.
    """

    def __init__(self, silent_mode=False, serial_number=None, prog_name="CrashInfo", logger=None):
        self._logger = logger
        self.__program_name = prog_name
        self.__silent_mode = silent_mode
        self.__device_serial_number = None
        self.__device_api_version = -1
        self.set_serial_number(serial_number)
        self.__device_api_version = self.__get_device_api_version()
        if self.__device_api_version == -1:
            raise CrashInfoException("Cannot retrieve crashinfo device version.")
        self.__init_proxy()

    def __init_proxy(self):
        """"
        Associate CrashInfo public functions with the right private method depending on
        versions of crashinfo daemon running on device
        """
        Api_CrashInfo_Version = {-1: 'version_0', 0: 'version_0', 1: 'version_1'}
        Get_Device_Info_Fct = {'version_0': self.__get_device_info_V0, 'version_1': self.__get_device_info_V1}
        Get_Event_Fct = {'version_0': self.__get_event_V0, 'version_1': self.__get_event_V1}

        self.get_device_info = Get_Device_Info_Fct[Api_CrashInfo_Version[self.__device_api_version]]
        self.get_event = Get_Event_Fct[Api_CrashInfo_Version[self.__device_api_version]]

    def __print(self, message):
        if not self.__silent_mode:
            if self._logger:
                self._logger.debug(("%s : %s") % (self.__program_name, message))
            else:
                print(("%s : %s") % (self.__program_name, message))

    def __internal_shell_exec(self, cmd, timeout):
        """
        Execute the input command regardless the Host OS
        and return the result message

        If the timeout is reached, return an exception

        @type  cmd: string
        @param cmd: cmd to be run
        @type  timeout: integer
        @param timeout: Script execution timeout in sec

        @return: Execution status & output string
        @rtype: Integer & String
        """
        # Revert to use TemporaryFile due to Windows 8 limitation
        # enhancement to be done without it
        f_stdout = tempfile.TemporaryFile()

        try:
            # Force str type in order to avoid bad string encoding nt/posix
            cmd = str(cmd)

            if self.__device_serial_number:
                cmd = cmd.replace("adb ", "adb -s %s " % self.__device_serial_number, 1)

            # If Linux split cmd
            if os.name in ['posix']:
                import shlex
                cmd = shlex.split(cmd)

            my_process = subprocess.Popen(
                cmd,
                shell=False,
                stdout=f_stdout,
                stderr=subprocess.STDOUT,
                close_fds=False)
            # Check if the child process has terminated or timeout reached.
            while (timeout > 0) and (my_process.poll() is None):
                time.sleep(1)
                timeout -= 1

            poll_value = my_process.poll()
            if poll_value is not None:
                # Process created by Popen is terminated by system

                # Read the output
                # Using communicate() rather than .stdin.write,
                # .stdout.read or .stderr.read
                # to avoid deadlocks due to any of the other OS pipe buffers
                # filling up and blocking the child process.
                (stdoutdata, stderrdata) = my_process.communicate()

                f_stdout.seek(0)
                stdoutdata = f_stdout.read()

                if poll_value == 0:
                    # Process successfully terminated
                    result = str(stdoutdata).rstrip("\r\n")
                    return (0, result)
                else:
                    # Note: A negative value -N indicates that the child
                    # was terminated by signal N (Unix only).
                    self.__print("Cmd subprocess killed by system ("
                                 + str(poll_value) + ")")
                    self.__print("*** COMMAND FAILED!\r")

                    # Process not terminated properly
                    self.__print("Command %s failed" % (str(cmd)))
                    self.__print("*** OUTPUT: %s\n" % str(stdoutdata))
                    self.__print("*** STDERR: %s\n" % str(stderrdata))
                    return (poll_value, str(stdoutdata))
            else:
                # Process was not terminated until timeout
                # Close the process
                if my_process is not None:
                    self.__print("Force terminating Cmd subprocess ...")
                    my_process.terminate()

                    if my_process.wait() is not None:
                        self.__print(
                            "Cmd subprocess successfully terminated !")
                    else:
                        self.__print(
                            "Cmd subprocess still not terminated !")

                else:
                    self.__print(
                        "Cmd subprocess not started or already terminated")

                self.__print("*** TIMEOUT!\r")

                # Process hasn't finished to execute command.
                err_msg = "Command %s has timeout!" % (str(cmd))
                self.__print(err_msg)
                return (-1, err_msg)
        finally:
            f_stdout.close()

    def __filter_crashinfo_result(self, coded_result):
        split_str = coded_result.split('\n')
        lst_result = []
        str_crashinfo_tag = "<crashinfo>"
        # remove crashinfo tag
        if len(split_str) > 0:
            for row in split_str:
                if row.find(str_crashinfo_tag) == 0:
                    lst_result.append(row.replace(str_crashinfo_tag, "", 1))
        return lst_result

    def __get_device_api_version(self):
        """
        Gets version of running crashinfo api on device
        @rtype: integer
        @return: an integer containing the version. -1 if failed.
        """
        if self.__device_api_version != -1:
            return self.__device_api_version
        else:
            version = -1
            if self.__device_serial_number in (None, "None", ""):
                self.__print("Serial number not set : can't retrieve CrashInfo API version running on device")
                return version
            infos = self.get_status()
            self._logger.debug("Device status: %s " % str(infos))
            if len(infos) != 0:
                if 'api_version' in infos:
                    version = int(infos['api_version'])
                    if version > 1:
                        self.__print(
                            "Try to run new API version {0} on old one waiting for crash uploader implementation.".format(version))
                        version = 1
                else:
                    version = 0
            return version

    def __get_device_info_V0(self):
        """
        Version 0 : returns an empty list since the version of crashinfo
        daemon running on target device doesn't support this feature.
        @rtype: list
        @return: Empty list.
        """
        device_info = []
        return device_info

    def __get_device_info_V1(self):
        """
        Versions 1 : Returns device info (spid, GCM...)
        @return: the unicode strings list of device info and their value. Empty
        list if failed.
        """
        device_info = []
        cmd_line = "adb shell crashinfo getdevice --json"
        # Execute the command
        try:
            result, output = self.__internal_shell_exec(cmd_line, 30)
        except:
            print "exception while executing crashinfo - command not available"
        else:
            if result == 0:
                try:
                    device_info = self.__convert_json_dict(json.loads(output))
                except Exception as e:
                    self.__print(("Exception while getting device info: %s") % e)
                    self.__print(("Wrong input : %s") % output)
            else:
                self.__print("Issue while getting device info")
                self.__print(output)
        return device_info

    def __convert_json_dict(self, input):
        """
        Convert all unicode elements of a dictionary loaded from JSON.
        This aims to print to the stdout a 'json' dictionary without 'u' char
        before each string.
        No conversion is performed if input data is not a dictionary.
        @rtype: dictionary
        @return: the input dictionary converted.
        """
        if isinstance(input, dict):
            rv = {}
            for key in input:
                value = str(input[key])
                key = str(key)
                rv[key] = value
            return rv
        else:
            return input

    def __get_event_V0(self, detail_level=0, filter_ge="", filter_parameter=None, use_filter_tag=False):
        events = []
        cmd_line = (self.__get_event_build_cmdline(detail_level, filter_ge, filter_parameter, use_filter_tag))
        # Execute the command
        try:
            result, output = self.__internal_shell_exec(cmd_line, 30)
        except:
            print "exception while executing crashinfo - command not available"
        else:
            if result == 0:
                # Parse result from output (method changed if filter tag is used)
                if use_filter_tag:
                    split_str = self.__filter_crashinfo_result(output)
                else:
                    split_str = output.split('\n')
                if self.__check_command_output(split_str):

                    if len(split_str) > 1:
                        header = split_str[0].split(' | ')

                        content = split_str[1: len(split_str)]
                        # print header
                        if len(split_str) > 0:
                            for row in content:
                                event = {}
                                for index in range(len(header)):
                                    split_row = row.split(' | ')
                                    value = split_row[index].strip()
                                    event[header[index].strip()] = value
                                # adding ssn for crashtool need to check it is not none
                                if self.__device_serial_number:
                                    event["ssn"] = self.__device_serial_number
                                events.append(event)
                    else:
                        self.__print("no element found")
                else:
                    if use_filter_tag:
                        self.__print("wrong output")
                    else:
                        self.__print("wrong output - now trying filter tag")
                        return self.get_event(detail_level, filter_ge, filter_parameter, True)
            else:
                self.__print("Issue while getting event:")
                self.__print(output)
        return events

    def __get_event_V1(self, detail_level=0, filter_ge="", filter_parameter=None, use_filter_tag=False):
        """
        Version 1 uses JSON option of crashinfo getevent command
        """
        events = []
        cmd_line = (self.__get_event_build_cmdline(detail_level, filter_ge, filter_parameter, use_filter_tag))
        cmd_line = "%s %s" % (cmd_line, "--json")
        print cmd_line
        # Execute the command
        try:
            result, output = self.__internal_shell_exec(cmd_line, 30)
        except:
            print "exception while executing crashinfo - command not available"
        else:
            if result == 0:
                # Split output into a list of strings and 'json' load each row
                split_str = output.split('\n')
                for row in split_str:
                    event = {}
                    try:
                        event = self.__convert_json_dict(json.loads(row))
                    except Exception as e:
                        # A row could not be loaded : stop processing and return an empty list
                        self.__print(("Exception while getting an event : %s") % e)
                        self.__print(("Wrong input : %s") % row)
                        events[:] = []
                        break
                    if self.__device_serial_number:
                        event["ssn"] = self.__device_serial_number
                    events.append(event)
        return events

    def __get_event_build_cmdline(self, detail_level, filter_ge, filter_parameter, use_filter_tag):
        """
        Common part of get_event : command line building
        """
        cmd_line = "adb shell crashinfo getevent"

        if (detail_level == 1):
            # Detail events
            cmd_line = "%s %s" % (cmd_line, "--detail")
        elif (detail_level == 2):
            # Full events
            cmd_line = "%s %s" % (cmd_line, "--full")
        # Get the filter type
        if (filter_ge == "name"):
            cmd_line = "%s %s" % (cmd_line, "--filter-name")
        elif (filter_ge == "type"):
            cmd_line = "%s %s" % (cmd_line, "--filter-type")
        elif (filter_ge == "id"):
            cmd_line = "%s %s" % (cmd_line, "--filter-id")
        elif (filter_ge == "time"):
            cmd_line = "%s %s" % (cmd_line, "--filter-time")
        elif (filter_ge == "last"):
            cmd_line = "%s %s" % (cmd_line, "--last")
        elif (filter_ge == "uploaded"):
            cmd_line = "%s %s" % (cmd_line, "--uploaded")
        # Get the filter value
        if filter_parameter is not None:
            cmd_line = "%s %s" % (cmd_line, filter_parameter)
        # Get header option (this option isn't taken into account when json is used)
        if use_filter_tag:
            cmd_line = "%s %s" % (cmd_line, "-a")
        return cmd_line

    def __check_command_output(self, output):
        """
        Check if the output of 'get_event' command matches the type of the command
        @rtype: boolean
        @return: True if the output matches the given command type
        """
        split_str = output
        if len(split_str) < 1:
            return False
        else:
            check_split = split_str[0].split(' | ')
            i_len_ref = len(check_split)
            if i_len_ref > 1:
                for line in split_str:
                    i_test_len = len(line.split(' | '))
                    if i_len_ref != i_test_len:
                        return False
                return True
            else:
                return False

    def __update_upload_state(self, row_id=-1, event={}, result_upload=-1):
        """
        Update the event state in the CrashReport DB after an upload.
        The event attributes 'uploaded'  and 'logsuploaded' are updated in
        CrashReport DB depending on the input result_upload value as following :
        upload result = -1                                          => no DB update
        upload result = 0  AND log is invalid                       => Event 'Uploaded' & Log 'Invalid'
        upload result = 0  AND event has no log                     => Event 'Uploaded' & Log 'NotUploaded'
        upload result = 0  AND log is valid AND dataReady = 0       => Event 'Uploaded' & Log 'NotUploaded'
        upload result = 0  AND log is valid AND dataReady = 1       => Event 'Uploaded' & Log 'Uploaded'
        upload result = 1 (upload failed : failure)                 => no DB update
        upload result = 2 (upload failed : invalid event)           => Event 'Invalid' & Log 'Invalid'
        upload result = 3 (event upload OK but log event upload KO) => Event 'Uploaded' & Log 'NotUplaoded'
        @rtype: void
        @return: void
        """
        # Check first if event has a crashdir associated
        if event["crashdir"] != "":
            # A log is associated to
            # Upload status treatment
            if (result_upload == 0):
                # Manage particular case where event to upload has an invalid log.
                # Event shall be set as uploaded (whatever its dataready value is )
                # but its log shall remains as invalid
                if event['logsuploaded'] == '-1':
                    self.update_upload_state(row_id, 'logInvalid')
                    self.__print("event upload OK & invalid log not uploaded")
                    return
                try:
                    ready = event["dataReady"]
                except:
                    ready = "1"
                if ready == "0":
                    # Upload Event success & no Log => set Event 'Uploaded' & leave Log 'NotUploaded' in DB
                    self.update_upload_state(row_id, 'logNotUploaded')
                    self.__print("event upload OK - data not ready")
                else:
                    # Upload Event & Log success => set Event 'Uploaded' & set Log 'Uploaded' in DB
                    self.update_upload_state(row_id, 'logUploaded')
                    self.__print("event upload OK & log upload OK")
            elif (result_upload == 1):
                # Event upload failed => leave Event 'NotUploaded' & leave Log 'NotUploaded' in DB
                self.__print("event and log upload failed")
            elif (result_upload == 2):
                # Event upload success but Event is Invalid => set Event 'Invalid' & Log 'Invalid' in DB
                self.update_upload_state(row_id, 'eventInvalid')
                self.__print("uploaded event is Invalid")
            elif (result_upload == 3):
                # Event upload success but Log upload failed => set Event 'Uploaded' & leave Log 'NotUploaded' in DB
                self.update_upload_state(row_id, 'logNotUploaded')
                self.__print("event upload OK & log upload failed")
            else:
                self.__print("Unexpected error during event & log upload")
        else:
            # No crashlog file
            # Upload Event success => set Event 'Uploaded' & Log 'Uploaded' in DB
            if (result_upload == 0):
                self.update_upload_state(row_id, 'logNotUploaded')
                self.__print("event upload OK")
            # Upload Event fails => leave Event 'NotUploaded' & Log 'NotUploaded' in DB
            elif (result_upload == 1):
                self.__print("event upload failed")
            # Uploaded Event Invalid => set Event 'Invalid' & Log 'Invalid' in DB
            elif (result_upload == 2):
                self.update_upload_state(row_id, 'eventInvalid')
                self.__print("uploaded event is Invalid")
            # Upload Event success but Log upload failed => shall not happens in this case (robustness)
            # set Event 'Uploaded' & leave Log 'NotUploaded' in DB
            elif (result_upload == 3):
                self.update_upload_state(row_id, 'logNotUploaded')
                self.__print("event upload OK but Log upload failed or uploaded log is Invalid")
            # Unexpected error
            else:
                self.__print("Unexpected error during event upload")

#----------------------------------------------------------------------------------------
#------------------------------ CrashInfo Public Functions ------------------------------
#----------------------------------------------------------------------------------------

    def get_device_info(self):
        """
        Returns device info (spid, GCM...).
        Version 0 : returns an empty list
        Version 1 : returns a well filled list
        @rtype: list
        @return: the unicode strings list of device info and their value. Empty
        list if failed or if the version of crashinfo daemon running on target
        device doesn't support this feature.
        """
        pass

    def get_event(self, detail_level=0, filter_ge="", filter_parameter=None, use_filter_tag=False):
        """
        Returns the content of CrashReport 'Event' database under dictionary format
        @rtype: list
        @return: the unicode strings dictionary of device info and their value. Empty
        list if failed or if the version of crashinfo daemon running on target
        device doesn't support this feature.
        """
        pass

    def get_build_id(self, spec=False, use_filter_tag=False):
        cmd_line = "adb shell crashinfo buildid"
        if use_filter_tag:
            cmd_line = "%s %s" % (cmd_line, "-a")
        build_info = {}

        if (spec == 1):
            # Detail events
            cmd_line = "%s %s" % (cmd_line, "--spec")
        # Execute the command
        try:
            result, output = self.__internal_shell_exec(cmd_line, 30)
        except:
            print "exception while executing crashinfo - command not available"
        else:
            if result == 0:
                # Parse result from output
                if (spec == 1):
                    split_str = output.split('\n')
                    split_str = split_str[1:11]
                else:
                    if use_filter_tag:
                        tmpresult = self.__filter_crashinfo_result(output)
                        if len(tmpresult) > 0:
                            output = tmpresult[0]
                        else:
                            self.__print("Output error, no line found with filter tag")
                            return build_info
                    split_str = output.split('\n')
                    if len(split_str) > 1:
                        if use_filter_tag:
                            self.__print("Output error, wrong number of build items with filter tag")
                            return build_info
                        else:
                            self.__print("Output error, now trying with filter tag")
                            return self.get_build_id(spec, True)
                    split_str = output.split(',')

                if (len(split_str) == 10):
                    build_info["BuildId"] = split_str[0].strip()
                    build_info["Fingerprint"] = split_str[1].strip()
                    build_info["KernelVersion"] = split_str[2].strip()
                    build_info["BuildUserHostname"] = split_str[3].strip()
                    build_info["ModemVersion"] = split_str[4].strip()
                    build_info["IfwiVersion"] = split_str[5].strip()
                    build_info["IafwVersion"] = split_str[6].strip()
                    build_info["ScufwVersion"] = split_str[7].strip()
                    build_info["PunitVersion"] = split_str[8].strip()
                    build_info["ValhooksVersion"] = split_str[9].strip()
                else:
                    self.__print("Output error, wrong number of build items")
                    self.__print(output)
            else:
                self.__print("Issue while getting build info:")
                self.__print(output)

        return build_info

    def clean(self, filter_ge="", filter_parameter=None):
        cmd_line = "adb shell crashinfo clean"

        if filter_ge == "force-all":
            # cmd_line = "%s %s" % (cmd_line, "force all option")
            print "not implemented"
            return

        elif (filter_ge == "id") and (filter_parameter is not None):
            cmd_line = "%s %s %s" % (cmd_line,
                                     "--filter-id", filter_parameter)

        elif (filter_ge == "time") and (filter_parameter is not None):
            cmd_line = "%s %s %s" % (cmd_line,
                                     "--filter-time", filter_parameter)
            self.__print(cmd_line)

        # Execute the command
        try:
            result, output = self.__internal_shell_exec(cmd_line, 30)
        except:
            self.__print("exception while executing crashinfo "
                         "- command not available")
        else:
            if result == 0:
                if "PARSE ERROR" in output:
                    self.__print("Error parsing filter")
                elif "Clean : OK" in output:
                    self.__print("Erase OK")
                else:
                    self.__print("Erase NOK")
            else:
                self.__print("Error in execution")
                self.__print(output)
            return

    def get_status(self, uptime=False):
        """
        Returns the device status
        @rtype: dictionary
        @return: dictionary containing fields and their values. Empty dict if
        command failed.
        """
        # Define expected fields returned by crashinfo 'status' command on device
        STATUS_CMD_FIELS_V0 = {0: 'version', 1: 'critical_crash', 2: 'events', 3: 'path_logs'}
        STATUS_CMD_FIELS_V1 = {0: 'version', 1: 'critical_crash', 2: 'events', 3: 'path_logs', 4: 'api_version'}
        UPTIME_CMD_FIELDS = {1: 'days', 2: 'hours', 3: 'minutes', 4: 'seconds'}
        status = {}
        cmd_line = "adb shell crashinfo status"

        if uptime:
            cmd_line = "%s %s" % (cmd_line, "--uptime")
            self.__print(cmd_line)

        # Execute the command
        try:
            self.__print("execute: {0}".format(cmd_line))
            result, output = self.__internal_shell_exec(cmd_line, 30)
        except:
            self.__print("exception while executing crashinfo "
                         "- command not available")
        else:
            # Check command result and output format (output status cmd is expected to
            # have a minimum number of fields with a line per field)
            if "Exception" in output:
                self.__print("Cannot retrieve crash_info status: {0}".format(output))
                return {}

            split_output = output.split('\n')
            if result == 0 and (len(split_output) >= len(STATUS_CMD_FIELS_V0) - 1):
                # Parse result from output to extract data and put them into the
                # returned dictionary
                status = {}
                # Compute the fields to use
                if len(split_output) > len(STATUS_CMD_FIELS_V0):
                    fields_dict = STATUS_CMD_FIELS_V1
                else:
                    fields_dict = STATUS_CMD_FIELS_V0

                # Check first which fields should be retrieved and then
                # retrieve their values
                if uptime:
                    # Create a dictionary with uptime values
                    for field in split_output:
                        field_split = field.split(' ')
                        value = field_split[0]
                        unit = field_split[1]
                        if unit:
                            unit = unit.strip().lower()
                        else:
                            unit = "unknown"
                        if unit in UPTIME_CMD_FIELDS.values():
                            status[unit] = value
                    # Consolidate the status dictionary
                    for unit in UPTIME_CMD_FIELDS.values():
                        if unit not in status:
                            status[unit] = "0"
                else:
                    for index in fields_dict.iterkeys():
                        status[fields_dict[index]] = \
                            split_output[index].split(':')[1].strip()

            else:
                self.__print("Issue while getting status info:")
                self.__print(output)

        return status

    def pull_event(
        self,
        row_id=-1,
        log_path=None,
        crash_server="tldlab112",
        port_server="4002"):
        """
        Pulls the given event from the I{DUT} and return a status
        indicating the operation's status.

        @rtype: tuple
        @return: a tuple indicating the operation success with the
            following pieces of data:
            - C{[0|1]} with:
                - 0: in case of failure
                - 1: otherwise
            - <str>:  the updated logpath
                - a string indicating the log path
                - C{None} if there was no logs to upload.
            - [optional] <str>: additional information
        """
        event_to_pull = self.get_event(2, "id", row_id)
        # If not event has been found
        if len(event_to_pull) == 0:
            # Return a tuple with the expected information
            error_msg = "%s : Pull KO : no event found" % self.__program_name
            return (0, None, error_msg)

        self.__print("ID : " + event_to_pull[0]["_id"])
        crash_dir = event_to_pull[0]["crashdir"]

        # Checks if the event has a log directory and if this log is valid
        if crash_dir and event_to_pull[0]['logsuploaded'] != '-1':
            if log_path is None:
                log_path = os.path.abspath("./log" + row_id + "/")
            else:
                log_path = os.path.join(log_path, row_id)

            cmd_line = "adb pull"
            cmd_line = "%s %s \"%s\"" % (cmd_line, crash_dir, log_path)
            # Timeout is huge, because someteime we've got to fetch BP logs (~500MB)
            result, output = self.__internal_shell_exec(cmd_line, 300)
            # If the command succeeded
            if result == 0:
                self.__print("adb pull event OK")
                if (not os.path.isdir(log_path)):
                    # Create a tmp file to indicate nothing was found on board
                    # (because the result was 0, it means it is "normal").
                    # Fix done for managing empty crashlog folder.
                    os.makedirs(log_path)
                    path_suffix = "/empty_directory_generated_by_lib_acs_crashinfo"
                    temp_path = os.path.normpath(log_path + path_suffix)
                    write_file(temp_path, '')

                result_upload = self.upload_event(
                    log_path,
                    event_to_pull[0],
                    crash_server,
                    port_server)

                # Upload event state in CrashReport DB depending
                # on upload status
                self.__update_upload_state(
                    row_id,
                    event_to_pull[0],
                    result_upload)

                # Return a tuple with the expected information
                return (1, log_path, "'adb pull' command success.")
            # If the command failed
            else:
                self.__print(cmd_line)
                self.__print("adb pull failed")
                self.__print(output)
                # Return a tuple with the expected information
                error_msg = "'adb pull' command failed (%s)." % cmd_line
                return (0, log_path, error_msg)
        else:
            # No crashlog file or invalid crashlog : upload only the event
            result_upload = self.upload_event(
                "",
                event_to_pull[0],
                crash_server,
                port_server)

            # Upload event state in CrashReport DB depending on upload status
            self.__update_upload_state(row_id, event_to_pull[0], result_upload)

            # Return a tuple with the expected information
            return (
                1,
                None,
                "Only event has been updated (no logs or invalid crashlogs.)")

    def upload_event(self, event_path, event={}, crash_server="", port_server="4002"):
        """
        Upload the input event depending on its "uploaded" and "logsuploaded" attributes values:
        uploaded = -1 & logsuploaded =  x => don't upload the event and returns -1
        uploaded =  1 & logsuploaded =  1 => don't upload the event and returns 0
        uploaded =  1 & logsuploaded = -1 => don't upload the event and returns 0
        uploaded =  0 & logsuploaded =  0 => upload the event and returns ClotaClient returned value
        uploaded =  1 & logsuploaded =  0 => upload the event and returns ClotaClient returned value
        uploaded =  0 & logsuploaded = -1 => upload the event without its log and returns ClotaClient returned value
        """
        import platform
        result = -1
        uploaded = event["uploaded"]
        logsUploaded = event["logsuploaded"]
        # Invalid event : don't upload and return -1
        if (uploaded == "-1"):
            self.__print("event is invalid : upload aborted")
            return 2
        # Not uploaded event or logs : upload and return ClotaClient returned value
        elif ((uploaded == "0") or (uploaded == "1" and logsUploaded == "0")):
            if platform.system() == "Windows":
                class_path = "-cp .;%s" % (os.path.normcase(".\jar\*"))
            else:
                class_path = "-cp .:%s" % (os.path.normcase("./jar/*"))
            class_name = "ClotaClient"
            cmd_line = "java %s %s" % (class_path, class_name)

            # Add device info to event if available
            device_info = []
            device_info = self.get_device_info()
            event['spid'] = str(device_info['spid']) if 'spid' in device_info else ''
            event['gcmToken'] = str(device_info['gcmToken']) if 'gcmToken' in device_info else ''

            # Store ingredients seperatedly
            if 'ingredients' in event:
                arg_ingredients = event['ingredients']
                # Replace ingredients by an empty value
                event['ingredients'] = ""
            else:
                arg_ingredients = ""
            # Consolidate ingredients argument
            arg_ingredients = "%s%s%s" % ("\'", str(arg_ingredients), "\'")

            arg_event = "\"" + str(event) + "\""
            # if log is invalid don't send it with event
            if logsUploaded == '-1':
                self.__print("event log is invalid : upload event only")
                event_path = ''
            arg_log_path = "\"" + event_path + "\""
            arg_build_id = "\"" + str(self.get_build_id()) + "\""
            current_dir = os.getcwd()
            self.__print("arg_event: '%s'" % str(arg_event))
            self.__print("arg_build_id: '%s'" % str(arg_build_id))
            self.__print("crash_server: '%s'" % str(crash_server))
            self.__print("arg_log_path: '%s'" % str(arg_log_path))
            self.__print("port_server: '%s'" % str(port_server))
            self.__print("arg_ingredients: '%s'" % str(arg_ingredients))
            cmd_line = "%s --event=%s --build=%s --host=%s --log_path=%s --port=%s --ingredients=%s" % (
                cmd_line, arg_event, arg_build_id, crash_server, arg_log_path, port_server, arg_ingredients)
            try:
                new_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                        "javacrashtool")
                os.chdir(new_path)
                # Performs the upload command and returns the output status value
                try:
                    self.__print("Executing command : " + cmd_line)
                    result, output = self.__internal_shell_exec(cmd_line, 50)
                except:
                    self.__print("exception while executing java crashtool client")
                    return -1
                else:
                    self.__print(output)
                    if result != -1:
                        self.__print(("Java ClotaClient upload_event command success : status = %d") % result)
                    else:
                        self.__print(("Error on execution of java crashtool client : status = %d") % result)
                    return result
            finally:
                os.chdir(current_dir)
        # Event already uploaded : don't upload and return 0
        else:
            self.__print("event already uploaded")
            return 0

    def update_upload_state(self, row_id=-1, option='logNotUploaded'):
        """
        This function uploads the state of an event in CrashReport DB on device.
        Returns OK if succeed. KO otherwise.
        """

        UPDATE_OPTIONS = ['logNotUploaded', 'logUploaded', 'logInvalid', 'eventInvalid']

        cmd_line = "adb shell crashinfo uploadstate --filter-id %s" % (row_id)

        if option in UPDATE_OPTIONS:
            if option == 'logUploaded':
                cmd_line = "%s %s" % (cmd_line, "--log")
            elif option == 'logInvalid':
                cmd_line = "%s %s" % (cmd_line, "--invalid-log")
            elif option == 'eventInvalid':
                cmd_line = "%s %s" % (cmd_line, "--invalid-event")
        else:
            raise ValueError("Invalid input option=%s" % option)
            return 'KO'

        # Execute the command
        try:
            result, output = self.__internal_shell_exec(cmd_line, 30)
        except:
            self.__print("exception while executing crashinfo "
                         "- command not available")
        else:

            if result == 0:
                self.__print("update uploaded state : OK")
            else:
                self.__print(cmd_line)
                self.__print("Issue while updating upload state:")
                self.__print(output)
                return "KO"

        return "OK"

    def set_serial_number(self, serial):
        if serial not in (None, "None", ""):
            self.__device_serial_number = serial

#------------------------------------------------------------------------------


def setup_arg_parser():
    parser = OptionParser()
    parser.add_option("-d", "--device_id", "--dvi",
                      help="Device id or serial number of the DUT. If not provided, "
                      "ANDROID_SERIAL environment variable content is used",
                      metavar="DEVICE_ID",
                      default="",
                      dest="device_id")

    parser.add_option("--get_event", "--ge",
                      help="Returns event info present in database",
                      action="store_true", default=False,
                      dest="get_event")
    parser.add_option("--get_detail_event", "--gde",
                      help="Returns details event info present in database",
                      action="store_true", default=False,
                      dest="get_detail_event")
    parser.add_option("--get_full_event", "--gfe",
                      help="Returns full event info present in database",
                      action="store_true", default=False,
                      dest="get_full_event")
    parser.add_option("--filter",
                      help="Filter",
                      metavar="FILTER",
                      default=None, dest="filter")

    parser.add_option("--get_build_id", "--build",
                      help="Returns the build signature",
                      action="store_true", default=False,
                      dest="get_build_id")
    parser.add_option("--spec",
                      help="[To use w/ \"--build\" option] Returns the"
                      " specification of the build signature",
                      action="store_true", default=False,
                      dest="get_build_id_spec")

    parser.add_option("--clean", "-c",
                      help="Erases unused logs",
                      action="store_true", default=False,
                      dest="clean")
    parser.add_option("--force_all",
                      help="[To use w/ \"-c\" option] Cleans all logs"
                      " folder, history event and eventdb.",
                      action="store_const", const="force-all",
                      dest="filter_ge")
    parser.add_option("--filter-id",
                      help="[To use w/ \"-c\" option] filter by ID",
                      action="store_const", const="id",
                      dest="filter_ge")
    parser.add_option("--filter-time",
                      help="[To use w/ \"-c\" option] filter by time",
                      action="store_const", const="time",
                      dest="filter_ge")
    parser.add_option("--filter-name",
                      help="[To use w/ \"--ge\" option] filter by name",
                      action="store_const", const="name",
                      dest="filter_ge")
    parser.add_option("--filter-type",
                      help="[To use w/ \"--ge\" option] filter by type",
                      action="store_const", const="type",
                      dest="filter_ge")
    parser.add_option("--last",
                      help="[To use w/ \"--ge\" option] to get last event",
                      action="store_const", const="last",
                      dest="filter_ge")
    parser.add_option("--uploaded",
                      help="[To use w/ \"--ge\" option] to get uploaded event",
                      action="store_const", const="uploaded",
                      dest="filter_ge")
    parser.add_option("--status", "-s",
                      help="Give the status of crash info data",
                      action="store_true", default=False,
                      dest="status")
    parser.add_option("--uptime",
                      help="[To use w/ \"-s\" option] Give phone uptime",
                      action="store_true", default=False,
                      dest="uptime")
    parser.add_option("--pull_event",
                      help="Get phone log on host",
                      action="store_true", default=False,
                      dest="pull")
    parser.add_option("--pull_all_events",
                      help="Push all events on crashtool server",
                      action="store_true", default=False,
                      dest="pull_all")
    return parser
#------------------------------------------------------------------------------


def print_events(events):
    for event in events:
        for data in sorted(event):
            print "%s = %s" % (data, event[data])
        print ""


def pull_events(events, crash_server="tldlab112", port_server="4002"):
    try:
        from cStringIO import StringIO
    except:
        print "Missing module cStringIO"
        return

    # Number of events
    count = len(events)
    print "%d Events" % count

    # Iterate through all events to upload event + log files if necessary
    failures = 0
    index = 0
    for event in events:
        index = index + 1
        event_id = event["_id"]
        crashdir = event["crashdir"]
        uploaded = event["uploaded"]
        logsuploaded = event["logsuploaded"]
        print ">> Uploading event %d / %d (id = %s)" % (index, count, event_id),
        if (uploaded == "1" and (crashdir == "" or logsuploaded == "1")):
            print " => Already uploaded"
        elif (uploaded == "-1" or (uploaded == "1" and crashdir != "" and logsuploaded == "-1")):
            print " => Invalid event => Skipped"
        else:
            sys.stdout = mystdout = StringIO()
            result = crash_info_lib.pull_event(event_id, None, crash_server, port_server)
            (status, logpath, message) = result
            sys.stdout = sys.__stdout__
            if not status:
                print "Upload of event (id) %s failed. Cause: %s" % (
                    event_id,
                    message)
            else:
                print "Upload of event (id) %s succeeded. Log path: %s." % (
                    event_id,
                    logpath)
            if ("upload_event command success : status = 0" in mystdout.getvalue()):
                print " => Succeeded"
            else:
                print " => Failed"
                print mystdout.getvalue()
                failures = failures + 1

    if (failures == 0):
        print "SUCCEEDED: %d events uploaded successfully." % count
    else:
        print "ERROR: %d events uploaded failures." % failures


#------------------------------------------------------------------------------
if __name__ == "__main__":
    arg_parser = setup_arg_parser()
    (options, args) = arg_parser.parse_args()

    program_name = os.path.basename(sys.argv[0])

    if len(sys.argv) == 1:
        arg_parser.print_help()
        sys.exit(-1)

    # Manage Device_Id
    if options.device_id:
        Android_Serial = options.device_id
    else:
        Android_Serial = None

    crash_info_lib = CrashInfo(False, Android_Serial, program_name)

    if options.get_event:
        events = crash_info_lib.get_event(0, options.filter_ge, options.filter)
        print_events(events)
    if options.get_detail_event:
        events = crash_info_lib.get_event(1, options.filter_ge, options.filter)
        print_events(events)
    if options.get_full_event:
        events = crash_info_lib.get_event(2, options.filter_ge, options.filter)
        print_events(events)
    if options.get_build_id:
        build_info = crash_info_lib.get_build_id(options.get_build_id_spec)
        for info in sorted(build_info):
            print "%s = %s" % (info, build_info[info])
    if options.clean:
        crash_info_lib.clean(options.filter_ge, options.filter)
    if options.status:
        infos = crash_info_lib.get_status(options.uptime)
        for info in sorted(infos):
            print "%s = %s" % (info, infos[info])
    if options.pull:
        print crash_info_lib.pull_event(options.filter, None, "crashtool.iind.intel.com", "4002")
        # print crash_info_lib.pull_event(options.filter, None, "crashtool.iind.intel.com")
        # print crash_info_lib.pull_event(options.filter, None, "tldlab112.tl.intel.com", 4000)
        # print crash_info_lib.pull_event(options.filter, None, "10.102.160.78") # integration server for test purpose
        # print crash_info_lib.pull_event(options.filter, None, "mauretx-mobl", "4002")
        # print crash_info_lib.pull_event(options.filter, None, "10.102.233.29", "4002")
        # print crash_info_lib.pull_event(options.filter, None, "tllab144", "4002")
    if options.pull_all:
        events = crash_info_lib.get_event(1, options.filter_ge, options.filter)
        # pull_events(events, "10.102.160.78", "4002") # integration server for test purpose
        pull_events(events, "crashtool.iind.intel.com", "4002")
