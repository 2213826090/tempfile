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
:summary: This file implements an engine which performs ftp_xfer,
          web crawling using curl command.
:since: 29/09/2011
:author: ssavrimoutou
"""
import os
import re
import sys
import time
import tempfile
import subprocess
import shutil
from datetime import datetime
from Core.PathManager import Paths
from ErrorHandling.AcsConfigException import AcsConfigException

from UtilitiesFWK.Utilities import Global


class CurlBinder():

    """
    This class implements a binder to execute curl commands.
    curl package MUST be installed on the host.
    For windows XP follow link : http://curl.haxx.se/dlwiz/?type=bin&os=Win32&flav=-&ver=2000%2FXP
    For linux (Ubuntu) : apt-get install curl
    """

    def __init__(self):
        """
        Initialize curl binder
        """
        self._cmd = "curl"
        self._default_timeout = 30

    def __parse_stderr(self, error_message):
        """
        Parse error message returned by curl command on stderr

        :type error_message: string
        :param error_message: Error message to parse

        :rtype: string
        :return: parsed error message
        """
        parse_err_msg = ""

        # Before parsing output, remove all special characters
        error_message = str(error_message).replace('\n', '')
        error_message = str(error_message).replace('\r', '')

        # Parse output in order to retrieve "code" and "message".
        # Output must be to this form:
        # "curl: (code) message" with message a string
        # and code representing an integer
        matches_str = re.match(".*curl: \((?P<code>.*)\) (?P<message>.*)$",
                               error_message)

        # if output doesn't contain tag,
        # return without parsing but add a warning
        if matches_str is None:
            parse_err_msg = error_message
        else:
            # Retrieve code and message
            parse_err_msg = \
                "error code (%s) : %s" % (matches_str.group("code"),
                                          matches_str.group("message"))

        return parse_err_msg

    def send_cmd(self,
                 command):
        """
        Execute the command and return the result

        :type command: string
        :param command: command to be executed

        :rtype: tuple
        :return: execution status (FAILURE or SUCCESS), stdout, stderr
        """

        # Initialize variables used for SubProcess
        proc = None
        expected_result = 0
        exec_status = Global.FAILURE
        stdout = ""
        stderr = ""

        # Temporary files created here are used as a SubProcess output
        f_stdout = tempfile.TemporaryFile()
        f_stderr = tempfile.TemporaryFile()

        # Build curl command before sending it using Popen
        if not command.startswith(self._cmd):
            args = [self._cmd]
        else:
            args = []

        for item in command.split():
            args.append(item)

        # Extract timeout option if exists
        if "-m" in args:
            timeout_index = args.index("-m") + 1
            timeout_cmd = args[timeout_index]

        elif "--max-time" in args:
            timeout_index = args.index("--max-time") + 1
            timeout_cmd = args[timeout_index]

        else:
            # Add default timeout if the option does not exist
            timeout_cmd = self._default_timeout
            args.append("-m")
            args.append("%s" % str(timeout_cmd))

        # Call popen to send the command
        try:
            print "*** RUN: %s" % ' '.join(args)
            proc = subprocess.Popen(args, stdout=f_stdout, stderr=f_stderr)

            # Add 5 more seconds for execution of the command
            timeout_cmd = float(timeout_cmd) + 5
            # pylint: disable=E1101
            while timeout_cmd > 0 and proc.poll() is None:
                # pylint: disable=C0103
                # Agree to keep t0 & t1 variable names
                t0 = time.time()
                time.sleep(0.2)
                t1 = time.time()
                timeout_cmd -= (t1 - t0)

            # Process or timeout terminated
            # Get return code
            return_code = proc.poll()

            if return_code is not None:
                if return_code == expected_result:
                    # If expected_result is an integer
                    # and if it's equal to the return code then
                    exec_status = Global.SUCCESS
                else:
                    stderr = "Command failed : "

            else:
                # Process has not finished to execute command.
                stderr = "Command has timeout : "

            # Get stdout
            f_stdout.seek(0)
            stdout = f_stdout.read()
            # Get stderr
            f_stderr.seek(0)
            # parse stderr
            parsed_stderr = self.__parse_stderr(f_stderr.read())
            stderr += parsed_stderr

        finally:
            # pylint: disable=E1101
            if proc is not None and not proc.terminate:
                proc.terminate()

            f_stdout.close()
            f_stderr.close()

        stdout = stdout.strip("\r\n")

        # Parsing the return messages from the command
        if exec_status == Global.FAILURE:
            if len(stdout) > 0:
                output = "stdout:\"%s\"" % stdout
            else:
                output = "stdout: empty"

            if len(stderr) > 0:
                output += ";stderr:\"%s\"" % stderr
            else:
                output += ";stderr: empty"

        else:
            output = "stdout: %s" % stdout

        print "*** OUTPUT : %s" % str(output)
        return exec_status, output

#-------------------------------------------------------------------------------


class CurlEngine():

    """
    This class implements an engine which using curl commands
    to perform transfer data from or to a server.
    """

    CURL_TOKEN_START = "CURL_TOKEN_START"
    CURL_TOKEN_STOP = "CURL_TOKEN_STOP"

    def __init__(self,
                 report_path=None,
                 execution_path=None):
        """
        Initialize curl Engine

        :type report_path: string
        :param report_path: Folder to store files,
            For FTP it is also used to to store files to upload

        :type execution_path: string
        :param execution_path: Folder which contains the file containing all curl commands to execute
        """

        # Local paths
        if execution_path is None:
            execution_path = Paths.EXECUTION_CONFIG
        if report_path is None:
            report_path = Paths.REPORTS
        self.__execution_path = execution_path
        self.__report_path = report_path

        self.__curl_binder = CurlBinder()

        error_file_log = "curl_error_log_%s.log" % \
            str(datetime.now().strftime("%Y-%m-%d_%Hh%M.%S"))

        self.__error_log_file = os.path.join(self.__report_path,
                                             error_file_log)

    def __write_error_log(self,
                          error_message):
        """
        Store error message from stderr into a log file
        :type error_message: String
        :param error_message: Message to log
        """

        if not os.path.exists(self.__report_path):
            os.mkdir(self.__report_path)

        file_desc = open(self.__error_log_file, 'a')
        file_desc.write("%s ERROR\t%s\n" %
                       (str(datetime.now().strftime("%H:%M:%S")),
                           error_message))
        file_desc.close()

    def __extract_cmds(self, config_file):
        """
        Extracts command from config file. This function will returns a list of curl commands.

        :type config_file  : string
        :param config_file : Configuration file which contains all curl commands

        :rtype: list
        :return: List of curl commands
        """
        cmds_list = []
        built_path_file = config_file

        if not os.path.isfile(built_path_file):
            # Try to find it in _Execution Config folder
            built_path_file = os.path.join(self.__execution_path,
                                           built_path_file)

        if os.path.isfile(built_path_file):
            file_desc = open(built_path_file, 'r')
            # Get lines without empty lines
            lines = [l for l in file_desc.readlines() if l.strip()]
            file_desc.close()

            for line in lines:
                # Avoid commented lines
                if not line.startswith("#"):
                    line = line.strip("\n")
                    cmds_list.append(line)

        else:
            return_msg = "Configuration file '%s' not found. " % str(config_file)
            return_msg += "Checks that it is in _Execution Config or give full path file."
            raise AcsConfigException(AcsConfigException.FILE_NOT_FOUND, return_msg)

        return cmds_list

    def __check_errors(self):
        """
        At the end of the curl engine sequences, we verify if there is no error occured

        :rtype: tuple
        :return: Returns the error code and error message
        """

        # Initialize local variables
        return_code = Global.SUCCESS
        return_msg = "No error"
        error_file_list = ""

        # Checking if errors occured
        files = os.listdir(self.__report_path)

        for filename in files:
            if "curl_error_log" in filename:
                return_code = Global.FAILURE
                error_file_list += " " + filename

        # Compute verdict
        if return_code == Global.FAILURE:
            return_msg = "Error occured during curl processing. "
            if error_file_list != "":
                return_msg += "Check following file(s) in Campaign report for further details :\n"
                return_msg += error_file_list

        # Remove temp directory
        tempdir = os.path.join(self.__report_path, "temp")
        shutil.rmtree(tempdir, True)

        return return_code, return_msg

    def start(self,
              config_file,
              timeout=None):
        """
        Start sequence to execute curl commands

        :type config_file  : string
        :param config_file : Configuration file which contains all curl commands

        :type timeout  : int
        :param timeout : Timeout used for loop condition
        """

        print "Starting curl engine"

        # Initialize local variables
        return_code = Global.SUCCESS
        return_msg = "No error"
        start_time = time.time()
        timeout_reached = False
        stop_engine = False

        # Initialize temp dir, files
        tempdir = os.path.join(self.__report_path, "temp")
        if not os.path.isdir(tempdir):
            os.makedirs(tempdir)
        token_start = os.path.join(tempdir, CurlEngine.CURL_TOKEN_START)
        token_stop = os.path.join(tempdir, CurlEngine.CURL_TOKEN_STOP)

        # Extract curl commands from config file
        cmds_list = self.__extract_cmds(config_file)
        if not cmds_list:
            return_msg = "No curl commands found in configuration file '%s'." % str(config_file)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, return_msg)

        # Create a token file for loop condition
        f = open(token_start, "w")
        f.close()

        # Remove stop token file if exists
        if os.path.isfile(token_stop):
            os.remove(token_stop)

        # Start looping on curl commands
        while not stop_engine and not timeout_reached:
            # Execute each curl command
            for curl_cmd in cmds_list:
                (exec_status, exec_output) = self.__curl_binder.send_cmd(curl_cmd)
                # Store error in log file
                if exec_status == Global.FAILURE:
                    self.__write_error_log(exec_output)
                time.sleep(5)

                # Stop looping if the token stop exists
                if os.path.isfile(token_stop):
                    stop_engine = True
                    break

                # Stop looping if timeout is reached
                if timeout is not None:
                    current_time = int(time.time() - start_time)
                    if current_time >= int(timeout):
                        timeout_reached = True
                        break

        # Remove start token file to signal that loop is finished
        if os.path.isfile(token_start):
            os.remove(token_start)

        # Compute verdict in case timeout is reached
        if timeout_reached:
            (return_code, return_msg) = self.__check_errors()

        return return_code, return_msg

    def stop(self, timeout=60):
        """
        Stop the curlEngine and check if error occurs
        """

        print "Stopping curl engine"

        # Initialize local variables
        start_time = time.time()
        timeout_reached = False

        # Initialize temp dir, files
        tempdir = os.path.join(self.__report_path, "temp")
        if not os.path.isdir(tempdir):
            os.makedirs(tempdir)
        token_start = os.path.join(tempdir, CurlEngine.CURL_TOKEN_START)
        token_stop = os.path.join(tempdir, CurlEngine.CURL_TOKEN_STOP)

        # Create token stop to stop curl engine
        f = open(token_stop, "w")
        f.close()

        # Waiting for curl engine to finish
        current_time = int(time.time() - start_time)
        while os.path.isfile(token_start) and not timeout_reached:
            time.sleep(1)
            current_time = int(time.time() - start_time)
            if current_time > int(timeout):
                timeout_reached = True

        # Check if the stop curl engine timed out
        if timeout_reached:
            print "Warning: stop curl engine reached timeout !"

        # Compute verdict
        (return_code, return_msg) = self.__check_errors()

        return return_code, return_msg

#------------------------------------------------------------------------------


def display_help():
    """
    Print how to use from the command line ACS
    """

    print ("Usage: %s <command>"
           % (sys.argv[0]))
    print ("")
    print ("<command> => Command to launch start|stop\n")
    print ("             start <config_file> <timeout> : to start the curl engine ")
    print ("                             <config_file> : Configuration file containing the curl commands")
    print ("                             <timeout>     : Optional timeout in seconds\n")
    print ("             stop  <timeout>               : to stop the curl engine  with optional timeout")
    print ("                             <timeout>     : Optional timeout in seconds")

#------------------------------------------------------------------------------

if __name__ == "__main__":

    try:
        return_code = -1

        if len(sys.argv) < 2:
            display_help()
            return_code = -1

        else:

            # Initialize curl Engine
            index = 0
            report_path = None
            for parameter in sys.argv:
                if "--output=" in parameter:
                    report_path = parameter.split("=")[1]
                    sys.argv.pop(index)
                    break
                index += 1

            if report_path is None:
                curlEngine = CurlEngine()
            else:
                curlEngine = CurlEngine(report_path)
            params = sys.argv[2:]

            if sys.argv[1] == "start":

                if len(sys.argv) < 3:
                    display_help()
                    return_code = -1
                else:
                    # Starting the curl Engine
                    (return_code, output) = curlEngine.start(*params)  # pylint: disable=W0142
                    print output

            elif sys.argv[1] == "stop":
                # Stopping the curl engine
                (return_code, output) = curlEngine.stop(*params)  # pylint: disable=W0142
                print output

            else:
                display_help()
                return_code = -1

        sys.exit(return_code)

    except KeyboardInterrupt:
        sys.exit("Killed by user")

    except AcsConfigException as ex:
        sys.exit(ex.get_error_message())
