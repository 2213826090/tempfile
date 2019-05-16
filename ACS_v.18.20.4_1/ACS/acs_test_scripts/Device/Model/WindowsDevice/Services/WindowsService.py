# pylint: disable = C0303
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
:summary: This file define an Exception class handling http request problems
:since: 16/06/2014
:author: dgonza4x
"""
import json
import time

from acs_test_scripts.Utilities.HTTPRequestHandler.HTTPRequestHandler import HTTPRequestHandler
from acs_test_scripts.Utilities.HTTPRequestHandler.HTTPRequestException import HTTPRequestException
from CommandResult import CommandResult
from CommandResultException import CommandResultException


class WindowsService:

    """
    Class that handle communication with the Windows Service
    running on the DUT, through http requests.
    """

    __ROOT_URL_PATTERN = "http://%s:%s/Acs/ws/"
    """
    Base url PATTERN used to call each methods of this web service
    """

    # these constants represent what Windows returns as status for task<result> class
    ASYNC_TASK_FINISH = "RANTOCOMPLETION"
    """
    Define an asynchronous task finished
    """

    ASYNC_TASK_FAULTED = "FAULTED"
    """
    Define an asynchronous task that has failed
    """

    ASYNC_TASK_CANCELED = "CANCELED"
    """
    Define an asynchronous task that has been canceled
    """

    ASYNC_TASK_RUNNING = "RUNNING"
    """
    Define an on-going asynchronous task
    """

    ASYNC_TASK_UNKNOWN = "UNKNOWN"
    """
    Define an unknown asynchronous task (bad id)
    """

    def __init__(self, device_ip, device_port, proxies, logger, default_timeout):
        """
        Constructor

        :type device_ip: str
        :param device_ip: String representing a valid IPv4 address

        :type device_port: str
        :param device_port: string representing a valid port number

        :type proxies: dict
        :param proxies: Dictionary mapping protocol to the URL of the proxy
        (e.g. "http": "foo.bar:3128") to be used on each requests

        :type logger: object
        :param logger: Instance used to log messages

        :type default_timeout: int
        :param default_timeout: default command response timeout
        """
        self.__http_handler = HTTPRequestHandler(logger=logger, proxies=proxies)
        self.__default_timeout = default_timeout
        self.__device_ip = str(device_ip)
        self.__device_port = str(device_port)
        self.__logger = logger

    def __root_url(self):
        """
        Compute the root URL of this service

        :rtype: str
        :return: base url for every service's method
        """
        return self.__ROOT_URL_PATTERN % (self.__device_ip, self.__device_port)

    def get_version(self):
        """
        Retrieve the service version

        :rtype: CommandResult
        :return: A command result containing the specific value 'version' (string)
        """
        response = self.__http_handler.get_request(
            self.__root_url() + "GetVersion",
            self.__default_timeout)
        decoded = json.loads(response)["GetVersionResult"]
        _version = "%s.%s.%s.%s" % (decoded["_Major"], decoded["_Minor"], decoded["_Build"], decoded["_Revision"])
        result = CommandResult(CommandResult.VERDICT_PASS, "No error", {'version': _version})
        return result

    def get_processes(self):
        """
        Retrieve current processes IDs

        :rtype: CommandResult
        :return: A command result containing the specific value 'processes'.
        This value is a list of process name.
        """
        response = self.__http_handler.get_request(
            self.__root_url() + "GetProcesses",
            self.__default_timeout)
        result = CommandResult(CommandResult.VERDICT_PASS, "No error",
                               {'processes': json.loads(response)["GetProcessesResult"]})
        return result

    def get_serial_number(self):
        """
        Retrieve the DUT serial number

        :rtype: CommandResult
        :return: A command result containing the specific value 'serial_number' (string).
        """
        response = self.__http_handler.get_request(
            self.__root_url() + "GetSerialNumber",
            self.__default_timeout)
        result = CommandResult(CommandResult.VERDICT_PASS, "No error",
                               {'serial_number': json.loads(response)["GetSerialNumberResult"]})
        return result

    def is_alive(self):
        """
        Check that the service is alive (replace 'Ping' command)

        :rtype: CommandResult
        :return: A command result containing the specific value 'is_alive' (boolean).
        """
        try:
            response = self.__http_handler.get_request(
                self.__root_url() + "GetState",
                self.__default_timeout)
            decoded = json.loads(response)
            is_up_str = str(decoded["GetStateResult"])
            result = CommandResult(CommandResult.VERDICT_PASS, "No error",
                                   {'is_alive': is_up_str.upper() == "ALIVE"})
        except HTTPRequestException:
            result = CommandResult(CommandResult.VERDICT_PASS, "No error",
                                   {'is_alive': False})
        return result

    def install(self, binary_pathname):
        """
        Install a binary on the DUT

        :type binary_pathname: str
        :param binary_pathname: complete file pathname (HOST path) of the binary to install

        :attention: not implemented yet.
        """
        raise Exception("WindowsService.install not implemented yet!")

    def push(self, file_pathname, destination):
        """
        Push a file on the DUP

        :type file_pathname: str
        :param file_pathname: complete file pathname (HOST path) of the file being pushed

        :type destination: str
        :param destination: complete file pathname (DUT side) representing the
        destination of the file to be pushed

        :attention: not implemented yet.
        """
        raise Exception("WindowsService.push not implemented yet!")

    def pull(self, file_pathname):
        """
        Retrieve a file from the DUT

        :type file_pathname: str
        :param file_pathname: complete file pathname (DUT path) of the file to be retrieved
        """
        raise Exception("WindowsService.pull not implemented yet!")

    def shell_cmd(self, binary_name, arguments, timeout):
        """
        Execute a shell command on the DUT

        :type binary_name: str
        :param binary_name: binary to execute on a Command prompt, DUT side

        :type arguments: str
        :param arguments: Arguments of the command to execute

        :type timeout: int
        :param timeout: Timeout of the command in seconds.
        If the command takes more than 'timeout' seconds, it will be considered as failed.

        :rtype: CommandResult
        :return: A command result containing as verdict message the command output.
        """
        if timeout is None:
            timeout = self.__default_timeout

        data = {
            "binary": binary_name,
            "args": arguments,
        }
        encoded_data = json.dumps(data)
        response = self.__http_handler.post_request(
            self.__root_url() + "ShellRun",
            encoded_data,
            timeout
        )
        decoded = json.loads(response)
        result = CommandResult(CommandResult.VERDICT_PASS, decoded["ShellRunResult"]["StdOut"])
        return result

    def launch_activity(self, assembly, class_name, method, args, timeout, debugger_mode):
        """
        Executes the specified uecmd returns the result

        :type  module_name: str
        :param module_name: assembly name of the uecmd to execute

        :type  class_name: str
        :param class_name: Full class name of the uecmd to execute

        :type  method_name: str
        :param method_name: method name corresponding to the uecmd to execute

        :type  args: str
        :param args: args to set as input parameter of the uecmd

        :type  timeout: int
        :param timeout: execution timeout in seconds

        :type debugger_mode: bool
        :param debugger_mode: True for enabling the Visual Studio debugger
        (need Microsoft Visual studio installed and the source code), False otherwise

        :return: Instance containing the result of the uecmd
        :rtype: CommandResult
        """
        if self.__logger:
            self.__logger.debug("Uecmd being executed : assembly=%s, class_name=%s, method=%s, args='%s'" %
                                (assembly, class_name, method, args))
        if debugger_mode is None:
            debugger_mode = False

        data = {
            "assembly": assembly,
            "class": class_name,
            "method": method,
            "debugger": debugger_mode,
            "args": args,
        }
        encoded_data = json.dumps(data)
        try:
            response = self.__http_handler.post_request(
                self.__root_url() + "LaunchActivity",
                encoded_data,
                timeout
            )
            decoded = json.loads(response)
        except HTTPRequestException as excp:
            raise CommandResultException("Communication error: " + str(excp))
        bundle_str = decoded["LaunchActivityResult"]["StdOut"]
        bundle = json.loads(bundle_str)
        result = self.__bundle_to_command_result(bundle)
        if self.__logger:
            self.__logger.debug("Result : %s" % str(result))
        return result

    def __bundle_to_command_result(self, bundle):
        """
        Encapsulate a result Bundle (response of a uecmd) into a CommandResult

        :type bundle: dict
        :param bundle: Dictionary containing data related to a uecmd execution

        :rtype: CommandResult
        :return: 'Packaged' result of a uecmd
        """
        result = CommandResult(bundle['verdict'], bundle['message'],
                               bundle['values'],
                               raised_excp=bundle['lastException'],
                               dev_logs=bundle['developerLogs'])
        return result

    def launch_async_uecmd(self, assembly, class_name, method, args, timeout, debugger_mode,
                           start_delay, repeat_delay=0, total_duration=0):
        """
        Executes the specified uecmd in Asynchronous mode, returning an
        identifier of the scheduled execution

        :type  assembly: str
        :param assembly: assembly name of the uecmd to execute

        :type  class_name: str
        :param class_name: Full class name of the uecmd to execute

        :type  method: str
        :param method: method name corresponding to the uecmd to execute

        :type  args: str
        :param args: args to set as input parameter of the uecmd

        :type  timeout: int
        :param timeout: execution timeout in seconds

        :type debugger_mode: bool
        :param debugger_mode: True for enabling the Visual Studio debugger
        (need Microsoft Visual studio installed and the source code), False otherwise

        :type  start_delay: int
        :param start_delay: time in second before starting the uecmd

        :type  repeat_delay: int
        :param repeat_delay: [optional] time in second before repeating
        the uecmd. If used, total_duration must be set too.

        :type  total_duration: int
        :param total_duration: [optional] the maximum duration time in second
        allowed to repeat the uecmd. If used, repeat_delay must be set too.

        :rtype: str
        :return: Task identifier
        """
        # check that all specific async parameters
        # are not null and higher than 0
        if start_delay < 0 or repeat_delay < 0 or total_duration < 0:
            raise Exception("Invalid asynchrone parameter " +
                            "(start_delay, repeat_delay or total_duration)")
        if type(debugger_mode) is not bool:
            debugger_mode = False

        data = {
            "assembly": assembly,
            "class": class_name,
            "method": method,
            "debugger": debugger_mode,
            "args": args,
            "repeat_delay": repeat_delay,
            "start_delay": start_delay,
            "total_duration": total_duration
        }
        encoded_data = json.dumps(data)
        try:
            response = self.__http_handler.post_request(
                self.__root_url() + "LaunchAsyncUecmd",
                encoded_data,
                timeout
            )
            decoded = json.loads(response)
        except HTTPRequestException as excp:
            raise CommandResultException("Communication error: " + str(excp))
        task_id = decoded["LaunchAsyncUecmdResult"]
        result = CommandResult(CommandResult.VERDICT_PASS, task_id, {'task_id': task_id})
        return result

    def get_async_result(self, task_id):
        """
        Retrieve the asynchronous UECmd status

        :type task_id: str
        :param task_id: Task identifier retrieved through 'launch_async_uecmd' method

        :rtype: tuple
        :return: Tuple containing as first element the status of the scheduled uecmd,
        and as second element the uecmd result (instance of CommandResult)
        """
        response = self.__http_handler.get_request(
            self.__root_url() + "getAsyncUecmdResult/" + task_id,
            self.__default_timeout)
        decoded = json.loads(response)
        status = str(decoded["getAsyncUecmdResultResult"]["m_Item1"]).upper()
        data = decoded["getAsyncUecmdResultResult"]["m_Item2"]["StdOut"]
        result = CommandResult(CommandResult.VERDICT_PASS, data, {'output': data})
        if self.ASYNC_TASK_FINISH == status:
            bundle = json.loads(data)
            result = self.__bundle_to_command_result(bundle)
        return (status, result)

    def clean_async_uecmd(self):
        """
        Clean all running task

        :rtype: CommandResult
        :return: Verdict of the command, containing as specific data:
        - 'status' : boolean defining whether the clean has been correctly executed or not
        """
        response = self.__http_handler.get_request(
            self.__root_url() + "cleanAsyncUecmd",
            self.__default_timeout)
        decoded = json.loads(response)
        is_done_str = str(decoded["cleanAsyncUecmdResult"])
        result = CommandResult(CommandResult.VERDICT_PASS, is_done_str,
                               {'status': is_done_str.upper() == "DONE"})
        return result


def __test_service():
    """
    Function testing the Windows service.
    :attention: the service must be launched before executing this file
    """
    service = WindowsService("192.168.0.166", "8080", None, 60)

    build_nbr = service.get_version()
    print "Version type:%s,  version data:%s" % (type(build_nbr), str(build_nbr))

    proc_list = service.get_processes()
    print "Process type:%s,  Process data:%s" % (type(proc_list), str(proc_list))

    serial_nbr = service.get_serial_number()
    print "Serial type:%s,  Serial data:%s" % (type(serial_nbr), str(serial_nbr))

    is_alive = service.is_alive()
    print "is_alive type:%s,  is_alive data:%s" % (type(is_alive), str(is_alive))

    shell_cmd = service.shell_cmd("ping", "127.0.0.1", 5)
    print "shell_cmd type:%s,  shell_cmd data:%s" % (type(shell_cmd), str(shell_cmd))

    uecmd_result = service.launch_activity("Intel.Acs.TestFmk.WifiConnectivity",
                                           "Intel.Acs.TestFmk.WifiConnectivity.WifiActivity",
                                           "ListAvailableEAPMethods",
                                           "", 20, False)
    print "launch_activity type:%s,  launch_activity data:%s" % (type(uecmd_result), str(uecmd_result))

    async_uecmd = service.launch_async_uecmd("Intel.Acs.TestFmk.WifiConnectivity",
                                             "Intel.Acs.TestFmk.WifiConnectivity.WifiActivity",
                                             "ListAvailableEAPMethods",
                                             "", 20, False, 7)
    print "launch_async_uecmd type:%s,  launch_async_uecmd data:%s" % (type(async_uecmd), str(async_uecmd))

    get_async_result = service.get_async_result(async_uecmd['task_id'])
    print "get_async_result type:%s,  get_async_result data:%s" % (
        type(get_async_result), (str(get_async_result[0]), str(get_async_result[1])))

    time.sleep(10)
    get_async_result = service.get_async_result(async_uecmd['task_id'])
    print "get_async_result type:%s,  get_async_result data:%s" % (
        type(get_async_result), (str(get_async_result[0]), str(get_async_result[1])))

    clean_async_uecmd = service.clean_async_uecmd()
    print "clean_async_uecmd type:%s,  clean_async_uecmd data:%s" % (
        type(clean_async_uecmd), str(clean_async_uecmd))


if __name__ == "__main__":
    __test_service()
