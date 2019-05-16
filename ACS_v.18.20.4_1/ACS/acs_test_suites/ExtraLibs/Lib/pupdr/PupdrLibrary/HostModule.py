#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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
@summary: Pupdr Library - HostModule
@since: 11/17/2014
@author: travenex
"""

import os
import sys
import base64
import urllib2
import signal
import subprocess
import LoggerModule
import datetime
import time
import shlex
import select
from Queue import Empty, Queue
from threading import Timer, Thread
try:
    # Try to load subprocess32
    import subprocess32 as subproc
except ImportError:
    # Else use std subprocess
    import subprocess as subproc

class HostModule(object):

    __instance = None
    __globalConf = None
    __logger = None
    __isExternal = None
    serial_number = None
    dnx_ssn = None
    ssnExtra = None
    __credentials = None
    fastboot_serial_number = None
    fastbootSsnExtra = None

    def __new__(cls):
        """
        Constructor that always returns the same instance
        """
        if not cls.__instance:
            cls.__instance = object.__new__(cls)
        return cls.__instance

    def init(self, globalConf=None, externalCmdExec=False):
        self.__globalConf = globalConf
        self.__logger = LoggerModule.LoggerModule()
        if externalCmdExec:
            self.__isExternal = True
        else:
            self.__isExternal = False
        self.updateSerialNumbers()
        self.__credentials = self.__globalConf.get("CREDENTIALS", "")
        if not self.__credentials or len(self.__credentials.split(":")) != 2 or [x for x in self.__credentials.split(":") if not x]:
            self.__logger.printLog("WARNING", "HostModule init: error with credentials, unexpected format or empty (artifactory download disabled)")
            self.__credentials = ""
        else:
            self.__logger.printLog("INFO", "HostModule init: using '{}' credentials".format(self.__print_creds_user()))

    def updateSerialNumbers(self, local_ssn=""):
        if not local_ssn:
            self.serial_number = self.__globalConf.get("DEVICE_SSN", "")
        else:
            self.serial_number = local_ssn
        self.fastboot_serial_number = self.__globalConf.get("FASTBOOT_SSN", self.serial_number)
        self.dnx_ssn = self.__globalConf.get("DNX_SSN", "")
        self.__logger.printLog("INFO", "Serial Number Update: starting with "
                                       "Android_SSN='{}', "
                                       "Fastboot_SSN='{}' and "
                                       "SOC_SSN='{}'".format(self.serial_number,
                                                             self.fastboot_serial_number,
                                                             self.dnx_ssn))
        if self.serial_number:
            self.ssnExtra = "-s " + self.serial_number + " "
        else:
            self.ssnExtra = ""
        if self.fastboot_serial_number:
            self.fastbootSsnExtra = "-s " + self.fastboot_serial_number + " "
        else:
            self.fastbootSsnExtra = ""
    @property
    def is_credential(self):
        return self.__credentials != ""

    def __print_creds(self):
        if self.__credentials:
            return self.__credentials.split(":")[0] + ":*********"
        else:
            return ""

    def __print_creds_user(self):
        if self.__credentials:
            return self.__credentials.split(":")[0]
        else:
            return ""

    # internal function if not provided
    def __internalCmdExec(self, command, timeout, silent_mode=False):
        log = "internalCmdExec():"
        if not silent_mode:
            self.__logger.printLog("INFO", "*** RUN: {0} (timeout={1}s)".format(command, timeout))

        # timeout inner method
        def timeout_end():
            self.__logger.printLog("WARNING", "*** TIMEOUT ! ***")
            os.killpg(p.pid, signal.SIGTERM)
            time.sleep(5)
            os.killpg(p.pid, signal.SIGKILL)
            return 1, ""

        # create signal alarm
        t = Timer(timeout, timeout_end)
        t.start()
        p = subprocess.Popen(command.split(), stdout=subprocess.PIPE, stderr=subprocess.STDOUT, preexec_fn=os.setsid)
        output = ""
        try:
            while p.poll() is None:
                # Loop output
                try:
                    for line in iter(p.stdout.readline, ''):
                        output += line + "\r\n"
                        if not silent_mode:
                            for l in line.rstrip("\r\n").replace("\r", "").split("\n"):
                                self.__logger.printLog("INFO", l)
                except Exception as e:
                    self.__logger.printLog("WARNING", "*** EXCEPTION ! *** ({0})".format(e))
        except KeyboardInterrupt:
            self.__logger.printLog("WARNING", "*** Interrupted ! ***")
            os.killpg(p.pid, signal.SIGTERM)
            time.sleep(5)
            os.killpg(p.pid, signal.SIGKILL)
        # Check returned code
        t.cancel()
        code = p.returncode
        if code != 0:
            # print code
            self.__logger.printLog("WARNING", "*** COMMAND FAILED!", frontContent=log)
        return code, output.rstrip("\r\n")

    # internal function if not provided
    def __internalShellCmdExec(self, command, timeout, silent_mode=False):
        log = "internalCmdExec():"
        if not silent_mode:
            self.__logger.printLog("INFO", "*** RUN: {0} (timeout={1}s)".format(command, timeout))

        # timeout inner method
        def timeout_end():
            self.__logger.printLog("WARNING", "*** TIMEOUT ! ***")
            os.killpg(p.pid, signal.SIGTERM)
            return 1, ""

        # create signal alarm
        t = Timer(timeout, timeout_end)
        t.start()
        p = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, preexec_fn=os.setsid, shell=True)
        output = ""
        try:
            while p.poll() is None:
                # Loop output
                try:
                    for line in iter(p.stdout.readline, ''):
                        output += line + "\r\n"
                        if not silent_mode:
                            for l in line.rstrip("\r\n").replace("\r", "").split("\n"):
                                self.__logger.printLog("INFO", l)
                except Exception as e:
                    self.__logger.printLog("WARNING", "*** EXCEPTION ! *** ({0})".format(e))
        except KeyboardInterrupt:
            self.__logger.printLog("WARNING", "*** Interrupted ! ***")
            os.killpg(p.pid, signal.SIGTERM)
        # Check returned code
        t.cancel()
        code = p.returncode
        if code != 0:
            # print code
            self.__logger.printLog("WARNING", "*** COMMAND FAILED!", frontContent=log)
        return code, output.rstrip("\r\n")

    # def __internalCmdExec(self, command, timeout, silent_mode=False):
    #     run_job = self.OsAgnosticSubprocess(command_line=command,
    #                                         logger=self.__logger,
    #                                         silent_mode=silent_mode)
    #
    #     result, output = run_job.executeCommand(timeout)
    #     return result, output

    def __runCmdExec(self, command, timeout, silent_mode):
        if self.__isExternal:
            return self.__globalConf["EXTERNAL_LOCAL_EXEC"](command, timeout, silent_mode=silent_mode)
        else:
            return self.__internalCmdExec(command, timeout, silent_mode=silent_mode)

    # method called for execution on host
    def commandExec(self, command, timeout=3, silent_mode=False):
        return self.__runCmdExec(command, timeout, silent_mode=silent_mode)

    def commandExecAdb(self, command, timeout=3, silent_mode=False):
        runCommand = "adb " + self.ssnExtra + command
        return self.__runCmdExec(runCommand, timeout, silent_mode=silent_mode)

    def commandExecFastboot(self, command, timeout=3, silent_mode=False):
        runCommand = "fastboot " + self.fastbootSsnExtra + command
        return self.__runCmdExec(runCommand, timeout, silent_mode=silent_mode)

    def commandExecCflasher(self, command, timeout=300):
        if not self.is_credential:
            self.__logger.printLog("WARNING", "commandExecCflasher(): download not possible as credentials not initialized")
            return -1, ""
        timeout_minutes = int(timeout / 60) + 1
        if (timeout_minutes < 25):
            timeout_minutes = 30
            self.__logger.printLog("WARNING", "commandExecCflasher(): CLI Timeout less than 25 minutes, setting it to 30 minutes.")

        if (int(timeout / 60) < 25):
            timeout = 1800
            self.__logger.printLog("WARNING", "commandExecCflasher(): Timeout less than 25 minutes, setting it to 30 minutes.")

        full_command = "cflasher " + command + " --timeout {0}".format(timeout_minutes)
        self.__logger.printLog("INFO", "*** RUN: " + full_command + " --auth {0}, timeout={1}s".format(self.__print_creds(), timeout+70))
        exec_status, output = self.__runCmdExec(full_command + " --auth " + self.__credentials, timeout+70, silent_mode=True)
        formatted_output = list()
        for line in output.split("\n"):
            if not line.isspace():
                formatted_output.append(line)
        self.__logger.printLog("INFO", "\n" + "\n".join(formatted_output))
        return exec_status, output

    def commandExecArtifactorySecureDownload(self, link, destination, timeout=300, file_name=""):
        if not self.is_credential:
            self.__logger.printLog("WARNING", "commandExecArtifactorySecureDownload(): download not possible as credentials not initialized")
            return -1, ""
        if file_name:
            destination_file = os.path.join(destination, file_name)
        else:
            destination_file = os.path.join(destination, os.path.basename(link))
        if os.path.isfile(destination_file):
            os.remove(os.path.join(destination_file))
        request=urllib2.Request(link)
        base64string = base64.encodestring(self.__credentials)
        request.add_header("Authorization", "Basic {0}".format(base64string).replace("\n", ""))
        self.__logger.printLog("INFO", "downloading '{0}' to {1}".format(link, destination_file))
        try:
            f = urllib2.urlopen(request)
            with open(destination_file, "wb") as local_file:
                local_file.write(f.read())
            output = urllib2.urlopen(request).read()
            exec_status = 0
        except Exception as e:
            output = ""
            exec_status = -1
            self.__logger.printLog("WARNING", "invalid url: {0} ({1})".format(link, e))
        return exec_status, output

    def commandExecArtifactorySecureContent(self, url):
        if not self.is_credential:
            self.__logger.printLog("WARNING", "commandExecArtifactorySecureContent(): download not possible as credentials not initialized")
            return -1, ""
        request=urllib2.Request(url)
        base64string = base64.encodestring(self.__credentials)
        request.add_header("Authorization", "Basic {0}".format(base64string).replace("\n", ""))
        self.__logger.printLog("INFO", "getting secured artifactory data from: {0}".format(url))
        try:
            output = urllib2.urlopen(request).read()
            exec_status = 0
        except Exception as e:
            output = ""
            exec_status = -1
            self.__logger.printLog("WARNING", "invalid url: {0} ({1})".format(url, e))
        return exec_status, output

    def commandExecExternalFotaSecureContent(self, url, sh):
        # force erasing of proxy setting
        previous_no_proxy_settings = os.environ.get("no_proxy", "")
        previous_http_proxy_settings = os.environ.get("http_proxy", "")
        previous_https_proxy_settings = os.environ.get("https_proxy", "")
        if sh == 0:
            os.environ["no_proxy"] = ""
            os.environ["http_proxy"] = "https://proxy-ir.intel.com:911/"
            os.environ["https_proxy"] = "https://proxy-ir.intel.com:911/"
        else:
            os.environ["no_proxy"] = ""
            os.environ["http_proxy"] = "https://child-prc.intel.com:913/"
            os.environ["https_proxy"] = "https://child-prc.intel.com:913/"

        request = urllib2.Request(url)
        # create a password manager
        password_mgr = urllib2.HTTPPasswordMgrWithDefaultRealm()
        password_mgr.add_password(None, url, "imeitest", "0000")
        handler = urllib2.HTTPBasicAuthHandler(password_mgr)
        opener = urllib2.build_opener(handler)
        urllib2.install_opener(opener)
        self.__logger.printLog("INFO", "getting secured external fota data from: {0}".format(url))
        try:
            output = urllib2.urlopen(request, timeout=30).read()
            exec_status = 0
        except Exception as e:
            output = ""
            exec_status = -1
            self.__logger.printLog("WARNING", "invalid url or failure to connect: {0} ({1})".format(url, e))
        # restore proxy settings
        os.environ["no_proxy"] = previous_no_proxy_settings
        os.environ["http_proxy"] = previous_http_proxy_settings
        os.environ["https_proxy"] = previous_https_proxy_settings
        return exec_status, output

    def commandExecExternalFotaSecureDownload(self, sh, link, destination, timeout=1200):
        self.__logger.printLog("INFO", "downloading '{0}' to: {1}".format(link, destination))
        # force erasing of proxy setting
        previous_no_proxy_settings = os.environ.get("no_proxy", "")
        previous_http_proxy_settings = os.environ.get("http_proxy", "")
        previous_https_proxy_settings = os.environ.get("https_proxy", "")
        if sh == 0:
            os.environ["no_proxy"] = ""
            os.environ["http_proxy"] = "https://proxy-ir.intel.com:911/"
            os.environ["https_proxy"] = "https://proxy-ir.intel.com:911/"
        else:
            os.environ["no_proxy"] = ""
            os.environ["http_proxy"] = "https://child-prc.intel.com:913/"
            os.environ["https_proxy"] = "https://child-prc.intel.com:913/"

        full_command = "wget --no-verbose --http-user=imeitest --http-password=0000 --no-check-certificate {0} -O {1}".format(link, os.path.join(destination, os.path.basename(link)))
        exec_status, output = self.__runCmdExec(full_command, timeout, silent_mode=False)
        # restore proxy settings
        os.environ["no_proxy"] = previous_no_proxy_settings
        os.environ["http_proxy"] = previous_http_proxy_settings
        os.environ["https_proxy"] = previous_https_proxy_settings
        return exec_status, output

    def commandExecActivityScan(self, command, timeout=3, silent_mode=False, scan_inactivity_timeout=60):
        log = "commandExecFileActivityScan():"
        if not silent_mode:
            self.__logger.printLog("INFO", "*** RUN: {0} (timeout={1}s)".format(command, timeout))

        # timeout inner method
        def timeout_end():
            self.__logger.printLog("WARNING", "*** TIMEOUT ! ***")
            os.killpg(p.pid, signal.SIGTERM)
            t2.cancel()
            return 1, ""

        def scan_end():
            self.__logger.printLog("WARNING", "*** SCAN TIMEOUT ! ***")
            os.killpg(p.pid, signal.SIGTERM)
            t.cancel()
            return 2, ""

        # create signal alarm
        t = Timer(timeout, timeout_end)
        t.start()
        t2 = Timer(scan_inactivity_timeout, scan_end)
        t2.start()
        p = subprocess.Popen(command.split(), stdout=subprocess.PIPE, stderr=subprocess.STDOUT, preexec_fn=os.setsid)
        output = ""
        try:
            while p.poll() is None:
                try:
                    for line in iter(p.stdout.readline, ''):
                        output += line + "\r\n"
                        # if output in scan file, restart timeout to avoid expiration
                        t2.cancel()
                        t2 = Timer(scan_inactivity_timeout, scan_end)
                        t2.start()
                        if not silent_mode:
                            for l in line.rstrip("\r\n").replace("\r", "").split("\n"):
                                self.__logger.printLog("INFO", l)
                except Exception as e:
                    self.__logger.printLog("WARNING", "*** EXCEPTION ! *** ({0})".format(e))
        except KeyboardInterrupt:
            self.__logger.printLog("WARNING", "*** Interrupted ! ***")
            os.killpg(p.pid, signal.SIGTERM)
        # Check returned code
        t.cancel()
        t2.cancel()
        code = p.returncode
        if code != 0:
            self.__logger.printLog("WARNING", "*** COMMAND FAILED ! ***", frontContent=log)
        return code, output.rstrip("\r\n")
    #
    # class OsAgnosticSubprocess(object):
    #     def __new__(cls, *args, **kwargs):
    #         """
    #         Constructor that will return proper implementation based on the OS
    #         """
    #         if "posix" in sys.builtin_module_names:
    #             # Load linux subprocess execution module
    #             subprocess_engine = HostModule.LinuxSubprocess
    #         else:
    #             # Load windows subprocess execution module
    #             subprocess_engine = HostModule.WindowsSubprocess
    #
    #         return subprocess_engine(*args, **kwargs)
    #
    # def enqueue_output(out, queue):
    #     """
    #     Local function that will consume stdout stream and put the content in a queue
    #
    #     :type   out: pipe
    #     :param  out: stdout stream
    #
    #     :type   queue: Queue
    #     :param  queue: queue where each stdout line will be inserted
    #     """
    #     for line in iter(out.readline, ''):
    #         queue.put(line)
    #     out.close()
    #
    # class SubprocessHandler(object):
    #
    #     def __init__(self, command_line, logger, silent_mode, max_empty_log_time=120):
    #         """
    #         Class that will execute a command regardless the Host OS and return the result message
    #
    #         :type  command_line: str
    #         :param command_line: Command to be run
    #
    #         :type  logger: logger object
    #         :param logger: logger to be used to log messages
    #
    #         :type  silent_mode: bool
    #         :param silent_mode: display logs in the logger
    #
    #         :type  max_empty_log_time: int
    #         :param max_empty_log_time: max delay w/o log, after this delay a message will be displayed
    #         """
    #         self._command_line = command_line
    #         self._original_command_line = command_line
    #         self._logger = logger
    #         self._silent_mode = silent_mode
    #
    #         self._my_process = None
    #         self._log_level = None
    #         self._last_log_time = None
    #         self._stdout_data = Queue()
    #         self._max_empty_log_time = max_empty_log_time
    #         self._readable = []
    #
    #         self.format_cmd()
    #
    #     @property
    #     def _stdout_iter(self):
    #         """
    #         Create iterator based on the stdout queue content
    #         """
    #         while True:
    #             try:
    #                 yield self._stdout_data.get_nowait()
    #             except Empty:
    #                 break
    #
    #     @property
    #     def command(self):
    #         """
    #         :return:    Properly formatted command to execute
    #         :rtype:     str
    #         """
    #         return self._command_line
    #
    #     def format_cmd(self):
    #         """
    #         Format command to be executed
    #         """
    #         # For retrocompatibilty we do not do anything if it is a list
    #         if not isinstance(self._command_line, list):
    #             self._command_line = str(self._command_line).encode('ascii', 'ignore')
    #
    #     def _safe_kill(self):
    #         """
    #         Kill process and subprocess if psutil is available, else do a simple process kill
    #         """
    #         if self._my_process:
    #             try:
    #                 import psutil
    #
    #                 main_proc = psutil.Process(self._my_process.pid)
    #                 try:
    #                     # psutil version > v0.4.1
    #                     procs_to_rip = main_proc.get_children(recursive=True)
    #                 except TypeError:
    #                     # psutil version <= v0.4.1
    #                     procs_to_rip = main_proc.get_children()
    #
    #                 procs_to_rip.append(main_proc)
    #
    #                 for proc_to_kill in procs_to_rip:
    #                     if psutil.pid_exists(proc_to_kill.pid) and proc_to_kill.is_running():
    #                         try:
    #                             proc_to_kill.terminate()
    #                         except psutil.NoSuchProcess:
    #                             continue
    #
    #                 _, proc_still_alive = psutil.wait_procs(procs_to_rip, timeout=1)
    #                 for proc_to_atom in proc_still_alive:
    #                     if psutil.pid_exists(proc_to_atom.pid) and proc_to_atom.is_running():
    #                         try:
    #                             proc_to_atom.kill()
    #                         except psutil.NoSuchProcess:
    #                             continue
    #             except Exception:
    #                 # Something wrong occurs with psutil
    #                 # Stop the process as usual
    #                 self._my_process.kill()
    #
    #     def _check_io(self, read_to_eof=False):
    #         # implemented at OS specific level
    #         pass
    #
    #     def _finalize(self, execution_time):
    #         """
    #         Finalize process operation
    #
    #         :type  execution_time: float
    #         :param execution_time: execution time of the command
    #
    #         :return: return True if the process was properly ended
    #         :rtype: bool
    #         """
    #         result = -1
    #         if self._my_process:
    #             poll_value = self._my_process.poll()
    #             if poll_value is not None:
    #                 # Read latest data
    #                 self._check_io(True)
    #                 # Process created by Popen is terminated by system
    #                 result = poll_value
    #                 if poll_value == 0 and not self._silent_mode:
    #                     self._logger.printLog(
    #                             "DEBUG",
    #                             "Command executed in {0}".format(datetime.timedelta(seconds=execution_time)),
    #                             raw=True)
    #                 elif not self._silent_mode:
    #                     # Note: A negative value -N indicates that the child
    #                     # was terminated by signal N (Unix only).
    #                     self._logger.printLog("ERROR", "*** COMMAND FAILED!", raw=True)
    #             else:
    #                 # Process was not terminated until timeout or cancel
    #                 try:
    #                     self._safe_kill()
    #                     # Read latest data
    #                     self._check_io(True)
    #                 except OSError:
    #                     pass
    #
    #                 if not self._silent_mode:
    #                     self._logger.printLog("ERROR", "*** TIMEOUT!", raw=True)
    #         return result
    #
    #     def _start_process(self):
    #         pass
    #
    #     def _init__readable(self):
    #         pass
    #
    #     def executeCommand(self, timeout):
    #         """
    #         Launch synchronized execution
    #
    #         :type  timeout: int
    #         :param timeout: command execution timeout in sec
    #
    #         :return: Execution status & output str (and optionally C{dict})
    #         :rtype: tuple(bool & str)
    #
    #         """
    #
    #         if not self._silent_mode:
    #             self._logger.printLog(
    #                     "INFO",
    #                     "*** RUN: {0} (timeout={1})".format(self._original_command_line,
    #                                                         datetime.timedelta(seconds=timeout)),
    #                     raw=True)
    #         try:
    #             # set the begin time for information about duration (printed on stdout)
    #             begin_time = time.time()
    #             end_time = begin_time + float(timeout)
    #             self._start_process()
    #             self._init__readable()
    #
    #             # retain the previous time for empty output duration
    #             self._last_log_time = begin_time
    #
    #             exec_time = time.time()
    #             while exec_time < end_time and self._my_process.poll() is None:
    #                 # if no output for x seconds, print an info
    #                 if int(time.time() - self._last_log_time) >= self._max_empty_log_time:
    #                     self._logger.printLog(
    #                             "INFO",
    #                             "Command running for {0}".format(
    #                                     datetime.timedelta(seconds=int(time.time() - begin_time))),
    #                             raw=True)
    #                     self._last_log_time = time.time()
    #
    #                 self._check_io()
    #                 exec_time = time.time()
    #
    #             # cleanup operations
    #             process_result = self._finalize(exec_time - begin_time)
    #
    #         except KeyboardInterrupt:
    #             self._logger.printLog("WARNING", "Command interruption!", raw=True)
    #             raise KeyboardInterrupt
    #
    #         finally:
    #             self._my_process = None
    #
    #         return process_result, "\n".join(self._stdout_iter)
    #
    # class LinuxSubprocess(SubprocessHandler):
    #
    #     def __init__(self, command_line, logger, silent_mode=False,
    #                  max_empty_log_time=120):
    #         """
    #         Class that will execute a command regardless the Host OS and return the result message
    #         :type  command_line: str
    #         :param command_line: Command to be run
    #         :type  logger: logger object
    #         :param logger: logger to be used to log messages
    #         :type  silent_mode: bool
    #         :param silent_mode: display logs in the logger
    #         :type  max_empty_log_time: int
    #         :param max_empty_log_time: max delay w/o log, after this delay a message will be displayed
    #         """
    #         super(HostModule.LinuxSubprocess, self).__init__(command_line, logger, silent_mode,
    #                                                          max_empty_log_time)
    #
    #     def format_cmd(self):
    #         """
    #         Format command to be executed
    #         """
    #         super(HostModule.LinuxSubprocess, self).format_cmd()
    #         # For retrocompatibilty we do not do anything if it is a list
    #         if not isinstance(self._command_line, list):
    #             self._command_line = shlex.split(self._command_line)
    #
    #     def _check_io(self, read_to_eof=False):
    #         """
    #         Method that will query stdout and stderr streams, log them if needed, and store them in internal queue
    #         :type  read_to_eof: bool
    #         :param read_to_eof: read to the end of stdout/stderr stream
    #         """
    #         if self._my_process and self._log_level:
    #             ready_to_read, _, _ = select.select(self._readable, [], [], 0)
    #             # While we have something to read
    #             while ready_to_read:
    #                 # Read it
    #                 for io in ready_to_read:
    #                     io_lines = io.read_lines()
    #                     if io_lines is None:
    #                         self._readable.remove(io)
    #                     else:
    #                         for io_line in io_lines:
    #                             if not self._silent_mode:
    #                                 self._logger.printLog(self._log_level[io], io_line, raw=True)
    #                             self._last_log_time = time.time()
    #                             self._stdout_data.put_nowait(io_line.rstrip("\r"))
    #                 if read_to_eof:
    #                     # Check if new data are available
    #                     ready_to_read, _, _ = select.select(self._readable, [], [], 0)
    #                 else:
    #                     # Stop reading
    #                     ready_to_read = None
    #
    #     def _init__readable(self):
    #         """
    #         Method that will init stdout and stderr stream and bind associate log level
    #         """
    #         # Create line reader for each stream
    #         proc_stdout = self.LinuxSubprocessIoHandler(self._my_process.stdout.fileno())
    #         proc_stderr = self.LinuxSubprocessIoHandler(self._my_process.stderr.fileno())
    #
    #         # Create readable collection for select usage
    #         self._readable = [proc_stdout, proc_stderr]
    #
    #         # Associate proper log level for each stream
    #         self._log_level = {proc_stdout: "INFO",
    #                            proc_stderr: "ERROR"}
    #
    #     def _start_process(self):
    #         """
    #         Initialize subprocess execution
    #         """
    #         try:
    #             # Clear the queue (in case of second run)
    #             self._stdout_data = Queue()
    #
    #             # Start the process
    #             self._my_process = subproc.Popen(self._command_line, shell=False,
    #                                              stdout=subproc.PIPE, stderr=subproc.PIPE,
    #                                              close_fds=True, universal_newlines=True)
    #         except OSError:
    #             if not self._silent_mode:
    #                 self._logger.printLog(
    #                         "ERROR",
    #                         "OSError, invalid paths in: {0}".format(self._command_line), raw=True)
    #             raise OSError
    #
    #     class LinuxSubprocessIoHandler(object):
    #
    #         """
    #         LineReader object to be use as select readable input
    #         """
    #
    #         # Size of the buffer (in bytes) to be read
    #         READ_BUFFER_SIZE = 8192
    #
    #         def __init__(self, fd):
    #             """
    #             :type fd: int
    #             :param fd: stream file descriptor
    #             """
    #             # File descriptor where read operation will be done
    #             self._fd = fd
    #             # Internal buffer used to store incomplete line
    #             self._buf = ""
    #
    #         def fileno(self):
    #             """
    #             Method for select operator.
    #             DO NOT CHANGE method name / type
    #             :return: File descriptor
    #             """
    #             return self._fd
    #
    #         def read_lines(self):
    #             """
    #             Method to return lines from data read from file descriptor.
    #             Complete line are returned, incomplete one are buffered
    #             :return: Array of complete line
    #             """
    #             lines = []
    #
    #             # Read data from file descriptor
    #             data = os.read(self._fd, self.READ_BUFFER_SIZE)
    #
    #             if data:
    #                 # Append and store read data with previous incomplete line
    #                 self._buf += data
    #
    #                 if "\n" in data:
    #                     # Line(s) is/are ready to be split
    #                     # Split all data
    #                     tmp = self._buf.split('\n')
    #
    #                     # Extract complete lines and incomplete one
    #                     lines, self._buf = tmp[:-1], tmp[-1]
    #             elif self._buf:
    #                 # No more data, return latest data we put in our internal buffer
    #                 lines = [self._buf]
    #                 self._buf = ""
    #             else:
    #                 # No more data at all, return None
    #                 lines = None
    #
    #             return lines
    #
    # class WindowsSubprocess(SubprocessHandler):
    #
    #     def __init__(self, command_line, logger, silent_mode=False,
    #                  max_empty_log_time=120):
    #         """
    #         Class that will execute a command regardless the Host OS and return the result message
    #
    #         :type  command_line: str
    #         :param command_line: Command to be run
    #
    #         :type  logger: logger object
    #         :param logger: logger to be used to log messages
    #
    #         :type  silent_mode: bool
    #         :param silent_mode: display logs in the logger
    #
    #         :type  max_empty_log_time: int
    #         :param max_empty_log_time: max delay w/o log, after this delay a message will be displayed
    #         """
    #         super(HostModule.WindowsSubprocess, self).__init__(command_line, logger, silent_mode,
    #                                                            max_empty_log_time)
    #
    #     def _check_io(self, read_to_eof=False):
    #         """
    #         Method that will query stdout and stderr streams, log them if needed, and store them in internal queue
    #
    #         :type  read_to_eof: bool
    #         :param read_to_eof: read to the end of stdout/stderr stream
    #         """
    #         if self._my_process:
    #             do_read = True
    #             while do_read:
    #                 for io_name, io_ctx in self._readable.items():
    #                     io_lines = io_ctx["stream"].read_lines()
    #                     if io_lines is None:
    #                         io_ctx["stream"].stop_read()
    #                         del self._readable[io_name]
    #                     else:
    #                         for io_line in io_lines:
    #                             if not self._silent_mode:
    #                                 self._logger.printLog(io_ctx["log_level"], io_line.rstrip("\n"), raw=True)
    #                             self._last_log_time = time.time()
    #                             self._stdout_data.put_nowait(io_line.rstrip("\r"))
    #
    #                 if read_to_eof and self._readable:
    #                     # Still some readable and we request us to read to the eof
    #                     do_read = True
    #                 else:
    #                     # Stop reading
    #                     do_read = False
    #
    #     def _init_readable(self):
    #         """
    #         Method that will init stdout and stderr stream and bind associate log level
    #         """
    #         if self._my_process:
    #             # Create readable collection for select usage
    #             self._readable = {"stdout": {"stream": self.WindowsSubprocessIoHandler("stdout",
    #                                                                                    self._my_process,
    #                                                                                    self._my_process.stdout),
    #                                          "log_level": "INFO"},
    #                               "stderr": {"stream": self.WindowsSubprocessIoHandler("stderr",
    #                                                                                    self._my_process,
    #                                                                                    self._my_process.stderr),
    #                                          "log_level": "ERROR"}}
    #
    #     def _start_process(self):
    #         """
    #         Initialize subprocess execution
    #         """
    #         try:
    #             # Clear the queue (in case of second run)
    #             self._stdout_data = Queue()
    #
    #             # Start the process
    #             self._my_process = subproc.Popen(self._command_line, shell=False,
    #                                              stdout=subproc.PIPE, stderr=subproc.PIPE,
    #                                              bufsize=1, universal_newlines=True)
    #         except OSError:
    #             if not self._silent_mode:
    #                 self._logger.printLog("ERROR",
    #                                       "OSError, invalid paths in: {0}".format(self._command_line), raw=True)
    #             raise OSError
    #
    #     class WindowsSubprocessIoHandler(object):
    #
    #         """
    #         LineReader object to be use as select readable input
    #         """
    #         MAX_LINES_TO_READ = 500
    #
    #         def __init__(self, name, process, stream):
    #             """
    #             :type   name: str
    #             :param  name: name of the process line reader object
    #
    #             :type   process: process
    #             :param  process: process to read from
    #
    #             :type   stream: int
    #             :param  stream: stream file descriptor
    #             """
    #
    #             # Nice name to ease debugging between different ProcessLineReaderWin
    #             # We usually have 2: one for stdout, one for stderr
    #             self.name = name
    #
    #             # Process to work on
    #             self._process = process
    #
    #             # Stream to read data from
    #             self._stream = stream
    #
    #             # Reader thread
    #             self._reader_thread = None
    #
    #             # Internal buffer
    #             self._lines_queue = Queue()
    #
    #             # Start reading
    #             self.start_read()
    #
    #         def _read_worker(self):
    #             """
    #             Reader thread that will retrieve data from the stream
    #             """
    #             for line in iter(self._stream.readline, ''):
    #                 line = line.rstrip("\n")
    #                 if line:
    #                     self._lines_queue.put(line)
    #
    #             self._stream.close()
    #
    #         def stop_read(self):
    #             """
    #             Notify the line reader that we want to stop reading
    #             """
    #             pass
    #
    #         def start_read(self):
    #             """
    #             Notify the line reader that we want to start reading
    #             A new reader thread will be started to retrieve data from the stream
    #             """
    #             self._reader_thread = Thread(target=self._read_worker)
    #             self._reader_thread.name = "{0} process reader line thread".format(self.name)
    #             self._reader_thread.daemon = True
    #             self._reader_thread.start()
    #
    #         def read_lines(self):
    #             """
    #             Method to return lines from data read from file descriptor.
    #             Complete line are returned
    #
    #             :return: Array of complete line
    #             """
    #             lines = []
    #             process_terminated = self._process.poll() is not None
    #             try:
    #                 data = self._lines_queue.get_nowait()
    #                 for line in data.splitlines():
    #                     lines.append(line)
    #             except Empty:
    #                 pass
    #
    #             if not lines and process_terminated:
    #                 # Nothing more to read
    #                 lines = None
    #
    #             return lines
