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
@summary: Pupdr Library - WatchdogModule
@since: 11/17/2014
@author: travenex
"""

import re
import os
import time
import OutputModule
import HostModule
import LoggerModule
import DeviceModule
import OsManagerModule
import WorkaroundModule
import LogsModule
import MiscModule
import ConfigurationModule

class WatchdogModule(object):

    __instance = None
    __crashlogs = None
    __historyEvent = None
    force_ramdump_removal = None
    __globalConf = None
    __logger = None
    __host = None
    __output = None
    __device = None
    __osManager = None
    __misc = None
    __workaround = None
    __configuration = None
    __logs = None
    wd_handler = None

    def __new__(cls):
        """
        Constructor that always returns the same instance
        """
        if not cls.__instance:
            cls.__instance = object.__new__(cls)
        return cls.__instance

    def init(self, globalConf):
        self.__globalConf = globalConf
        self.__logger = LoggerModule.LoggerModule()
        self.__host = HostModule.HostModule()
        self.__output = OutputModule.OutputModule()
        self.__device = DeviceModule.DeviceModule()
        self.__osManager = OsManagerModule.OsManagerModule()
        self.__misc = MiscModule.MiscModule()
        self.__workaround = WorkaroundModule.WorkaroundModule()
        self.__configuration = ConfigurationModule.ConfigurationModule()
        self.__logs = LogsModule.LogsModule()
        self.force_ramdump_removal = False
        self.wd_handler = self.HistoryEventHandler(self.__globalConf)

    class HistoryEventHandler():
        def __init__(self, conf):
            self.__globalConf = conf
            self.__historyEvent = ""
            self.__initial_content = []
            self.__delta = ""
            self.__crashlogs = None
            self.__configuration = ConfigurationModule.ConfigurationModule()
            self.__output = OutputModule.OutputModule()
            self.__logger = LoggerModule.LoggerModule()
            self.__host = HostModule.HostModule()
            self.__device = DeviceModule.DeviceModule()
            self.__workaround = WorkaroundModule.WorkaroundModule()
            self.__logs = LogsModule.LogsModule()
            self.__misc = MiscModule.MiscModule()

        def start(self):
            self.__historyEvent = "/".join((self.__configuration.logs.LOG_PATH, "history_event"))
            if self.__device.waitAdbDevices(10):
                status, self.__initial_content = self.__getHistoryEvent()
            else:
                status = False
                self.__initial_content = []
            if not status and self.__configuration.logs.CRASHLOGS:
                self.__logger.printLog("WARNING", "cannot get data from: " + self.__historyEvent)
                self.__initial_content = []
            # delta between before and after
            self.__delta = ""
            self.__output.appendOutput("", None)

        def __getHistoryEvent(self):
            """ get the content line count of /logs/history_event
            """
            log = "getHistoryEvent(): "
            verdict = True
            output = ""
            returnOutput = []
            self.__logger.printLog("INFO", log + "read")
            if not self.__logs.bootota_log_path:
                output = "no tc report path to pull file to"
                verdict = False
            else:
                pulled_file = os.path.join(self.__logs.bootota_log_path, os.path.basename(self.__historyEvent))
                if os.path.isfile(pulled_file):
                    os.remove(pulled_file)
                self.__misc.local_files_handler.addEntry(pulled_file)
                self.__logs.pull(self.__historyEvent, self.__logs.bootota_log_path, push_to_server=False)
                if not os.path.exists(pulled_file):
                    output =  "failure to pull file {0}".format(self.__historyEvent)
                    verdict = False
                else:
                    with open(pulled_file) as f:
                        returnOutput = f.readlines()
                    os.remove(pulled_file)
            self.__output.appendOutput(output, verdict)
            return verdict, returnOutput

        def __getCrashlogsInHistoryEvent(self, pattern_list=None):
            """ Method to get list of pattern in history_event.
                It can return the same list than pattern_list if the order matches.
            """
            verdict = True
            output = ""
            self.__crashlogs = list()

            if not self.__logs.bootota_log_path:
                self.__output.appendOutput("no tc report path to pull file to", False)
                return False, ""

            # Read history_event
            pulled_file = os.path.join(self.__logs.bootota_log_path, os.path.basename(self.__historyEvent))
            if os.path.isfile(pulled_file):
                os.remove(pulled_file)
            self.__misc.local_files_handler.addEntry(pulled_file)
            self.__logs.pull(self.__historyEvent, self.__logs.bootota_log_path, push_to_server=False)
            if not os.path.exists(pulled_file):
                returnOutput =  "failure to pull file {0}".format(self.__historyEvent)
                verdict = False
                self.__logger.printLog("WARNING", returnOutput)
            else:
                with open(pulled_file) as f:
                    history_event_data = f.readlines()
                os.remove(pulled_file)
                delta = []
                for line in history_event_data:
                    if line in self.__initial_content:
                        delta = []
                    else:
                        delta.append(line)

                self.__delta = "\n".join(delta)
                self.__logger.printLog("INFO", "getCrashlogsInHistoryEvent(): delta in history event:\n{0}".format(self.__delta))
                if pattern_list:
                    self.__logger.printLog("INFO", "search data: {0}".format(pattern_list))
                    returnOutput = re.findall("|".join(pattern_list), self.__delta)
                    self.__logger.printLog("INFO", "data found: {0}".format(returnOutput))
                else:
                    pattern_list = [".*"]
                    returnOutput = None
                # If nothing added
                if not self.__delta:
                    returnOutput = "no data added into " + self.__historyEvent
                    self.__logger.printLog("WARNING", returnOutput)
                    verdict = False
                # Fill crashlogs path list
                for c in pattern_list:
                    p = re.findall(c + ".* (/?.*/logs/crashlog[0-9a-z_]*)", self.__delta)
                    self.__logger.printLog("INFO", "crashlogs: " + str(p))
                    self.__crashlogs = p

            self.__output.appendOutput(output, verdict)
            return verdict, returnOutput

        @property
        def crashlogs(self):
            """ Method to get list of crashlogs paths history_event.
            """
            verdict = self.__getCrashlogsInHistoryEvent()[0]
            self.__output.appendOutput("", verdict)
            if verdict and self.__crashlogs:
                return self.__crashlogs
            else:
                return []

        def __checkCrashlogs(self, num):
            """ Check presence of ipanic_emmc files in crashlog
            """
            returnOutput  = ""
            verdict = True
            output = ""

            # List of expected files
            files = ["emmc_ipanic_console*",
                     "emmc_ipanic_header*",
                     "emmc_ipanic_thread*",
                     "*ramoops*",
                     "crashfile*"]

            # If nothing found (So at least one crashlog path)
            if not self.__crashlogs:
                returnOutput = "no crashlog path found in '{0}'".format(self.__historyEvent)
                output = returnOutput
                verdict = False

            # Else search crashlog paths
            else:
                if len(self.__crashlogs) != num:
                    returnOutput = "expecting {0} crashlogs, {1} found".format(str(num), str(len(self.__crashlogs)))
                    output = returnOutput
                    verdict = False
                else:
                    # Check presence of files
                    missing = list()
                    for c in self.__crashlogs:
                        for f in files:
                            if not self.__device.checkPresence(c + "/" + f):
                                missing.append(c + "/" + f)
                    # If missing list exists
                    if missing:
                        returnOutput = "missing files ({0})".join(", ".join(missing))
                        output = returnOutput
                        verdict = False
            # If verdict is SUCCESS then output is an empty string
            self.__output.appendOutput(output, verdict)
            return verdict, returnOutput

        def __checkWdInHistoryEvent(self, wd_type, num, emmc_ipanic=True):
            """ Check that pattern_list matches with data added in history_event.
                If emmc_ipanic is True then check that the crashlog paths found in history_event contain
                the three files emmc_ipanic ("console", "header", "thread")
            """
            output = ""
            pattern_total = num * {"IPANIC": ["IPANIC", "SWWDT_RESET"],
                                   "FABRIC": ["FABRIC", "HWWDT_RESET"]}[wd_type]
            # Must be executed before __checkCrashlogs (for delta creation)
            verdict, returnOutput = self.__getCrashlogsInHistoryEvent(pattern_total)
            if verdict:
                if returnOutput == pattern_total:
                    returnOutput = ""
                    if emmc_ipanic:
                        verdict, output = self.__checkCrashlogs(num)
                else:
                    returnOutput = "{0} failure: actual is {1}, expected is {2}".format(self.__historyEvent, str(returnOutput), str(pattern_total))
                    output = returnOutput
                    verdict = False

            if not verdict and self.__workaround.isWorkaround(data_type=self.__workaround.Function, output=returnOutput)[0]:
                verdict = True
            self.__output.appendOutput(output, verdict)
            return verdict, returnOutput

        def check_SW_WD_in_history_event(self, num=1, emmc_ipanic=True):
            """ Check SW WD in history_event
            """
            verdict, returnOutput = self.__checkWdInHistoryEvent("IPANIC", num, emmc_ipanic)
            output = ""
            if not verdict :
                output = "check has failed"
            self.__output.appendOutput(output, verdict)
            return verdict, returnOutput

        def check_HW_WD_in_history_event(self, num=1, emmc_ipanic=True):
            """ Check hW WD in history_event
            """
            verdict, returnOutput =  self.__checkWdInHistoryEvent("FABRIC", num, emmc_ipanic)
            output = ""
            if not verdict :
                output = "check has failed"
            self.__output.appendOutput(output, verdict)
            return verdict, returnOutput

    def __getDebugfs(self):
        """ get the debugfs partition path on the phone.
        """
        output = ""
        verdict = True
        returnOutput = False
        # Set rights as root
        if not self.__device.adbRoot():
            output = "failure to root device"
            verdict = False
        else:
            mount = self.__host.commandExecAdb("shell cat /proc/mounts")[1]
            search = re.search("/[^ ]*debug", mount)
            if search:
                returnOutput = search.group()
            else:
                output = "no path found for debugfs"
                verdict = False
        self.__output.appendOutput(output, verdict)
        return returnOutput

    def __triggerWd(self, trigger):
        """ Generate a watchdog and throw a failure or success depending on the result.
        """
        log = "triggerWd(): "
        output = ""
        returnOutput = "0"
        verdict = True
        # Check that the trigger WD file is present
        local_verdict = self.__getDebugfs()
        trigger_content = trigger.split(":")
        if len(trigger_content) == 2:
            var_trigger = trigger_content[1]
        else:
            var_trigger = "1"
        trigger = trigger.split(":")[0]
        if not local_verdict and not trigger.startswith("/"):
            output = "debugfs not found"
            ll = ""
            verdict = False
        else:
            # do not use os.path.join to avoid issues on windows
            if not trigger.startswith("/"):
                trigger = local_verdict + "/" + trigger
            ll = self.__host.commandExecAdb("shell ls -l {}".format(trigger))[1]
        if verdict and "No such file or directory" in ll:
            output = "trigger not found ({0})".format(ll)
            verdict = False
        elif verdict:
            # Wait that rights become root
            timeout = 10
            start = time.time()
            while True:
                idRights = " ".join(self.__host.commandExecAdb("shell id")[1].split(" ")[:2])
                if "root" in idRights:
                    bootCompleted = self.__device.getProperty("sys.boot_completed")
                    if bootCompleted != "1":
                        bootCompleted = "0"
                    self.__logger.printLog("INFO", log + "sys.boot_completed={0}".format(bootCompleted))
                    returnOutput = bootCompleted
                    break
                if time.time() > (start + timeout):
                    output = "timeout {0} seconds".format(timeout)
                    verdict = False
            if verdict:
                # Trigger the WD
                self.__host.commandExecAdb("shell echo '{}' > {}".format(var_trigger, trigger), 10)
                # WD has been successfully triggered if device is frozen or already rebooting
                expectedOs = ("frozen", "offline", "dnx")
                if self.__osManager.getOs() in expectedOs:
                    self.__logger.printLog("INFO", log + "triggered successfully")
                else:
                    self.__logger.printLog("DEBUG", log + "device not in: {0}".format(", ".join(expectedOs)))
                verdict = self.__osManager.waitOs("offline", timeout=self.__configuration.timeout.WATCHDOG_EXPIRATION)
                if not verdict:
                    output = "Board did not go offline after triggerWD ({0})".format(trigger)

        if not verdict:
            returnOutput = False
        self.__output.appendOutput(output, verdict, argument="trigger={0}".format(trigger))
        return returnOutput

    def generate_SW_WD(self):
        """ Generate a Software watchdog and throw a failure or success depending on the result.
            ( adb shell echo '1' > /d/watchdog/kernel_watchdog/trigger )
        """
        self.__logger.printLog("INFO", "Generate SW WD")
        if self.__configuration.boot.WATCHDOG_PATH:
            p = self.__configuration.boot.WATCHDOG_PATH
        elif self.__configuration.flash.UEFI:
            p = "emmc_ipanic"
        else:
            p = "watchdog/kernel_watchdog/trigger"
        verdict = self.__triggerWd(p)
        output = ""
        if not verdict:
            output = "failure to generate WD with '{0}'".format(p)
        self.__output.appendOutput(output, verdict)
        return verdict

    def generate_HW_WD(self):
        """ Generate a Hardware watchdog and throw a failure or success depending on the result.
            ( adb shell echo '1' > <root>/watchdog/security_watchdog/trigger )
        """
        self.__logger.printLog("INFO", "Generate HW WD")
        exec_status, ll = self.__host.commandExecAdb("shell ls -l {0}".format(self.__getDebugfs() + "/" + "watchdog/sc_watchdog/trigger"))
        if "No such file or directory" in ll:
            p = "watchdog/security_watchdog/trigger"
        else:
            p = "watchdog/sc_watchdog/trigger"
        verdict = self.__triggerWd(p)
        output = ""
        if not verdict:
            output = "failure to generate WD with '{0}'".format(p)
        self.__output.appendOutput(output, verdict)
        return verdict

    def kill_daemon(self):
        """ Kill watchdogd
        """
        self.__device.adbRoot()
        self.__logger.printLog("INFO", "kill watchdogd")
        if not self.__misc.checkProcess("watchdogd"):
            output = "watchdogd process does not exist"
            verdict = False
        else:
            exec_status, adbOutput = self.__host.commandExecAdb("shell ps | grep watchdogd")
            if exec_status != 0 :
                output = "failure to find watchdogd ({0})".format(adbOutput)
                verdict = False
            else:
                adbList = [e for e in adbOutput.split(" ") if e]
                try:
                    pid = int(adbList[1])
                    ppid = int(adbList[2])
                except Exception as e:
                    output = "not possible to extract PID and PPID values from: {0} ({1})".format(", ".join(adbList), e)
                    verdict = False
                else:
                    if ppid != 1:
                        output = "unexpected PPID value: {0} (1 expected for init)".format(ppid)
                        verdict = False
                    else:
                        exec_status, adbOutput = self.__host.commandExecAdb("shell kill -STOP {0}".format(pid))
                        if exec_status != 0:
                            output = "failure to kill watchdogd process"
                            verdict = False
                        else:
                            output = ""
                            verdict = True
        self.__output.appendOutput(output, verdict)
        return verdict

    def getRamdump(self):
        log = "getRamdump(): "
        self.__logger.printLog("INFO", log + "starting")
        if not self.__configuration.logs.RAMDUMP:
            self.__logger.printLog("INFO", log + "nothing to do, no Ramdump mechanism")
        else:
            # check ramdump presence on device
            if self.__device.checkPresence(self.__configuration.logs.RAMDUMP):
                self.__logger.printLog("INFO", log + "ramdump found")
                if self.force_ramdump_removal:
                    self.__logger.printLog("INFO", log + "force erasing of Ramdump file (intentionally triggered WD)")
                    self.__host.commandExecAdb("shell rm {0}".format(self.__configuration.logs.RAMDUMP), 60)
                else:
                    self.__logger.printLog("INFO", log + "pulling ramdump file")
                    self.__logs.pull(self.__configuration.logs.RAMDUMP, timeout=400, push_to_server=False)
                    self.__logger.printLog("INFO", log + "erasing ramdump to avoid pulling it later")
                    self.__host.commandExecAdb("shell rm {0}".format(self.__configuration.logs.RAMDUMP), 60)
            else:
                self.__logger.printLog("INFO", log + "no ramdump found")