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
@summary: Pupdr Library - LogsModule
@since: 11/17/2014
@author: travenex
"""

import os
import re
import zipfile
import LoggerModule
import HostModule
import OutputModule
import DeviceModule
import WorkaroundModule
import MiscModule
import ConfigurationModule
import OsManagerModule
import ReportModule

class LogsModule(object):

    __instance = None
    __globalConf = None
    __logger = None
    __host = None
    __device = None
    __workaround = None
    __configuration = None
    __misc = None
    __osManager = None
    __report = None
    __output = None
    logsFiles = None
    __mandatoryLogsFiles = None
    tc_name = None
    tc_name_number = None
    tc_report_path = None
    bootota_log_path = None

    def __new__(cls):
        """
        Constructor that always returns the same instance
        """
        if not cls.__instance:
            cls.__instance = object.__new__(cls)
        return cls.__instance

    def init(self, globalConf=None):
        self.__globalConf = globalConf
        self.__logger = LoggerModule.LoggerModule()
        self.__host = HostModule.HostModule()
        self.__device = DeviceModule.DeviceModule()
        self.__workaround = WorkaroundModule.WorkaroundModule()
        self.__configuration = ConfigurationModule.ConfigurationModule()
        self.__misc = MiscModule.MiscModule()
        self.__osManager = OsManagerModule.OsManagerModule()
        self.__output = OutputModule.OutputModule()
        self.__report = ReportModule.ReportModule()
        self.logsFiles = list()
        self.__mandatoryLogsFiles = ["history_event"]
        self.tc_name = ""
        self.tc_name_number = ""
        campaign_report_path = self.__globalConf.get("REPORT_PATH")
        if campaign_report_path and os.path.isdir(campaign_report_path):
            self.bootota_log_path = os.path.abspath(os.path.join(campaign_report_path, "logs_bootota"))
            # if directory not created, create it for future use in TC
            if not os.path.isdir(self.bootota_log_path):
                os.makedirs(self.bootota_log_path)
            self.__logger.printLog("WARNING", "LogsModule init: storing BootOta specific data into "
                                              "{}".format(self.bootota_log_path))
        else:
            self.__logger.printLog("WARNING", "LogsModule init: invalid or empty report path "
                                              "('{}')".format(campaign_report_path))
            self.bootota_log_path = ""
        self.tc_report_path = ""

    def cleanAplogs(self):
        """ Remove aplog.* and empties aplog file in logs directory
        """
        self.__logger.printLog("INFO", "Clean aplog files")
        exec_status1, output = self.__host.commandExecAdb("shell rm {0}/aplog.*".format(self.__configuration.logs.LOG_PATH))
        exec_status2, output = self.__host.commandExecAdb("shell > {0}/aplog".format(self.__configuration.logs.LOG_PATH))
        if exec_status1 != 0 or exec_status2 != 0:
            self.__logger.printLog("WARNING", "ERROR - Failed to clean aplog files")
        self.__output.appendOutput("", None)

    def printFlags(self, tag="START"):
        log = "printFlags(): "
        self.__logger.printLog("INFO", log + "start")
        if self.__device.waitAdbDevices(10):
            self.__device.adbRoot()
            self.flagInAplog("{0}__{1}".format(tag, self.tc_name_number))
            self.flagInUart("{0}__{1}".format(tag, self.tc_name_number))
        else:
            self.__logger.printLog("DEBUG", log + "failure to detect device, cannot print flags")

    def flagInAplog(self, flag):
        """ Add a flag in aplog
        """
        self.__logger.printLog("INFO", "add flag '{0}' in aplog file".format(flag))
        status = self.__host.commandExecAdb("shell log -p i -t PUPDR '{0}'".format(flag))[0]
        if status != 0:
            self.__logger.printLog("WARNING", "failure to add flag '{0}' in aplog file".format(flag))
        self.__output.appendOutput("", None)

    def flagInUart(self, flag):
        """ Add a flag in uart
        """
        argument = "flag={0}".format(flag)
        self.__logger.printLog("INFO", "add flag '{0}' in uart console".format(flag))
        exec_status, cmdline = self.__host.commandExecAdb("shell cat /proc/cmdline")
        if exec_status != 0:
            self.__output.appendOutput("failure with adb command to get tty", False, argument)
        else:
            if "ctp" in self.__configuration.board:
                search = re.search("console=ttyMFD2", cmdline)
            else:
                search = re.search("console=ttyS[^, ]*", cmdline)
            if not search:
                self.__output.appendOutput("failure with regular expression to get tty", False, argument)
            else:
                console = search.group(0)
                exec_status, _ = self.__host.commandExecAdb(
                    "shell echo {0} > {1}".format(flag, console.replace("console=", "/dev/")))
                if exec_status != 0:
                    self.__output.appendOutput("failure writing flag in uart", False, argument)
                else:
                    self.__output.appendOutput("", True, argument=argument)

    def createReport(self, tcname=""):
        """ Create report logs directory in self.__globalConf["REPORT_PATH"]
        """
        log = "createReport(): "
        self.__logger.printLog("INFO", log + "start")
        if not tcname:
            # Value from Framework
            tcname = self.__globalConf.get("TCNAME", "")
        if not tcname and "TestCase" in self.__misc.getCaller(4):
            tcname = self.__misc.getCaller(4)
        # format TC Name:
        if "_" not in tcname and tcname:
            cap_list = re.findall('[A-Z][^A-Z]*', tcname)
            search = re.search("(.*){0}".format("".join(cap_list)), tcname)
            if search and search.group(1) != "":
                cap_list.insert(0, search.group(1).capitalize())
            tcname = "PUPDR_" + "_".join([f.upper() for f in cap_list]).replace("_TEST_CASE", "")
        self.tc_name = tcname
        self.tc_name_number = self.tc_name

        if not tcname or not self.bootota_log_path:
            self.__output.appendOutput("tc name or path to report missing "
                                       "(name='{0}', path='{1}')".format(tcname, self.bootota_log_path), None)
            return
        index = len([f for f in os.listdir(self.bootota_log_path) if (tcname+"_") in f])
        self.tc_name_number = self.tc_name + "_" + str(index)
        self.tc_report_path = os.path.abspath(os.path.join(self.bootota_log_path, self.tc_name_number))

        if not os.path.isdir(self.tc_report_path):
            os.makedirs(self.tc_report_path)
        self.__logger.printLog("INFO", log + "path to current TC data = {0}".format(self.tc_report_path))
        self.__output.appendOutput("", None)

    def pull(self, logs, target_path="", timeout=30, push_to_server=True):
        """ Pull the logs files to REPORT_PATH.
            One folder for each TC in REPORT_PATH.
        """
        self.__logger.printLog("INFO", "pull({0}): start".format(logs))

        self.__device.adbRoot()

        if not target_path:
            target_path = self.tc_report_path
        if not target_path:
            output = "no directory to pull files to"
            self.__output.appendOutput(output, False)
            return False

        verdict = True
        output = ""

        if self.__device.waitAdbDevices(20):
            # check if input is list of space-separated string list
            if isinstance(logs, list):
                fullList = logs
            else:
                fullList = logs.split()
            for log in fullList:
                if self.__device.checkPresence(log) or (self.__osManager.getOs(check_charger=False) == "recovery" and not self.__configuration.boot.SHELL_IN_ROS):
                    # Add sub folder target for directory logs
                    if self.__host.commandExecAdb("shell [ -d " + log + " ] && echo dir")[1] == "dir":
                        sub_folder = os.path.basename(log.strip("/"))
                        target_final_path = os.path.join(target_path, sub_folder)
                    else:
                        sub_folder = ""
                        target_final_path = target_path

                    self.__logger.printLog("INFO", "pull {0} to {1}".format(log, target_final_path))
                    exec_status, pullOutput = self.__host.commandExecAdb("pull {0} {1}".format(log, target_final_path), timeout)
                    if "No response" in pullOutput or exec_status != 0:
                        output = "cannot pull '{0}', adb command failed".format(log)
                        verdict = False
                    # push single files to TCR
                    if not sub_folder and os.path.isfile(os.path.join(target_final_path, log.split("/")[-1])) and push_to_server:
                        self.__report.TCRAttachmentPush(local_file=os.path.join(target_final_path, log.split("/")[-1]))
                    elif sub_folder and verdict and push_to_server:
                        try:
                            zip_file = zipfile.ZipFile(target_final_path + ".zip", "w")
                            for dirname, subdirs, files in os.walk(target_final_path):
                                zip_file.write(dirname)
                                for filename in files:
                                    zip_file.write(os.path.join(dirname, filename))
                            zip_file.close()
                            self.__report.TCRAttachmentPush(local_file=target_final_path + ".zip",
                                                            retention=self.__report.short_retention)
                        except Exception as e:
                            self.__logger.printLog("WARNING", "failure pushing to TCR {} (error={})".format(target_final_path, e))
        else:
            output = "cannot pull '{0}' because adb not connected".format(logs)
            verdict = False
        self.__output.appendOutput(output, verdict, argument="file={0}".format(logs))
        return verdict

    def pullAll(self, target_path="", verdict=True):
        """ Pull all the log files to REPORT_PATH.
            One folder for each TC in REPORT_PATH.
        """
        # add mandatory pulled files if not already done
        for element in self.__mandatoryLogsFiles:
            fullPath = "/".join((self.__configuration.logs.LOG_PATH, element))
            if fullPath not in self.logsFiles:
                self.logsFiles.append(fullPath)
        self.pull(self.logsFiles, target_path, push_to_server=True)
        self.extractTcAplog(self.tc_name_number, tc_verdict=verdict)
        self.__output.appendOutput("", None)

    def pullAllInto(self, target_dirname):
        pass
        # """
        # Creates a folder inside TC folder and stores inside all logs
        # """
        # if self.__osManager.gotoMos():
        #     flash_failure_path_logs = self.createReport(self.__globalConf["TCNAME_FLAG"] + "/"+ target_dirname)
        #     self.pullAll(flash_failure_path_logs)
        # self.__output.appendOutput("", None)

    def extractTcAplog(self, tc_flag, target_path=None, tc_verdict=True):
        """
        Dump the content of the aplog files on the phone between the START__ and the END__ flag of the specified tc_name
        And writes them in the corresponding TC report directory in file named "aplog".
        """
        verdict = True
        output = ""
        self.__logger.printLog("INFO", "extractTcAplog(): start")
        if not self.__globalConf.get("APLOGS_PUSH", False) and tc_verdict:
            self.__logger.printLog("INFO", "extractTcAplog(): skipping aplog extracting because 'APLOGS_PUSH' not set "
                                           "and test verdict is True")
            return False
        latest_aplogs = "{0}/aplog {0}/aplog.1".format(self.__configuration.logs.LOG_PATH)
        if not target_path:
            target_path = self.tc_report_path
        if not target_path:
            output = "no directory to pull files to"
            self.__output.appendOutput(output, False)
            return False
        # Sort the list of aplog files based on the number after the .
        exec_status, lsOutput = self.__host.commandExecAdb("shell ls {0}/aplog*".format(self.__configuration.logs.LOG_PATH))
        if exec_status != 0\
            or any(element in lsOutput for element in ["/system/bin/sh", "No such file or directory"])\
            or not lsOutput:
            output = "no aplog: '{0}'".format(lsOutput)
            self.pull(latest_aplogs, target_path=target_path, push_to_server=True)
            self.__output.appendOutput(output, False)
            return False
        else:
            aplog_list = " ".join(lsOutput.split('\n'))
        # Test if the two flags are present, else pull all the files
        s_exec_status, s_tag = self.__host.commandExecAdb("shell grep -m 1 -o START__{0} {1}".format(tc_flag, aplog_list))
        e_exec_status, e_tag = self.__host.commandExecAdb("shell grep -m 1 -o END__{0} {1}".format(tc_flag, aplog_list))

        if s_exec_status == 0 and e_exec_status == 0 and "START__" in s_tag and "END__" in e_tag:
            self.pull(latest_aplogs, target_path=target_path, push_to_server=False)
            joined_aplog_file = os.path.join(target_path, "aplog.{0}.log".format(tc_flag))
            local_aplog_0 = os.path.join(target_path, "aplog")
            local_aplog_1 = os.path.join(target_path, "aplog.1")
            aplog_content = list()
            if os.path.isfile(local_aplog_1):
                with open(local_aplog_1) as local_file:
                    aplog_content = aplog_content + local_file.readlines()
            if os.path.isfile(local_aplog_0):
                with open(local_aplog_0) as local_file:
                    aplog_content = aplog_content + local_file.readlines()
            output_string = ""
            for line in reversed(aplog_content):
                # store line in font of content
                output_string = line + output_string
                # if start found, stop
                if "START__{0}".format(tc_flag) in line:
                    break
                # if end found, erase previously found data
                if "END__{0}".format(tc_flag) in line:
                    output_string = line

            if output_string:
                with open(joined_aplog_file, "w") as local_file:
                    local_file.write(output_string)

                # check file presence, and pull aplog aplog.1 if file too small
                if not os.path.exists(joined_aplog_file) or os.path.getsize(joined_aplog_file) < 1000:
                    if not os.path.exists(joined_aplog_file):
                        self.__logger.printLog("WARNING", "pulled file does not exist: {0}".format(joined_aplog_file))
                    elif os.path.getsize(joined_aplog_file) < 1000:
                        self.__logger.printLog("WARNING", "pulled file size too small: {0} < 1000".format(os.path.getsize(joined_aplog_file)))
                else:
                    self.__logger.printLog("DEBUG", "joined aplog file checks passed for {0}".format(joined_aplog_file))
                    # remove previous files
                    if os.path.isfile(local_aplog_0):
                        os.remove(local_aplog_0)
                    if os.path.isfile(local_aplog_1):
                        os.remove(local_aplog_1)
            else:
                self.__logger.printLog("WARNING", "failure to extract data from pulled files")

            # push aplogs
            if os.path.isfile(local_aplog_0):
                self.__report.TCRAttachmentPush(local_aplog_0)
            if os.path.isfile(local_aplog_1):
                self.__report.TCRAttachmentPush(local_aplog_1)
            if os.path.isfile(joined_aplog_file):
                self.__report.TCRAttachmentPush(joined_aplog_file)
        else:
            # awk is not in the tool box, or one tag is missing
            # Pull last 2 aplogs only
            self.pull(latest_aplogs, target_path=target_path, push_to_server=True)
        self.__output.appendOutput(output, verdict)
        return verdict