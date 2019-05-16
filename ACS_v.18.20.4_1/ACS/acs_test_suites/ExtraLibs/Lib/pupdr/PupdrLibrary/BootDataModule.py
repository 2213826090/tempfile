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
@summary: Pupdr Library - BootDataModule
@since: 11/17/2014
@author: travenex
"""

import re
import OutputModule
import HostModule
import LoggerModule
import EfiVarModule
import ResetIrqModule
import DeviceModule
import OsManagerModule
import LogsModule
import WorkaroundModule
import ConfigurationModule

class BootDataModule(object):

    __instance = None
    __globalConf = None
    __logger = None
    __host = None
    __output = None
    __efiVar = None
    __resetIrq = None
    __device = None
    __osManager = None
    __logs = None
    __workaround = None
    __configuration = None
    __RSCI_table = None

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
        self.__efiVar = EfiVarModule.EfiVarModule()
        self.__resetIrq = ResetIrqModule.ResetIrqModule()
        self.__device = DeviceModule.DeviceModule()
        self.__osManager = OsManagerModule.OsManagerModule()
        self.__logs = LogsModule.LogsModule()
        self.__workaround = WorkaroundModule.WorkaroundModule()
        self.__configuration = ConfigurationModule.ConfigurationModule()
        self.__RSCI_table = list()

    def getSingleBootVar(self, bootLog, level=0):
        """ Read aplog and search for bootLog. Return the string found.
            level argument allows to return the historic level (0 is latest, 1 is penultimate...)
            i.e.: "WD=0x00"
        """
        self.__logger.printLog("INFO", "getSingleBootVar({0}): start - history level={1}".format(bootLog, level))
        verdict = True
        output = ""

        # LEVEL 0
        log = ""
        if level == 0:

            # bootreason
            if bootLog == "bootreason":
                output_value = ""
                exec_status, log = self.__host.commandExecAdb("shell cat /proc/cmdline")
                if exec_status != 0:
                    output = "error while reading /proc/cmdline"
                    verdict = False
                elif log:
                    search = re.search("androidboot.bootreason=(\S+)", log)
                    if search:
                        output_value = search.group(1)
                        self.__logger.printLog("DEBUG", "getSingleBootVar({0})={1}".format(bootLog, output_value))
                else:
                    output = "cmdline output empty"
                    verdict = False
                self.__output.appendOutput(output, verdict)
                return output_value

            # fw_error_code
            if bootLog == "fw_error_code":
                output_value = ""
                output_log = ""
                status, efi_var = self.__efiVar.getOctalDumpValue("SSRAMBASE")
                if status:
                    ssram_base_address = int(efi_var, 16)
                    offset = int("720", 16)
                    target_address = format(ssram_base_address + offset, 'x')
                    self.__logger.printLog("DEBUG", "target address = 0x{}, base = 0x{}, offset = 0x720".format(
                            target_address, efi_var))
                    exec_status, peeknpoke_output = self.__host.commandExecAdb("shell peeknpoke r {} 16".format(target_address), timeout=5)
                    if exec_status == 0 and "peeknpoke" not in peeknpoke_output and peeknpoke_output.split() > 6:
                        peeknpoke_output = peeknpoke_output.split()
                        output_value = peeknpoke_output[6]
                        self.__logger.printLog("DEBUG", "getSingleBootVar({0})={1}".format(bootLog, output_value))
                    else:
                        output_log = "failure with peeknpoke command"
                        verdict = False
                else:
                    output_log = "failure to dump 'SSRAMBASE' efi var"
                    verdict = False

                self.__output.appendOutput(output_log, verdict)
                return output_value

            # If level 0 and UEFI : read efi variables in sysfs
            if self.__configuration.flash.UEFI:
                if self.__configuration.boot.UEFI_VARS == "efi_vars":
                    status, efi_var = self.__efiVar.getEfiVar(self.__efiVar.efi_vars_matching[bootLog])
                elif self.__configuration.boot.UEFI_VARS == "rsci_table":
                    status, efi_var = self.__efiVar.getRSCIValue(bootLog, self.__RSCI_table)
                else:
                    status = False
                    efi_var = ""

                if not status:
                    efi_var = ""
                    output = "failure to get '{0}'".format(bootLog)
                else:
                    efi_var = efi_var.lower()
                self.__output.appendOutput(output, status)
                return efi_var


            # If level 0 and IAFW: read current wakesrc in /proc/cmdline
            if bootLog == "wakesrc":
                exec_status, log = self.__host.commandExecAdb("shell grep -a -o androidboot.wakesrc=.. /proc/cmdline")
                if exec_status != 0:
                    output = "error while reading /proc/cmdline (status='{0}', output='{1}')".format(str(exec_status), str(log))
                    verdict = False
                elif log:
                    log = log.replace("androidboot.wakesrc=", "")
                    self.__logger.printLog("DEBUG", "history(0): %s" %  log)
                else:
                    output = "no wakesrc found in /proc/cmdline"
                    verdict = False

            # If level 0: read current reset/wd in dmesg
            elif "reset" in bootLog or bootLog == "watchdog":
                if bootLog == "watchdog":
                    searchText = "wd"
                else:
                    searchText = bootLog
                exec_status, log = self.__host.commandExecAdb("shell dmesg | grep -a -i -o {0}=0x..".format(searchText))
                if exec_status != 0:
                    output = "error while reading dmesg (status='{0}', output='{1}')".format(str(exec_status), str(log))
                    verdict = False
                elif log:
                    log = log.splitlines()[-1]
                    log = log.split("0x")[-1].lower()
                    self.__logger.printLog("DEBUG", "getSingleBootVar({0})={1}".format(bootLog, log))
                else:
                    var = ""
                    exec_status, log = self.__device.checkPartitions(partition_list=["debug"], custom_reg_ex="/sys/kernel\S+")
                    self.__logger.printLog("DEBUG", "Check the debugfs partition presence : %s" % log)
                    if not exec_status:
                        exec_status, log = self.__host.commandExecAdb("shell mount -rw -t debugfs none /sys/kernel/debug")
                        if exec_status != 0:
                            output = "unable to mount debugfs : {0}".format(log)
                            verdict = False
                    if bootLog == "watchdog":
                        var = self.__device.getProperty("sys.watchdog.previous.counter")
                        if var != "":
                            log = "{0}0".format(var)
                    elif "reset" in bootLog:
                        exec_status, var = self.__host.commandExecAdb("shell cat /d/intel_scu_osnib/RESETSRC1")
                        if exec_status == 0 and "No such file or directory" not in var:
                            length = len(var.split("0x")[1])
                            if length == 1:
                                log = "0{0}".format(var.split("0x")[1])
                            else:
                                log = "{0}".format(var.split("0x")[1])
                        else:
                            var = ""
                    if var == "":
                        output = "'{0}' not found in sysfs".format(bootLog)
                        verdict = False
                    else:
                        self.__logger.printLog("DEBUG", "getSingleBootVar({0})={1}".format(bootLog, log))

            # Return formatted string (SSSSSSSS=0xss)
            if not verdict:
                outputValue = ""
            else:
                if log:
                    outputValue = log.lower()
                else:
                    outputValue = ""
            self.__output.appendOutput(output, verdict)
            return outputValue

        #
        # LEVEL > 0: APLOG
        #
        # Check that LOGS exists
        if not self.__device.checkPresence(self.__logs.deviceLogsPath):
            output = "missing '%s' on device".format(self.__logs.deviceLogsPath)
            self.__output.appendOutput(output, False)
            return ""
        # Add a range for PRINT_DEBUG
        history = level + 4
        # Get list of available files
        file_list = list()
        for l in ("aplog.1", "aplog"):
            l = self.__logs.deviceLogsPath + "/" + l
            if self.__device.checkPresence(l):
                file_list.append(l)

        returnOutput = ""
        if file_list:
            # Get last 'history' number of lines in aplog with 'bootLog' inside (duplicated removed)
            cmd = "shell grep -a %s= %s | grep -v PMIT | grep -v CRASHLOG | uniq" \
                  % (bootLog, " ".join(file_list))
            exec_status, log = self.__host.commandExecAdb(cmd, 3)

            # If empty
            if not log:
                output = "aplog is empty"
                verdict = False

            # if we do not have errors when reading aplogs
            elif exec_status == 0 and re.search(bootLog, log):
                # Update 'history' with number of lines returned
                log_list = log.splitlines()
                # If history bigger
                if len(log_list) < history:
                    history = len(log_list)
                # elif history included
                elif len(log_list) > history:
                    log_list = log_list[-history:]

                for line in log_list:
                    history -= 1
                    try:
                        output = filter(lambda x: bootLog+"=" in x,line.split(' '))[0]
                    except IndexError as e:
                        output = "index error ({})".format(e)
                        verdict = False
                        break
                    if output:
                        output = output.replace("androidboot.wakesrc=", "")
                        self.__logger.printLog("DEBUG", "history(%s): %s" % (history, output))
                        if history == level:
                            # Return formatted string (SSSSSSSS=0xss)
                            returnOutput =  output.lower()
                            break
                if verdict:
                    output = "'{0}' not found in '{1}'".format(bootLog, ", ".join(log_list))
                    verdict = False

            # Else we got errors
            else:
                output = "error while reading aplogs (status='{0}', output='{1}')".format(str(exec_status), str(log))
                verdict = False
        # Else no file available
        else:
            output= "no aplog"
            verdict = False

        if not verdict:
            returnOutput =  ""
        self.__output.appendOutput(output, verdict)
        return returnOutput

    def getAllBootVars(self, level=0):
        """ Read the aplogs and search for bootLog. Return the string found for WD, RESETIRQ1 and WAKESRC.
            i.e.: "WD=0x00 RESETIRQ1=0x20 WAKESRC=0x01"
        """
        output = ""
        verdict = True
        exec_status = self.__device.waitAdb(20, loopTime=3)
        if exec_status == "-1":
            verdict = False
            returnOutput = "no adb, cannot check boot data vars"
            output = returnOutput
        if not self.__device.adbRoot():
            verdict = False
            returnOutput = "failure to root device before getting boot vars"
            output = returnOutput
        # return string with one space
        elif self.__configuration.flash.UEFI:
            returnOutput = dict()
            # wd and mode are not available in efi_vars only (given by osloader)
            if self.__configuration.boot.UEFI_VARS == "efi_vars":
                varList = ["reset_source", "reset_type", "wake_source", "shutdown_source", "watchdog", "mode"]
            elif self.__configuration.boot.UEFI_VARS == "rsci_table":
                table = "/sys/firmware/acpi/tables/RSCI"
                if not self.__device.checkPresence(table):
                    table = "/sys/firmware/acpi/tables/RSCI1"
                self.__RSCI_table = self.__efiVar.getRSCITable(table)
                if not self.__RSCI_table:
                    self.__logger.printLog("INFO", "retrying to get RSCI table")
                    self.__RSCI_table = self.__efiVar.getRSCITable(table)
                varList = ["reset_source", "reset_type", "wake_source", "shutdown_source"]
                if len(self.__RSCI_table) > 8 and self.__RSCI_table[8] == "02":
                    varList.append("reset_extra_information")
                    if len(self.__RSCI_table) >= 48:
                        varList.append("reset_extra_information")
                    varList.append("fw_error_code")
            else:
                varList = list()
                self.__logger.printLog("WARNING", "unknown uefi var type: '{0}'".format(self.__configuration.boot.UEFI_VARS))
            # fill output dictionary
            for element in varList:
                returnOutput[element] = self.getSingleBootVar(element, level)
            returnOutput["bootreason"] = self.getSingleBootVar("bootreason", level)
        else:
            returnOutput = dict()
            returnOutput["watchdog"] = self.getSingleBootVar("watchdog", level)
            returnOutput["wakesrc"] = self.getSingleBootVar("wakesrc", level)
            returnOutput[self.__resetIrq.name().lower()] = self.getSingleBootVar(self.__resetIrq.name().lower(), level)

        self.__output.appendOutput(output, verdict)
        return returnOutput

    def bootDataCheck(self, expected, level=0):
        """ Uses the self.getAll function to retrieve the information about WD, RESETIRQ1 and WAKESRC from the last boot
            It returns SUCCESS if the expected string == in the information and the output string.
        """
        self.__logger.printLog("INFO", "Checking Register WD, RESETIRQ1, WAKESRC, EFI VARS")
        output = ""
        verdict = True

        if not self.__configuration.boot_vars.BOOT_DATA_CHECK:
            output = "skipping because 'BOOT_DATA_CHECK' configuration parameter is set to False"
            self.__output.appendOutput(output, True)
            return True, output

        if self.__device.getProperty("ro.build.type") == "user":
            output = "skipping because user build"
            self.__output.appendOutput(output, True)
            return True, output

        if not expected:
            output = "skipping because expected boot data is empty"
            self.__output.appendOutput(output, True)
            return True, output

        if self.__osManager.getOs() == "recovery" and not self.__configuration.boot.SHELL_IN_ROS:
            output = "skipping because no shell in ROS"
            self.__output.appendOutput(output, True)
            return True, output

        # Read values
        actualDict = self.getAllBootVars(level)

        # format expected value in dict
        expectedDict = dict()
        for element in expected.split(";"):
            splitElement = element.split("=")
            if len(splitElement) != 2:
                self.__logger.printLog("WARNING", "bootDataCheck(): invalid entry '{0}'".format(element))
            else:
                if splitElement[0] == "pattern":
                    splitElement = self.__resetIrq.pattern(splitElement[1]).split("=")
                expectedDict[splitElement[0]] = splitElement[1]

        # check getAllBootVars returned a dictionary
        if not actualDict or not isinstance(actualDict, dict):
            verdict = False
            return_output = "no boot data found"
        else:
            # Set default values
            return_output = "as expected:"
            for element in actualDict:
                if self.__configuration.boot.UEFI_VARS == "rsci_table" and self.__efiVar.isRsciEntry(element):
                    return_output += " {0}={1}".format(
                            element, self.__efiVar.formatRsciValue(element, actualDict[element]))
                else:
                    return_output += " {0}={1}".format(element, actualDict[element])

            # If only wakesrc, check wakesrc entry present both in actual and expected
            if self.__configuration.boot.WAKESRC_ONLY:
                # If no WAKESRC requested
                if "wakesrc" not in expected:
                    verdict = False
                    return_output = "no WAKESRC requested (actual: '{0}', expected: '{1}')".format(actualDict, expected)
                # Elif no WAKESRC read
                elif "wakesrc" not in actualDict:
                    verdict = False
                    return_output = "WAKESRC is empty (actual: '{0}', expected: '{1}')".format(actualDict, expected)

            if verdict:
                # Full check
                errors = list()
                for element in sorted(expectedDict):
                    if element not in actualDict:
                        errors.append(element)
                        self.__logger.printLog("WARNING", "boot log '{0}' missing from actual results".format(element))
                    else:
                        if not re.search(expectedDict[element], actualDict[element]):
                            errors.append(element)
                            self.__logger.printLog(
                                    "WARNING",
                                    "boot log '{0}' differs: actual='{1}', expected='{2}'".format(
                                            element, actualDict[element],
                                            expectedDict[element]))

                # if some errors found, change verdict and format output
                if errors:
                    return_output = ""
                    for element in sorted(expectedDict):
                        if element in errors:
                            if element in actualDict:
                                if self.__configuration.boot.UEFI_VARS == "rsci_table" \
                                        and self.__efiVar.isRsciEntry(element):
                                    return_output += "{0}=(actual: {1}, expected: " \
                                                     "{2}) ".format(
                                            element,
                                            self.__efiVar.formatRsciValue(element, actualDict[element]),
                                            self.__efiVar.formatRsciValue(element, expectedDict[element]))
                                else:
                                    return_output += "{0}=(actual: {1}, expected: {2}) ".format(
                                            element, actualDict[element], expectedDict[element])
                            else:
                                return_output += "{0}=(actual: None, expected: {1}) ".format(
                                        element, expectedDict[element])
                        else:
                            if self.__configuration.boot.UEFI_VARS == "rsci_table" \
                                    and self.__efiVar.isRsciEntry(element):
                                return_output += "{0}={1} ".format(
                                        element,
                                        self.__efiVar.formatRsciValue(element, actualDict[element]))
                            else:
                                return_output += "{0}={1} ".format(element, actualDict[element])
                    for element in \
                            [actual_element for actual_element in actualDict if actual_element not in expectedDict]:
                        if self.__configuration.boot.UEFI_VARS == "rsci_table" and self.__efiVar.isRsciEntry(element):
                            return_output += "{0}=(actual: {1}, expected: does not matter) ".format(
                                    element,
                                    self.__efiVar.formatRsciValue(element, actualDict[element]))
                        else:
                            return_output += "{0}=(actual: {1}, expected: " \
                                             "does not matter) ".format(element, actualDict[element])
                    output = return_output
                    verdict = False
                else:
                    self.__logger.printLog("INFO", return_output)

        if not verdict \
                and self.__workaround.isWorkaround(data_type=self.__workaround.Function, output=return_output)[0]:
            verdict = True
        self.__output.appendOutput(output, verdict)
        return verdict, return_output
