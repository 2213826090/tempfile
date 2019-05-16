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
@summary: Pupdr Library - DeviceModule
@since: 11/17/2014
@author: travenex
"""

import os
import re
import time
import tempfile
import json
import datetime
import LoggerModule
import HostModule
import RelayCardModule
import OutputModule
import MiscModule
import ConfigurationModule

class DeviceModule(object):

    __instance = None
    __globalConf = None
    __logger = None
    __host = None
    __relayCard = None
    __output = None
    __misc = None
    __configuration = None

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
        self.__relayCard = RelayCardModule.RelayCardModule()
        self.__output = OutputModule.OutputModule()
        self.__misc = MiscModule.MiscModule()
        self.__configuration = ConfigurationModule.ConfigurationModule()

    def getAllProperties(self):
        """ Return all properties
        """
        exec_status, properties = self.__host.commandExecAdb("shell getprop")
        if exec_status != 0 or properties == "error: device not found":
            self.__logger.printLog("WARNING", "Failed to read all properties")
            properties = ""
        return properties

    def getProperty(self, prop):
        """ Return property value
        """
        exec_status, prop_value = self.__host.commandExecAdb("shell getprop %s" % prop)
        if exec_status != 0 or prop_value == "error: device not found" or not prop_value \
                or "No such file or directory" in prop_value:
            output = "getProperty(%s): Failed (output='%s')" % (prop, prop_value)
            self.__logger.printLog("WARNING", output)
            prop_value = ""
        return prop_value

    def getBiosVersion(self):
        """ Return property value
        """
        prop_value = self.getProperty("sys.ifwi.version")
        if not prop_value:
            exec_status, prop_value = self.__host.commandExecAdb("shell cat /sys/class/dmi/id/bios_version")
            if exec_status != 0 or\
                    any(element in prop_value for element in ["error: device not found", "/system"]) or\
                    not prop_value:
                output = "getBiosVersion(): failure to read /sys/class/dmi/id/bios_version"
                self.__logger.printLog("WARNING", output)
                prop_value = ""
        self.__logger.printLog("DEBUG", "getBiosVersion() = {}".format(prop_value))
        return prop_value

    def getDumpsysFocus(self):
        """ get the Windows dumpsys information on Focus (open window in UI)
        """
        log = "getDumpsysFocus(): "
        exec_status, focus = self.__host.commandExecAdb("shell dumpsys window windows | grep -a Focus")

        # No adb
        if exec_status != 0:
            output = log + focus
            self.__logger.printLog("WARNING", output)
            return False, output

        return True, focus

    def setProp(self, property_name, value, timeout=10):
        log = "setProp({}): ".format(property_name)
        value = str(value)
        self.__logger.printLog("INFO", log + "start (value={})".format(value))
        verdict = True
        output = ""
        exec_status = self.__host.commandExecAdb(
                "shell setprop {} {}".format(property_name, value), timeout=timeout)[0]
        if exec_status != 0:
            output = "failure with setprop command for {}={}".format(property_name, value)
            verdict = False
        else:
            for retry in range(3):
                adb_output = self.__host.commandExecAdb(
                        "shell getprop {}".format(property_name), timeout=timeout)[1]
                if adb_output == value:
                    break
                else:
                    time.sleep(1)
                if retry == 2:
                    output = "failure to read {}={}".format(property_name, value)
                    verdict = False
        self.__output.appendOutput(output, verdict, argument="{}={}".format(property_name, value))
        return verdict

    def getDumpsysBattery(self):
        """ return presence and level battery
        """
        log = "getDumpsysBattery(): "
        self.__logger.printLog("INFO", log + "start")

        exec_status, dumpsys = self.__host.commandExecAdb("shell dumpsys battery", silent_mode=True)

        if exec_status != 0:
            self.__logger.printLog("WARNING", "failure to read dumpsys battery: {0}".format("; ".join(dumpsys.split("\n"))))
            return False, "", ""

        # print debug data
        self.__host.commandExecAdb("shell cat /sys/class/power_supply/*/uevent", timeout=10)

        search = re.search("present: ([a-z]+)", dumpsys)
        if search:
            present = search.group(1)
        else:
            present = ""
        search = re.search("level: ([0-9]+)", dumpsys)
        if search:
            level = search.group(1)
        else:
            level = ""

        self.__logger.printLog("INFO", log + "present={0}, level={1}".format(present, level))
        return True, present, level

    def getBootTime(self):
        btime = ""
        verdict = True
        output = ""
        exec_status, proc_output = self.__host.commandExecAdb("shell cat /proc/stat", 10)
        if exec_status == 0:
            search = re.search("btime ([0-9]+)", proc_output)
            if not search:
                verdict = False
                output = "failure to extract btime from /proc/stat"
            else:
                btime = search.group(1)
        else:
            verdict = False
            output = "failure to get /proc/stat content"
        self.__logger.printLog("DEBUG", "getBootTime() = {}".format(btime))
        self.__output.appendOutput(output, verdict)
        return btime

    def getChargerType(self, print_warning=True):
        """ return charger type
        """
        if self.__configuration.boot.NOCOS:
            return True, "unknown"

        charger_type = "unknown"
        verdict = True
        log = "getChargerType(): "
        self.__logger.printLog("INFO", log + "start")

        # launch adb command
        cmd = "shell cat /sys/class/power_supply/*_charger/type"
        exec_status, charger_type_output = self.__host.commandExecAdb(cmd)

        if exec_status:
            verdict = False
        else:
            # compute charger type when adb command is successful
            if "CDP" in charger_type_output:
                charger_type = "CDP"
            elif "DCP" in charger_type_output:
                charger_type = "DCP"
            elif "USB"in charger_type_output:
                charger_type = "USB"
            else:
                verdict = False
            # display detected charger type
            self.__logger.printLog("INFO", log + "detected charger type is: %s" % charger_type)

        boardType = self.__configuration.board
        if not boardType:
            boardType = ""
        # print warning message when cht or byt devices are not powered properly
        if print_warning\
            and re.search("cht|byt", boardType)\
            and not re.search("CDP|DCP", charger_type):
            self.__logger.printLog("WARNING", log + "Board battery might be empty soon "
                                     "because trail device is not connected to CDP or DCP")

        return verdict, charger_type

    def removeStartScreen(self):
        """ remove start screen
        """
        log = "remove_start_screen(): "
        self.__logger.printLog("INFO", log + "start")
        exec_status, output = self.__host.commandExecAdb("shell pm disable com.google.android.setupwizard")
        if exec_status != 0:
            self.__logger.printLog("WARNING", log + "adb command failed")
        self.waitAdb(timeout=10)
        if re.search("WelcomeActivity", self.getDumpsysFocus()[1]):
            self.__logger.printLog("WARNING", log + "removing start screen with adb command failed")
        else:
            self.__logger.printLog("INFO", log + "start screen disabled")
        self.__output.appendOutput("", None)

    def waitAdb(self, timeout=None, start=None, loopTime=1):
        """ Wait that adb is available return sys.boot_completed value (or -'1' if timeout)
        """
        if timeout is None:
            timeout = self.__configuration.timeout.BOOT_TIMEOUT
        if not start:
            start = time.time()
        self.__logger.printLog("INFO", "waitAdb(timeout={0}): start".format(timeout))

        # Wait adb availability
        # If above timeout is reached the while will not be run
        readyOutput = ""
        while time.time() - start < timeout:
            exec_status, readyOutput = self.__host.commandExecAdb("shell echo ready", 10)
            self.__logger.printLog("INFO", "waitAdb(): timeout in %s" % str(timeout - (time.time() - start)))
            if readyOutput == "ready":
                break
            time.sleep(loopTime)

        verdict = True
        output = ""
        if readyOutput != "ready":
            output = "timeout {0}s".format(timeout)
            verdict = False
            boot_completed = "-1"
        else:
            boot_completed = self.getProperty("sys.boot_completed")
            if boot_completed != "1":
                boot_completed = "0"
            self.__logger.printLog("INFO", "waitAdb(): adb is available (%.3fs), sys.boot_completed=%s"
                    % (time.time() - start, boot_completed))

        self.__output.appendOutput(output, verdict, argument="timeout={0}".format(str(timeout)))
        return boot_completed

    def adbRoot(self):
        """ Run adb root
        """
        # directly return if user build is flashed
        if "user" in [self.__globalConf.get("PROVISIONING_DATA", {}).get("build_variant", ""), self.getProperty("ro.build.type")]:
            self.__output.appendOutput("cannot root user build", True)
            return True

        adbRootTimeout = self.__configuration.timeout.ADB_ROOT_TIMEOUT
        if not adbRootTimeout:
            adbRootTimeout = 30
        self.__logger.printLog("INFO", "adbRoot(timeout={0}): start".format(adbRootTimeout))

        if not self.waitAdbDevices(self.__configuration.timeout.MOS_BOOT):
            self.__output.appendOutput("failure to detect board before rooting device", False)
            return False

        exec_status = 1
        rootOutput = "-1"
        already_running = "adbd is already running as root"
        start = time.time()

        while rootOutput != already_running:
            if rootOutput != "-1" and (time.time() - start <= adbRootTimeout):
                self.__logger.printLog("INFO", "adbRoot(): timeout in {0:.2f}s...".format(adbRootTimeout-(time.time()-start)))
            exec_status, rootOutput = self.__host.commandExecAdb("root")
            if rootOutput != already_running:
                time.sleep(3)
            if time.time() - start > adbRootTimeout:
                exec_status = 1
                break

        verdict = True
        output = ""
        if exec_status != 0 and rootOutput != already_running:
            output = "'adb root' command failed (output={0})".format(rootOutput)
            verdict =  False
        self.__output.appendOutput(output, verdict, argument="timeout={0}".format(adbRootTimeout))
        return verdict

    # inner function for switchScreenState
    def __state(self):
        s = self.__host.commandExecAdb("shell grep screen_state= {0}/aplog | tail -1".format(self.__configuration.logs.LOG_PATH))[1]
        if "screen_state=on" in s:
            return True
        elif "screen_state=off" in s:
            return False
        else:
            return None

    def switchScreenState(self, on):
        """ Switch the screen to on or off state
        """
        log = "switchScreenState(%s): " % on
        self.__logger.printLog("INFO", "start", frontContent=log)
        self.adbRoot()

        s = self.__state()
        if s is on:
            self.__output.appendOutput("", True)
            return True

        if s is None:
            self.__relayCard.powerButtonPress(0.2)
            time.sleep(2)

        if self.__state() is not on:
            self.__relayCard.powerButtonPress(0.2)
            time.sleep(2)

        if self.__state() is on:
            output = ""
            verdict = True
        else:
            output = "screen switch failed"
            verdict = False
        self.__output.appendOutput(output, verdict)
        return verdict

    def keyEvent(self, keys, interval=2):
        """ keys is a list of key
        Press all the keys given in parameter through adb
        (adb shell input keyevent keys)
        keys can be a list ["a", "b" ,...] or a tuple ("a", "b", ...)
        """
        if self.waitAdb(timeout=self.__configuration.timeout.MOS_BOOT, loopTime=5) == "-1":
            self.__output.appendOutput("failure to get adb", False)
            return None, False

        for key in keys:
            time.sleep(interval)
            exec_status, output = self.__host.commandExecAdb("shell input keyevent %s" % str(key))
            if exec_status != 0:
                self.__output.appendOutput("keyEvent(): failed ({0})".format(output), False)
                return None, False
        time.sleep(interval)
        self.__output.appendOutput("", True)
        return time.time(), True

    def checkPresence(self, fileOrDir):
        """ Check that a file/directory exists in device
        """
        shellOutput = self.__host.commandExecAdb("shell [ -f " + fileOrDir + " ] || [ -d " + fileOrDir + " ] && echo present")[1]
        output = ""
        verdict = True
        if shellOutput != "present":
            output = "failure checking {0} (output='{1}')".format(fileOrDir, output)
            verdict = False
        self.__output.appendOutput(output, verdict, "file={0}".format(fileOrDir))
        return verdict

    def checkPartitions(self, partition_list, custom_reg_ex=""):
        """ Returns True and empty string if the partitions are present in main os
            else returns False and output error log.
        """
        self.__logger.printLog("INFO", "checkPartitions({0}): start".format(", ".join(partition_list)))

        if custom_reg_ex == "":
            custom_reg_ex = "/dev/block\S+ \S+|/dev/disk\S+ \S+"

        # Get all mounted partition on device
        exec_status, mount = self.__host.commandExecAdb("shell cat /proc/mounts", 3)
        if exec_status != 0:
            output = "check_partitions({0}): no adb result".format(custom_reg_ex)
            self.__output.appendOutput(output, False)
            return False, output

        # Find partitions matching regular expression
        search_list = re.findall(custom_reg_ex, mount)
        if len(search_list) == 0:
            output = "checkPartitions({0}): no partition found".format(custom_reg_ex)
            self.__output.appendOutput(output, False)
            return False, output
        else:
            self.__logger.printLog("INFO", "checkPartitions({0}): regular expression found '{1}'".format(custom_reg_ex, " - ".join(search_list)))

        not_found_partitions = list()
        # For each partition that presence must be checked
        for partition in partition_list:
            partition_found = False
            # check if one found partition matches the requested partition
            for element in search_list:
                if len(re.findall(partition, element)) > 0:
                    partition_found = True
                    break
            # if no partition matches
            if not partition_found:
                not_found_partitions.append(partition)

        if len(not_found_partitions) == 0:
            output = "all mounted"
            verdict = True
        else:
            output = "{0} partitions not mounted".format(", ".join(not_found_partitions))
            verdict = False

        self.__output.appendOutput(output, verdict, argument=", ".join(partition_list))
        return verdict, output

    # timeout specific method
    def getOsExtentedTimeout(self, os):
        if os == "main":
            return self.__configuration.timeout.MOS_BOOT + self.__configuration.timeout.TIMEOUT_TOLERANCE
        elif os == "charger":
            return self.__configuration.timeout.COS_BOOT + self.__configuration.timeout.TIMEOUT_TOLERANCE
        elif os == "recovery":
            return self.__configuration.timeout.ROS_BOOT + self.__configuration.timeout.TIMEOUT_TOLERANCE
        elif os == "fastboot":
            return self.__configuration.timeout.POS_BOOT + self.__configuration.timeout.TIMEOUT_TOLERANCE
        elif os == "dnx":
            return self.__configuration.timeout.SHUTDOWN_TIMEOUT + self.__configuration.timeout.TIMEOUT_TOLERANCE
        elif os == "offline":
            return self.__configuration.timeout.SHUTDOWN_TIMEOUT + self.__configuration.timeout.TIMEOUT_TOLERANCE
        else:
            return 0

    def adbDevices(self):
        verdict = True
        log = "adbDevices(): "
        self.__logger.printLog("INFO", log + "start")
        exec_status, devices = self.__host.commandExec("adb devices")
        # WA because of bad acs return for adb devices
        if not re.search("List of devices attached", devices):
            # add delay to avoid error in adb devices command return message
            self.__misc.waitDelay("adb command return message invalid, wait before retrying", 5)
            exec_status, devices = self.__host.commandExec("adb devices", 20)

        # extract only the concerned device
        if self.__host.serial_number:
            serial_number_list = [self.__host.serial_number]
            if self.__globalConf.get("DEVICE_SSN"):
                serial_number_list.append(self.__globalConf["DEVICE_SSN"])
            if self.__globalConf.get("CUSTOM_CONFIG", {}).get("extra_device_SSN") and \
                        isinstance(self.__globalConf["CUSTOM_CONFIG"]["extra_device_SSN"], list):
                    serial_number_list.extend(self.__globalConf["CUSTOM_CONFIG"]["extra_device_SSN"])
            serial_number_list = list(set(serial_number_list))
            self.__logger.printLog("INFO", log + "filtering results for {} ssn".format(", ".join(serial_number_list)))
            regexp = "({})".format("|".join((single_ssn + ".*") for single_ssn in serial_number_list))
            isDeviceHere = re.search(regexp, devices)
            if isDeviceHere:
                devices = isDeviceHere.group(0)
                regexp_ssn_only = "({})".format("|".join(serial_number_list))
                ssn_check = re.search(regexp_ssn_only, devices)
                if ssn_check and ssn_check.group(1) != self.__host.serial_number:
                    self.__logger.printLog("DEBUG", log +
                                           "updating device ssn from {} to "
                                           "{}".format(self.__host.serial_number,
                                                       ssn_check.group(1)))
                    self.__host.updateSerialNumbers(local_ssn=ssn_check.group(1))
            else:
                devices = ""
            self.__logger.printLog("INFO", log + "results for {1} device is: '{0}'".format(devices, self.__host.serial_number))

        if not re.search("(recovery|sideload|fastboot|device|bootloader)$", devices):
            self.__logger.printLog("WARNING", log + "device not detected")
            verdict = False

        return verdict, devices

    def waitAdbDevices(self, timeout):
        log = "waitAdbDevices(): "
        self.__logger.printLog("INFO", "waitAdbDevices(timeout={0}): start".format(timeout))
        if not timeout:
            timeout = self.__configuration.timeout.MOS_BOOT

        start = time.time()
        verdict = False
        while True:
            verdict = self.adbDevices()[0]
            if not verdict:
                self.__logger.printLog("INFO", log + "timeout in {0}".format(timeout-(time.time()-start)))
            if verdict:
                break
            if (time.time() - start) > timeout:
                self.__logger.printLog("INFO", log + "timeout expired")
                break
            time.sleep(5)

        if not verdict:
            output = "timeout {0}s".format(timeout)
        else:
            output = ""

        self.__output.appendOutput(output, verdict, argument="timeout={0}".format(timeout))
        return verdict

    def fastbootOemGetHashes(self, algorithm=None):
        log = "fastbootOemGetHashes(): "
        self.__logger.printLog("INFO", log + "start")
        exec_status, output = self.__host.commandExecFastboot("oem get-hashes{0}".
                                                              format(" {0}".format(algorithm) if algorithm else ""),
                                                              timeout=420)
        if exec_status != 0 or not output:
            self.__output.appendOutput("failure with fastboot command", False)
            return dict()
        output_dict = dict()
        partitions = re.findall("/[a-zA-Z0-9/\._]+", output)
        sha1_values = re.findall("hash: ([a-z0-9]+)", output)
        if len(partitions) == len(sha1_values):
            for index in range(len(sha1_values)):
                output_dict[partitions[index]] = sha1_values[index]
        if output_dict:
            verdict = True
            output = ""
        else:
            verdict = False
            output = "could not detect hashes from fastboot command output"
        self.__output.appendOutput(output, verdict)
        return output_dict

    def partitionSizeList(self):
        log = "partitionSizeList(): "
        self.__logger.printLog("INFO", log + "start")
        output_dict = {}
        output = ""
        verdict = True
        exec_status, adb_output = self.__host.commandExecAdb("shell cat /proc/partitions",
                                                         timeout=20)
        if exec_status != 0 or not adb_output:
            output = "failure with 'cat /proc/partitions' command on device"
            verdict = False
        else:
            formatted_list = []
            adb_output = adb_output.replace("\r", "")
            for single_line in adb_output.split("\n"):
                formatted_single_line = []
                for element in single_line.split(" "):
                    if element:
                        formatted_single_line.append(element)
                if formatted_single_line:
                    formatted_list.append(formatted_single_line)
            if len(formatted_list) < 2 or any(len(single_list) != 4 for single_list in formatted_list):
                output = "failure with to parse output"
                self.__logger.printLog("WARNING", log + "formatted list = {}".format(formatted_list))
                verdict = False
            elif formatted_list[0][3] != "name":
                output = "failure with parsed output"
                self.__logger.printLog("WARNING", log + "formatted list = {}".format(formatted_list))
                verdict = False
            else:
                for single_line in formatted_list[1:]:
                    output_dict[single_line[3]] = single_line[2]

        if not verdict:
            output_dict = {}
        else:
            self.__logger.printLog("DEBUG", log + "formatted output =\n{}".format(
                json.dumps(output_dict, indent=4, sort_keys=True)))

        self.__output.appendOutput(output, verdict)
        return output_dict

    @staticmethod
    def __versionCompare(actual, version):
        """ Compare 2 versions (ie 4.2.1) and returns -1,0,1 if actual<version,actual=version,actual>version
        """
        tup = lambda x: [int(y) for y in (x+'.0.0.0.0').split('.')][:4]
        # return -1, 0 or +1 respectively when actual<version, actual=version or actual>version
        return cmp(tup(actual), tup(version))

    def checkVersionFrom(self, version):
        """ Compare 2 versions (ie 4.2.1) and returns FAILURE,SUCCESS,SUCCESS if actual<version,actual=version,actual>version
        """
        log = "checkVersionFrom({0}): ".format(version)
        self.__logger.printLog("INFO", log + "start")
        osversion = self.getProperty("ro.build.version.release")
        self.__logger.printLog("INFO", log + "Android OS version on device: {0}".format(osversion))
        if self.__versionCompare(osversion, version) >= 0:
            return True
        else:
            return False

    def getPartitionInformation(self, partition):
        log = "getPartitionInformation(): "
        self.__logger.printLog("INFO", log + "start")
        data = {}
        if self.__configuration.branch in ["master", "n_mr0_cht", "n_mr1_cht", "master_car", "kernel41_master_car", "n_car", "n_master"]:
            exec_status, output = self.__host.commandExecAdb("shell df -h /{}".format(partition))
        else:
            exec_status, output = self.__host.commandExecAdb("shell df /{}".format(partition))
        if exec_status != 0:
            self.__logger.printLog("WARNING", log + "adb command failed, cannot get system data")
        else:
            formatted_output = list()
            for line in output.split("\n"):
                if not line.isspace():
                    formatted_output.append(line)
            if len(formatted_output) != 2:
                self.__logger.printLog("WARNING", "invalid output (it should contain 2 lines)")
            else:
                textLine = list()
                out = formatted_output[0].split(" ")
                for element in out:
                    if element != "":
                        textLine.append(element)
                dataLine = list()
                out = formatted_output[1].split(" ")
                for element in out:
                    if element != "":
                        dataLine.append(element)
                # keep 4 latest element
                if self.__configuration.branch in ["master", "n_mr0_cht", "n_mr1_cht", "master_car", "kernel41_master_car", "n_car" , "n_master"]:
                    textLine = textLine[-5:]
                    textLine = textLine[:4]
                else:
                    textLine = textLine[-4:]
                dataLine = dataLine[-4:]
                for index in range(4):
                    data[textLine[index].replace("\r", "")] = dataLine[index].replace("\r", "")
        if not data:
            self.__logger.printLog("INFO", log + "failure with system data gathering")
        else:
            self.__logger.printLog("DEBUG", log + "formatted data =\n{}".format(
                json.dumps(data, indent=4, sort_keys=True)
            ))
        return data

    def gvbLock(self):
        self.__logger.printLog("INFO", "Locking Device")
        if self.__configuration.boot.GVB_ACTIVATION.endswith("-M"):
            exec_status, output = self.__host.commandExecFastboot("flashing lock", 180)
        elif self.__configuration.boot.GVB_ACTIVATION == "verified":
            exec_status, output = self.__host.commandExecFastboot("oem verified", 180)
        else:
            exec_status, output = self.__host.commandExecFastboot("oem lock", 180)
        return exec_status, output

    def gvbUnlock(self):
        self.__logger.printLog("INFO", "Unlocking Device")
        if self.__configuration.boot.GVB_ACTIVATION.endswith("-M"):
            exec_status, output = self.__host.commandExecFastboot("flashing unlock", 180)
        else:
            exec_status, output = self.__host.commandExecFastboot("oem unlock", 180)
        return exec_status, output

    def scriptPush(self, command_list, host_script_name="", device_script_name="", run_script=False, timeout=10):
        self.__logger.printLog("INFO", "scriptPush(): start")
        self.adbRoot()
        if not host_script_name:
            host_script_name = "local_script"
        if not device_script_name:
            device_script_name = "/data/{}.sh".format(host_script_name)
        local_file = os.path.join(tempfile.gettempdir(), "{}_{}.sh".format(host_script_name, self.__misc.getUser()))
        self.__misc.local_files_handler.addEntry(local_file)
        if os.path.exists(local_file):
            os.remove(local_file)
        error_log = "ERROR - failure to run local script"
        end_log = "SUCCESS - code reached end"
        with open(local_file, "w+") as f:
            for element in command_list:
                f.write(element + "\n")
                f.write("return_code=$?" + "\n")
                f.write("if [[ $return_code != 0 ]]; then" + "\n")
                f.write("echo '{}'".format(error_log) + "\n")
                f.write("return 1" + "\n")
                f.write("fi" + "\n")
            f.write("echo '{}'".format(end_log) + "\n")
        self.__host.commandExecAdb("push {0} {1}".format(local_file, device_script_name))
        if os.path.isfile(local_file):
            os.remove(local_file)
        self.__host.commandExecAdb("shell cat {}".format(device_script_name))
        verdict = True
        output = ""
        if not self.checkPresence(device_script_name):
            verdict = False
            output = "failure to push script to device"
        elif self.__host.commandExecAdb("shell chmod +x {}".format(device_script_name))[0] != 0:
            verdict = False
            output = "failure to change script mode"
        elif run_script:
            exec_status, adb_output = self.__host.commandExecAdb("shell {}".format(device_script_name), timeout=timeout)
            if exec_status != 0:
                verdict = False
                output = "failure run script on device"
            elif error_log in adb_output:
                verdict = False
                output = "failure with imei flashing script (non-zero return code)"
            elif end_log not in adb_output:
                verdict = False
                output = "failure with imei flashing script (end not reached)"
        self.__output.appendOutput(output, verdict)
        return verdict

    def coldBootDone(self):
        self.__logger.printLog("INFO", "Get Cold Boot Done Status")
        exec_status, output = self.__host.commandExecAdb("shell dmesg | grep 'coldboot_done took'")

        if exec_status != 0:
            output_message = "adb shell dmesg | grep 'coldboot_done took' command execution failed"
            self.__logger.printLog("WARNING", output_message)
        else:
            if output:
                output = output.rstrip(".")
                if output.endswith("s"):
                    textLine = list()
                    out = output.split(" ")
                    for element in out:
                        if element != "":
                            textLine.append(element)
                    textLine = textLine[-1:]
                    output_message = "Cold reset time is: {}".format(textLine)
                    self.__logger.printLog("INFO", output_message)
                else:
                    output_message = "Time out waiting for coldboot_done. Cold reset time is > 1s"
                    self.__logger.printLog("WARNING", output_message)
            else:
                output_message = "Dmesg flashed. Please check the Serial Logs manually"
                self.__logger.printLog("WARNING", output_message)

        return output_message

    class DateTimeClass():
        """ Class dateTimeClass to get methods to set and read date. Both methods share common values.
        """
        def __init__(self):
            """ init
            """
            self.__host = HostModule.HostModule()
            self.__logger = LoggerModule.LoggerModule()
            # Shift of /proc/uptime in device
            self.uptimeShift  = None
            # Shift of date in device
            self.offtimeShift = None
            # Past time on host
            self.timePast     = None
            # Time read on device
            self.deviceTime   = None
            # Time read on host
            self.hostTime     = None
            self.__timeSet    = None
            self.__delta      = None
            self.__uptimeDiff = None

        def __getDateTime(self):
            """ Private method to get host time.
            """
            output = (datetime.datetime.now() +
                      datetime.timedelta(days=self.__delta["d"],
                                         hours=self.__delta["h"],
                                         minutes=self.__delta["m"],
                                         seconds=self.__delta["s"])).replace(microsecond=0)
            return output

        def __getUptimeDiff(self):
            """ Private method to compute uptime diff of host and device
            """
            host = "%.2f" % time.time()
            exec_status, device = self.__host.commandExecAdb("shell cat /proc/uptime")
            return float(host.split()[0])-float(device.split()[0])

        @staticmethod
        def __totalSeconds(td):
            """ Convert timedelta object into seconds.
            """
            output = td.seconds + td.days * 24 * 3600
            return output

        def setDateTime(self, deltaDays=0, deltaHours=0, deltaMinutes=0, deltaSeconds=0):
            """ Public method to set device time.
            """
            log = "setDateTime(): "
            self.__logger.printLog("INFO", log + "Setting date and time on host")
            self.__delta = {"d":deltaDays, "h":deltaHours, "m":deltaMinutes, "s":deltaSeconds}
            # Get time and set time device
            self.__timeSet = self.__getDateTime()
            dt = self.__timeSet.strftime("%Y%m%d.%H%M%S")
            exec_status, output = self.__host.commandExecAdb("shell date -s {0}".format(dt))
            # If command failed
            if exec_status != 0:
                self.__logger.printLog("WARNING", log + "Can't set date time, adb command failure")
            else:
                self.__logger.printLog("INFO", "Set date and time ok: {0}".format(dt))
            # Get uptime diff of host and device
            self.__uptimeDiff = self.__getUptimeDiff()
            return dt

        def readDateTime(self):
            """ Public method to read device time and compute time shift.
            """
            log = "readDateTime(): "
            self.__logger.printLog("INFO", log + "Reading date")
            # Read time in order: on host and right after on device
            now = self.__getDateTime()
            exec_status, dt = self.__host.commandExecAdb("shell date +%Y%m%d.%H%M%S")
            # End of time readings
            if exec_status != 0:
                self.__logger.printLog("WARNING", log + "cannot read date time, adb command failure")
                self.uptimeShift = -1
                return 0
            timePast = now - self.__timeSet
            try:
                # Convert date string into datetime object (y, m, d, H, M, S)
                timeRead = datetime.datetime(int(dt[:4]),   int(dt[4:6]),   int(dt[6:8]),
                                             int(dt[9:11]), int(dt[11:13]), int(dt[13:]))
                self.deviceTime = timeRead.strftime("%Y-%m-%d %H:%M:%S")
                self.hostTime   = now.strftime("%Y-%m-%d %H:%M:%S")
            except:
                self.__logger.printLog("WARNING", log + "failure to convert '{0}' into datetime object".format(dt))
                self.uptimeShift = -1
                return 0
            self.__logger.printLog("INFO", "host={0}, device={1}".format(self.hostTime, self.deviceTime))
            # Get total seconds from past time
            self.timePast = self.__totalSeconds(timePast)
            #
            # offtimeShift (to be used if device has been off) : Get seconds from diff between past time of device and host
            #
            self.offtimeShift = self.__totalSeconds((timeRead - self.__timeSet) - timePast)
            # If 1s shift then keep 0s since 1ms interval can cover 2 different seconds
            if abs(self.offtimeShift) == 1:
                self.offtimeShift = 0
            #
            # uptimeShift (more accurate, to be used if device has not been off): Compare previous uptime diff with a new one
            #
            self.uptimeShift = float("%.1f" % (self.__getUptimeDiff() - self.__uptimeDiff))
            # If more than 0.5s/60s time shifting
            if (self.uptimeShift / self.timePast) >= (0.5/60):
                self.uptimeShift = 0
            #
            # Return read time
            self.__logger.printLog("INFO", "uptimeShift={0}s, offtimeShift={1}s, timePast={2}s, deviceTime={3}, hostTime={4}"
                              .format(self.uptimeShift, self.offtimeShift,
                                      self.timePast, self.deviceTime, self.hostTime))

            return timeRead.strftime("%Y%m%d.%H%M%S")
