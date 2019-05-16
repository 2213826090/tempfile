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
@summary: Pupdr Library - MOS scripts
@since: 07/01/2015
@author: travenex
"""

import re
from .. import TestCaseModule
import Reboot_scripts
import Check_scripts
import S3_scripts
import COS_scripts

class MOS(object):

    __instance = None
    __testCase = None

    def __new__(cls):
        """
        Constructor that always returns the same instance
        """
        if not cls.__instance:
            cls.__instance = object.__new__(cls)
        return cls.__instance

    def __init__(self):
        pass

    @classmethod
    def init(cls, globalConf=None):
        cls.__globalConf = globalConf
        cls.__testCase = TestCaseModule.TestCaseModule()

    def adbRebootInMosAfterS3TestCase(self, wait=5, duration=60, loopTime=3):
        TestCase = MOS.adbRebootInMosScript(self.__globalConf)
        TestCase(S3mode=True, wait=wait, duration=duration, loopTime=loopTime)
        return TestCase.verdict, TestCase.output_dict

    def adbRebootInMosTestCase(self, wait=5, duration=60, loopTime=3):
        TestCase = MOS.adbRebootInMosScript(self.__globalConf)
        TestCase(S3mode=False, wait=wait, duration=duration, loopTime=loopTime)
        return TestCase.verdict, TestCase.output_dict

    def rebootInMosIocTestCase(self):
        TestCase = MOS.rebootInMosIocScript(self.__globalConf)
        TestCase()
        return TestCase.verdict, TestCase.output_dict

    def AdbRebootInPtestTestCase(self):
        TestCase = MOS.adbRebootInPtestScript(self.__globalConf)
        TestCase()
        return TestCase.verdict, TestCase.output_dict

    def CheckCryptoPropertyTestCase(self):
        TestCase = MOS.checkCryptoPropertyScript(self.__globalConf)
        TestCase()
        return TestCase.verdict, TestCase.output_dict

    def WarmRebootTestCase(self):
        TestCase = MOS.warmRebootScript(self.__globalConf)
        TestCase()
        return TestCase.verdict, TestCase.output_dict

    def DmVerityTestCase(self):
        TestCase = MOS.DmVerityScript(self.__globalConf)
        TestCase()
        return TestCase.verdict, TestCase.output_dict

    def DmVerityVerifiedSystemPartitionTestCase(self):
        TestCase = MOS.DmVerityVerifiedSystemPartitionScript(self.__globalConf)
        TestCase()
        return TestCase.verdict, TestCase.output_dict

    def GoToMosTestCase(self):
        TestCase = MOS.GoToMosScript(self.__globalConf)
        TestCase()
        return TestCase.verdict, TestCase.output_dict

    class warmRebootScript(TestCaseModule.TestCaseModule.TC_structure):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_structure.__init__(self, conf)
            if not self.name:
                self.name = "MOS ADB REBOOT - "
        def runAllSteps(self, S3mode=False, wait=5, duration=60, loopTime=3):
            if S3mode:
                if "_" not in self.name:
                    self.name = "ADB REBOOT AFTER S3 - "
                self.runStep(S3_scripts.S3.S3Step, parameter={"duration": duration, "wait": wait})
            self.runStep(COS_scripts.COS.BatteryChargingStep)
            self.runStep(Reboot_scripts.Reboot.AdbRebootStep, parameter={"loopTime": loopTime})
            self.runStep(Check_scripts.Check.MosCheckAfterRebootStep)

    class adbRebootInMosScript(TestCaseModule.TestCaseModule.TC_structure):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_structure.__init__(self, conf)
            if not self.name:
                self.name = "MOS ADB REBOOT - "
        def runAllSteps(self, S3mode=False, wait=5, duration=60, loopTime=3):
            if S3mode:
                if "_" not in self.name:
                    self.name = "ADB REBOOT AFTER S3 - "
                self.runStep(S3_scripts.S3.S3Step, parameter={"duration": duration, "wait": wait})
            self.runStep(Reboot_scripts.Reboot.AdbRebootStep, parameter={"loopTime": loopTime})
            self.runStep(Check_scripts.Check.MosCheckAfterRebootStep)

    class rebootInMosIocScript(TestCaseModule.TestCaseModule.TC_structure):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_structure.__init__(self, conf)
            if not self.name:
                self.name = "MOS REBOOT IOC - "
        def runAllSteps(self):
            self.logger.printLog("INFO", "Disable init and final")
            self.enable_init = False
            self.enable_final = False
            self.runStep(Reboot_scripts.Reboot.RebootBxtpMrbStep)

    class adbRebootInPtestScript(TestCaseModule.TestCaseModule.TC_structure):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_structure.__init__(self, conf)
            if not self.name:
                self.name = "ADB REBOOT PTEST - "

        def runAllSteps(self):
            self.runStep(Reboot_scripts.Reboot.AdbRebootPtestStep)
            self.runStep(Reboot_scripts.Reboot.AdbRebootRecoveryFromPtestStep)

    class checkCryptoPropertyScript(TestCaseModule.TestCaseModule.TC_structure):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_structure.__init__(self, conf)
            if not self.name:
                self.name = "CHECK CRYPTO PROPERTY - "
        def runAllSteps(self):
            self.runStep(MOS.GotoMosStep)
            self.runStep(MOS.CheckCryptoPropertyStep)

    class DmVerityScript(TestCaseModule.TestCaseModule.TC_structure):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_structure.__init__(self, conf)
            if not self.name:
                self.name = "DM VERITY - "
        def runAllSteps(self):
            self.runStep(MOS.CheckSystemMountTypeMosStep, parameter={"type": "ro"})
            self.runStep(MOS.DisableVerityStep)
            self.runStep(MOS.CheckSystemMountTypeMosStep, parameter={"type": "rw"})

    class DmVerityVerifiedSystemPartitionScript(TestCaseModule.TestCaseModule.TC_structure):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_structure.__init__(self, conf)
            if not self.name:
                self.name = "VERIFIED SYSTEM - "
        def runAllSteps(self):
            self.runStep(MOS.ReadSystemPartitionStep)
            self.runStep(Reboot_scripts.Reboot.AdbRebootStep)
            self.runStep(MOS.CheckVerifiedPropertyStep)

    class GoToMosScript(TestCaseModule.TestCaseModule.TC_structure):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_structure.__init__(self, conf)
            if not self.name:
                self.name = "GO TO MOS - "
        def runAllSteps(self):
            self.runStep(MOS.GotoMosStep)

    class MosGracefulShutdownStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "Gracefully Shutdown the Board"
            self.description = "launching intent to gracefully shutdown the board"
            self.defaultParameter = {"plug": True, "intent": True}
        def step(self, parameter):
            if parameter["plug"] and parameter["intent"]:
                self.description = "shutdown with adb intent and keep USB charger plugged in"
            elif not parameter["plug"] and parameter["intent"]:
                self.description = "shutdown with nohup command and disconnect USB charger"
            else:
                self.description = "shutdown with keyevent to toggle shutdown via UI"
            self.verdict = self.osManager.gracefulShutdown(plug=parameter["plug"], intent=parameter["intent"])

    class GotoMosStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "Go To Mos"
            self.description = "launch recovery procedure to go to MOS"
        def step(self, parameter):
            if not self.osManager.gotoMos():
                if self.osManager.getOs(check_charger=False) != "main":
                    self.verdict = False

    class ReadSystemPartitionStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "Read System Partition"
            self.description = "launch dd command to have all system blocks read"
        def step(self, parameter):
            output_dict = self.device.partitionSizeList()
            if not output_dict:
                self.verdict = False
                return
            if "dm-0" not in output_dict:
                self.verdict = False
                self.output = "missing dm-0 from partition size list"
                return
            cmd = "shell dd if=/dev/block/dm-0 of=/dev/null bs=1024 count={}".format(output_dict["dm-0"])
            exec_status, output = self.host.commandExecAdb(cmd, timeout=150)
            if exec_status != 0:
                self.verdict = False
                self.output = "failure with dd command"

    class CheckVerifiedPropertyStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "Check Verified System Partition"
            self.description = "get 'partition.system.verified' property and check value is 2"
        def step(self, parameter):
            prop = self.device.getProperty("partition.system.verified")
            if prop != "2":
                self.verdict = False
                self.output = "invalid partition.system.verified value (expected=2, actual={})".format(prop)
            else:
                self.output = "as expected, partition.system.verified=2"

    class CheckCryptoPropertyStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "Check Crypto Property"
            self.description = "get 'ro.crypto.fs_crypto_blkdev' property and check value is '/dev/block/dm-1'"
        def step(self, parameter):
            prop = self.device.getProperty("ro.crypto.fs_crypto_blkdev")
            if prop != "/dev/block/dm-1":
                self.verdict = False
                self.output = "invalid ro.crypto.fs_crypto_blkdev value (expected=/dev/block/dm-1, actual={})".format(prop)
            else:
                self.output = "as expected, ro.crypto.fs_crypto_blkdev=/dev/block/dm-1"

    class CheckSystemMountTypeMosStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "Check System Mount"
            self.description = "launch adb command to check mounted partitions and system type"
            self.defaultParameter = {"type": "ro"}
        def step(self, parameter):
            expected_type = parameter["type"]
            if expected_type == "rw":
                self.description = "remount and check /system/bin can be written in"
            else:
                self.description = "check /system/bin cannot be written in"
            file_location = "/system/bin/BootOtaTest"

            if expected_type == "rw":
                self.verdict =  self.device.adbRoot()
                if self.verdict:
                    exec_status, output = self.host.commandExecAdb("remount", 10)
                    if exec_status != 0:
                        self.verdict = False
                        self.output = "failure with 'adb remount' command"

            if self.verdict:
                exec_status, output = self.host.commandExecAdb("shell echo 'test' > {0}".format(file_location), 10)
                if exec_status != 0 and self.configuration.branch not in ["master", "n_mr0_cht", "n_mr1_cht", "n_master"]:
                    self.verdict = False
                    self.output = "failure with 'adb shell' command to write in {0}".format(file_location)
                elif expected_type == "ro":
                    if not re.search("Read-only file system", output) or self.device.checkPresence(file_location):
                        self.verdict = False
                        self.output = "system expected in RO but able to write in {0}".format(file_location)
                    else:
                        self.logger.printLog("INFO", "system checked in RO state")
                elif expected_type == "rw":
                    if not self.device.checkPresence(file_location):
                        self.verdict = False
                        self.output = "system expected in RW but not able to write in {0}".format(file_location)
                    else:
                        self.logger.printLog("INFO", "system checked in RW state")
                    self.host.commandExecAdb("shell rm {0}".format(file_location), 10)
                    self.osManager.adbReboot()

    class DisableVerityStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "Disable Verity"
            self.description = "launch adb command to disable dm-verity and reboot device"
            self.defaultParameter = {}
        def step(self, parameter):
            self.verdict =  self.device.adbRoot()
            if self.verdict:
                exec_status, output = self.host.commandExecAdb("disable-verity", 10)
                if exec_status == 0 and re.search("(Verity disabled on /system|Verity already disabled on /system)", output):
                    self.verdict = self.osManager.adbReboot()
                else:
                    self.verdict = False
                    self.output = "failure with 'adb disable-verity' command (error={0})".format(output)

    class CheckBootTimeStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "Check Boot Time"
            self.description = "check boot time value in /proc/stat"
            self.defaultParameter = {"compare_with": "", "match": False}
        def step(self, parameter):
            compare_with = parameter["compare_with"]
            match = parameter["match"]
            if compare_with and match:
                self.description += "and check value is {}".format(compare_with)
            elif compare_with and not match:
                self.description += "and check value is not {}".format(compare_with)
            b_time = self.device.getBootTime()
            if not b_time:
                self.verdict = False
            else:
                self.globalConf["CURRENT_BOOT_TIME"] = b_time
                if compare_with and b_time == compare_with and not match:
                    self.verdict = False
                    self.output = "boot time matches, board has not rebooted"
                elif compare_with and b_time != compare_with and match:
                    self.verdict = False
                    self.output = "boot time does not match, booted unexpected"
                else:
                    self.output = "actual value = {}".format(b_time)
