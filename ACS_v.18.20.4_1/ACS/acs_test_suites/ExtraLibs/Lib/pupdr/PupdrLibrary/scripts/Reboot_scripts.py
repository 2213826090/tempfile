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
@summary: Pupdr Library - COS scripts
@since: 07/01/2015
@author: travenex
"""

from .. import TestCaseModule

class Reboot(object):

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

    class Mos2CosStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "MOS to COS Transition"
            self.description = "launch 'adb reboot charging' command to reboot the board in COS"
        def step(self, parameter):
            if self.configuration.boot.ARCH == "sofia":
                self.description = "use graceful shutdown intent (instead of adb reboot command) to reboot the board in COS"
                if not self.osManager.gracefulShutdown():
                    self.verdict = False
                elif not self.osManager.waitOs("charger"):
                    self.verdict = False
            else:
                if not self.osManager.adbRebootCharging():
                    self.verdict = False

    class AdbRebootStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "Adb Reboot"
            self.description = "launch 'adb reboot' command to reboot the board in MOS"
            self.defaultParameter = {"loopTime": 3}
        def step(self, parameter):
            if not self.osManager.adbReboot(loopTime=parameter["loopTime"]):
                self.verdict = False

    class RebootBxtpMrbStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "Reboot Bxtp Mrb"
            self.description = "reboot Bxtp Mrb board in MOS using IOC commands 'r' and 'g' and check that board " \
                               "correctly booted in MOS (boot completed)"
        def step(self, parameter):
            if not self.osManager.rebootBxtpMrb():
                self.verdict = False

    class AdbRebootPtestStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "Adb Reboot in ptest mode"
            self.description = "launch 'adb reboot ptest' command to reboot the board in ptest mode"

        def step(self, parameter):
            if not self.osManager.adbRebootPtest():
                self.verdict = False

    class AdbRebootRecoveryFromPtestStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "Adb Reboot from ptest mode"
            self.description = "launch 'adb reboot ptest_clear' command to reboot the board in MOS from ptest"

        def step(self, parameter):
            if not self.osManager.adbRebootFromPtest():
                self.verdict = False

    class AdbRebootDnxStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "Adb Reboot Dnx"
            self.description = "launch 'adb reboot dnx' command to reboot the board in DNX"
        def step(self, parameter):
            if not self.osManager.adbRebootDnx():
                self.verdict = False

    class AdbRebootFromRecoveryStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "Adb Reboot In ROS"
            self.description = "launch 'adb reboot' command to reboot the board in MOS from ROS"
        def step(self, parameter):
            if self.configuration.boot.ARCH == "sofia":
                if not self.osManager.adbRebootRecoveryClear():
                    self.verdict = False
            elif not self.osManager.adbReboot():
                self.verdict = False

    class AdbRebootProvisioningStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "Adb Reboot Bootloader"
            self.description = "launch adb command to reboot the board in POS"
            self.defaultParameter = {"check_ssn": False}
        def step(self, parameter):
            self.description = "launch 'adb reboot {}' to reboot the board in POS".format(self.configuration.boot.FASTBOOT_TARGET)
            SSN_MOS = None
            if parameter["check_ssn"]:
                self.description += " and check SSN consistency"
                SSN_MOS = self.device.getProperty("ro.serialno")
                if not SSN_MOS:
                    self.logger.printLog("WARNING", "ro.serialno is empty in main !")
            if not self.osManager.adbRebootBootloader():
                self.verdict = False
            elif parameter["check_ssn"]:
                SSN_POS = self.host.commandExec("fastboot devices")[1]
                if not SSN_POS:
                    SSN_POS.print_log("WARNING", "fastboot devices is empty in POS !")
                else:
                    SSN_POS = SSN_POS.split()[0]
                if SSN_MOS != SSN_POS:
                    self.output = "SSN differ (main={0}, fastboot={1})".format(SSN_MOS, SSN_POS)
                    self.verdict = False

    class AdbRebootProvisioningIocStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "Reboot Bootloader using IOC commands"
            self.description = "launch adb IOC commands 'r' and 'n2#' to reboot the board in POS; check board is in POS"
        def step(self, parameter):
            if not self.osManager.gotoFastbootBxtpMrb():
                self.verdict = False

    class AdbRebootRecoveryStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "Adb Reboot Recovery"
            self.description = "launch adb command to reboot the board in ROS"
            self.defaultParameter = {"check_ssn": False, "check_tag": False}
        def __get_tag(self, os):
            incr = self.device.getProperty("ro.build.version.incremental")
            if not incr:
                self.logger.printLog("WARNING", "ro.build.version.incremental is empty in {0} !".format(os))
            return incr
        def __get_ssn(self, os):
            ssn = self.host.commandExecAdb("get-serialno")[1]
            if not ssn:
                self.logger.printLog("WARNING", "adb get-serialno is empty in {0} !".format(os))
            return ssn
        def step(self, parameter):
            # skip check if no shell in ROS
            doCheck = self.configuration.boot.SHELL_IN_ROS
            SSN_MOS = None
            TAG_MOS = None
            if doCheck:
                if parameter["check_ssn"]:
                    SSN_MOS = self.__get_ssn("main")
                    self.description += " and check SSN consistency"
                if parameter["check_tag"]:
                    TAG_MOS = self.__get_tag("main")
                    self.description += " and check build version consistency"
            if not self.osManager.adbRebootRecovery():
                self.verdict = False
            elif doCheck:
                errorList = list()
                if parameter["check_ssn"]:
                    SSN_ROS = self.__get_ssn("recovery")
                    if SSN_MOS != SSN_ROS:
                        errorList.append("serial number error (main='{0}', recovery='{1}')".format(SSN_MOS, SSN_ROS))
                if parameter["check_tag"]:
                    TAG_ROS = self.__get_tag("recovery")
                    if TAG_MOS != TAG_ROS:
                        errorList.append("tag error (main='{0}', recovery='{1}')".format(TAG_MOS, TAG_ROS))
                if errorList:
                    self.verdict = False
                    self.output = ", ".join(errorList)
