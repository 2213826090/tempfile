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
@summary: Pupdr Library - Watchdog scripts
@since: 15/01/2015
@author: travenex
"""

from .. import TestCaseModule
from .. import WatchdogModule
import Check_scripts
import Usb_scripts

class Watchdog(object):

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
        cls.__watchdog = WatchdogModule.WatchdogModule()

    def SwWatchdogDaemonTestCase(self):
        # remove intentionally created ramdump files
        self.__watchdog.force_ramdump_removal = True
        TestCase = Watchdog.SwWatchdogDaemonScript(self.__globalConf)
        TestCase()
        return TestCase.verdict, TestCase.output_dict

    def KernelWatchdogTestCase(self):
        # remove intentionally created ramdump files
        self.__watchdog.force_ramdump_removal = True
        TestCase = Watchdog.KernelWatchdogScript(self.__globalConf)
        TestCase()
        return TestCase.verdict, TestCase.output_dict

    def SecurityWatchdogTestCase(self):
        # remove intentionally created ramdump files
        self.__watchdog.force_ramdump_removal = True
        TestCase = Watchdog.SecurityWatchdogScript(self.__globalConf)
        TestCase()
        return TestCase.verdict, TestCase.output_dict

    def WatchdogFallBackPolicyTestCase(self):
        # remove intentionally created ramdump files
        self.__watchdog.force_ramdump_removal = True
        TestCase = Watchdog.WatchdogFallBackPolicyScript(self.__globalConf)
        TestCase()
        return TestCase.verdict, TestCase.output_dict

    class SwWatchdogDaemonScript(TestCaseModule.TestCaseModule.TC_structure):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_structure.__init__(self, conf)
            if not self.name:
                self.name = "SW WD DAEMON - "
        def runAllSteps(self):
            if not self.configuration.flash.UEFI and not self.configuration.boot.WATCHDOGD_ENABLED:
                output = "not an UEFI board and no watchdogd enabled"
                self.logger.printLog("INFO", output)
                self.skipping = True
                self.toggleVerdict(self.VALID)
                self.output_dict["valid_log"] = output
            self.runStep(Watchdog.ChangeFakePropertyStep, parameter={"value": "WDT,PANIC"})
            self.runStep(Watchdog.KillWatchdogDaemonStep, parameter={"expectedOsAfterReboot": "main|bootloader"})
            self.runStep(Check_scripts.Check.MosCheckAfterKernelWatchdogStep)
            self.runStep(Watchdog.ChangeFakePropertyStep, parameter={"value": '\\"\\"'}, run_on_failure=True)

    class KernelWatchdogScript(TestCaseModule.TestCaseModule.TC_structure):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_structure.__init__(self, conf)
            if not self.name:
                self.name = "SW WD MOS - "
        def runAllSteps(self):
            self.runStep(Watchdog.TriggerKernelWatchdogStep, parameter={"expectedOsAfterReboot": "main"})
            self.runStep(Check_scripts.Check.MosCheckAfterKernelWatchdogStep)

    class SecurityWatchdogScript(TestCaseModule.TestCaseModule.TC_structure):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_structure.__init__(self, conf)
            if not self.name:
                self.name = "HW WD MOS - "
        def runAllSteps(self):
            if self.configuration.flash.UEFI:
                output = "no Security WD available on UEFI boards"
                self.logger.printLog("INFO", output)
                self.skipping = True
                self.toggleVerdict(self.VALID)
                self.output_dict["valid_log"] = output
            else:
                self.runStep(Watchdog.TriggerSecurityWatchdogStep, parameter={"expectedOsAfterReboot": "main"})
                self.runStep(Check_scripts.Check.MosCheckAfterSecurityWatchdogStep)

    class WatchdogFallBackPolicyScript(TestCaseModule.TestCaseModule.TC_structure):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_structure.__init__(self, conf)
            if not self.name:
                self.name = "WD FALLBACK - "
        def runAllSteps(self):
            self.runStep(Watchdog.SetWatchdogCounterStep, parameter={"counter": 3})
            if self.configuration.flash.UEFI:
                self.runStep(Watchdog.KillWatchdogDaemonStep, parameter={"expectedOsAfterReboot": "recovery"})
            else:
                self.runStep(Watchdog.TriggerKernelWatchdogStep, parameter={"expectedOsAfterReboot": "recovery"})
            self.runStep(Check_scripts.Check.RosCheckAfterWatchdogStep)
            self.runStep(Usb_scripts.Usb.UsbUnplugReplugStep, parameter={"duration": self.configuration.timeout.ROS_EXPIRATION, "replug": True})
            self.runStep(Usb_scripts.Usb.PlugAndWaitOsStep, parameter={"expectedOs": "main", "timeout": self.device.getOsExtentedTimeout("main"), "plug": False})
            self.runStep(Check_scripts.Check.MosCheckAfterWatchdogFallbackStep)

    class KillWatchdogDaemonStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "Kill Watchdog Daemon"
            self.description = "send kill command to watchdogd process"
            self.defaultParameter = {"expectedOsAfterReboot": "main"}
        def step(self, parameter):
            expectedOs = parameter["expectedOsAfterReboot"]
            self.description += " and expect '{0}' OS after reboot".format(expectedOs)
            if not self.watchdog.kill_daemon():
                self.verdict = False
            elif not self.osManager.waitOs("offline", self.configuration.timeout.WATCHDOG_EXPIRATION, blind=True, loopTime=10):
                self.verdict = False
            elif expectedOs != "" and not self.osManager.waitOs(expectedOs, blind=True, loopTime=20, timeout=self.device.getOsExtentedTimeout("main") ):
                self.verdict = False
            elif "bootloader" in expectedOs and self.osManager.getOs() == "bootloader" and not self.osManager.adbReboot():
                self.verdict = False
                self.output = "failure to go reboot in MOS from crashmod"

    class TriggerKernelWatchdogStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "Trigger Kernel Watchdog"
            self.description = "trigger watchdog with file in debugfs"
            self.defaultParameter = {"expectedOsAfterReboot": "main"}
        def step(self, parameter):
            self.description += " and expect '{}' os after reboot".format(parameter["expectedOsAfterReboot"])
            if not self.watchdog.generate_SW_WD():
                self.verdict = False
            elif parameter["expectedOsAfterReboot"] != "" and not self.osManager.waitOs(parameter["expectedOsAfterReboot"], blind=True, loopTime=20):
                self.verdict = False

    class ChangeFakePropertyStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "Update Fake Event Property"
            self.description = "enable or disable event tagging as Fake"
            self.defaultParameter = {"value": ""}
        def step(self, parameter):
            value = parameter["value"]
            if value == "":
                self.osManager.gotoMos()
            self.logger.printLog("INFO", "Changing property value to '{0}'".format(value))
            self.host.commandExecAdb("shell setprop persist.crashlogd.events.fake {0}".format(value))
            self.logger.printLog("DEBUG", "Checking property value")
            self.host.commandExecAdb("shell getprop persist.crashlogd.events.fake")

    class TriggerSecurityWatchdogStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "Trigger Kernel Watchdog"
            self.description = "trigger watchdog with file in debugfs"
            self.defaultParameter = {"expectedOsAfterReboot": "main"}
        def step(self, parameter):
            self.description = " and expect '{}' os after reboot".format(parameter["expectedOsAfterReboot"])
            if not self.watchdog.generate_HW_WD():
                self.verdict = False
            elif parameter["expectedOsAfterReboot"] != "" and not self.osManager.waitOs(parameter["expectedOsAfterReboot"], blind=True, loopTime=20):
                self.verdict = False

    class SetWatchdogCounterStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "Set Watchdog Counter"
            self.description = "set value of counter before triggering watchdog"
            self.defaultParameter = {"counter": 3}
        def step(self, parameter):
            self.description = "set value of counter to {} before triggering watchdog".format(parameter["counter"])
            if self.configuration.flash.UEFI:
                self.verdict = self.efiVar.setEfiVar("WdtCounter", parameter["counter"])[0]
            else:
                exec_status = self.host.commandExecAdb("shell echo {0} > /sys/devices/virtual/misc/watchdog/counter".format(parameter["counter"]))[0]
                if exec_status != 0:
                    self.verdict = False