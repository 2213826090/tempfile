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

import time
from .. import TestCaseModule
from .. import ConfigurationModule
import Reboot_scripts
import Check_scripts
import Usb_scripts
import KeyPress_scripts
import MOS_scripts

class COS(object):

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
        cls.__configuration = ConfigurationModule.ConfigurationModule()

    def UsbUnplugInCosTestCase(self):
        TestCase = COS.UsbUnplugInCosScript(self.__globalConf)
        TestCase()
        return TestCase.verdict, TestCase.output_dict

    def AdbRebootInCosTestCase(self):
        TestCase = COS.AdbRebootInCosScript(self.__globalConf)
        TestCase()
        return TestCase.verdict, TestCase.output_dict

    def PkpOnInCosTestCase(self):
        TestCase = COS.PkpInCosScript(self.__globalConf)
        TestCase(pkpDuration=self.__configuration.timeout.PKPON)
        return TestCase.verdict, TestCase.output_dict

    def PkpOffInCosTestCase(self):
        TestCase = COS.PkpInCosScript(self.__globalConf)
        TestCase(pkpDuration=self.__configuration.timeout.PKPOFF)
        return TestCase.verdict, TestCase.output_dict

    def HardShutdownAndCosBootTestCase(self):
        TestCase = COS.HardShutdownAndCosBootScript(self.__globalConf)
        TestCase()
        return TestCase.verdict, TestCase.output_dict

    def GracefulShutdownAndCosBootTestCase(self):
        TestCase = COS.GracefulShutdownAndCosBootScript(self.__globalConf)
        TestCase()
        return TestCase.verdict, TestCase.output_dict

    def BatteryChargingTestCase(self):
        TestCase = COS.BatteryChargingScript(self.__globalConf)
        TestCase()
        return TestCase.verdict, TestCase.output_dict

    class AdbRebootInCosScript(TestCaseModule.TestCaseModule.TC_structure):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_structure.__init__(self, conf)
            if not self.name:
                self.name = "COS ADB REBOOT - "
        def runAllSteps(self):
            if self.configuration.boot.NOCOS:
                output = "no COS on board"
                self.logger.printLog("INFO", output)
                self.skipping = True
                self.toggleVerdict(self.VALID)
                self.output_dict["valid_log"] = output
            self.runStep(Reboot_scripts.Reboot.Mos2CosStep)
            self.runStep(Reboot_scripts.Reboot.AdbRebootStep)
            self.runStep(Check_scripts.Check.MosCheckAfterRebootStep)

    class PkpInCosScript(TestCaseModule.TestCaseModule.TC_structure):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_structure.__init__(self, conf)
            if not self.name:
                self.name = "COS PKP - "
        def runAllSteps(self, pkpDuration=3):
            if "_" not in self.name:
                self.name = "COS PKP {0}S - ".format(pkpDuration)
            if self.configuration.boot.NOCOS:
                output = "no COS on board"
                self.logger.printLog("INFO", output)
                self.skipping = True
                self.toggleVerdict(self.VALID)
                self.output_dict["valid_log"] = output
            self.runStep(Reboot_scripts.Reboot.Mos2CosStep)
            self.runStep(Check_scripts.Check.CosCheckAfterRebootStep)
            self.runStep(COS.PkpInCosStep, parameter={"pkpDuration": pkpDuration})

    class UsbUnplugInCosScript(TestCaseModule.TestCaseModule.TC_structure):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_structure.__init__(self, conf)
            if not self.name:
                self.name = "COS USB UNPLUG - "
        def runAllSteps(self):
            if self.configuration.boot.NOCOS:
                output = "no COS on board"
                self.logger.printLog("INFO", output)
                self.skipping = True
                self.toggleVerdict(self.VALID)
                self.output_dict["valid_log"] = output
            self.runStep(Reboot_scripts.Reboot.Mos2CosStep)
            self.runStep(Usb_scripts.Usb.UsbUnplugReplugStep, parameter={"duration": self.configuration.timeout.SHUTDOWN_TIMEOUT, "replug": False})
            self.runStep(Usb_scripts.Usb.UsbInsertionFromOffStep)

    class HardShutdownAndCosBootScript(TestCaseModule.TestCaseModule.TC_structure):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_structure.__init__(self, conf)
            if not self.name:
                self.name = "F_SHUTDOWN UNPLUG - "
        def runAllSteps(self):
            self.runStep(KeyPress_scripts.KeyPress.ForceShutdownStep, parameter={"plug": False})
            self.runStep(Usb_scripts.Usb.PlugAndGetOsStep, parameter={"expectedOs": ("offline", "dnx"), "replug": True, "wait_charger": False})
            if self.configuration.boot.NOCOS:
                self.runStep(Usb_scripts.Usb.PlugAndWaitOsStep, parameter={"expectedOs": "offline", "timeout": self.configuration.timeout.COS_BOOT, "plug": False})
            else:
                self.runStep(Usb_scripts.Usb.PlugAndWaitOsStep, parameter={"expectedOs": "charger", "timeout": self.configuration.timeout.COS_BOOT, "plug": False})
                self.runStep(Check_scripts.Check.CosCheckAfterHardShutdownStep)

    class GracefulShutdownAndCosBootScript(TestCaseModule.TestCaseModule.TC_structure):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_structure.__init__(self, conf)
            if not self.name:
                self.name = "G_SHUTDOWN UNPLUG - "
        def runAllSteps(self):
            self.runStep(MOS_scripts.MOS.MosGracefulShutdownStep, parameter={"plug": False})
            self.runStep(Usb_scripts.Usb.PlugAndGetOsStep, parameter={"expectedOs": ("offline", "dnx"), "replug": True, "wait_charger": False})
            if not self.verdict and self.osManager.getOs() == "main":
                self.logger.printLog("INFO", "retrying graceful shutdown")
                self.toggleVerdict(self.SUCCESS)
                self.runStep(Usb_scripts.Usb.PlugAndGetOsStep, parameter={"expectedOs": "main", "replug": False, "wait_charger": False})
                self.runStep(MOS_scripts.MOS.MosGracefulShutdownStep, parameter={"plug": False})
                self.runStep(Usb_scripts.Usb.PlugAndGetOsStep, parameter={"expectedOs": ("offline", "dnx"), "replug": True, "wait_charger": False})
            if self.configuration.boot.NOCOS:
                self.runStep(Usb_scripts.Usb.PlugAndWaitOsStep, parameter={"expectedOs": "offline", "timeout": self.configuration.timeout.COS_BOOT, "plug": False})
            elif "fugu" in self.configuration.board:
                self.runStep(Usb_scripts.Usb.PlugAndWaitOsStep, parameter={"expectedOs": "main", "timeout": self.configuration.timeout.COS_BOOT, "plug": False})
                self.runStep(Check_scripts.Check.CosCheckAfterGracefulShutdownStep)
            else:
                self.runStep(Usb_scripts.Usb.PlugAndWaitOsStep, parameter={"expectedOs": "charger", "timeout": self.configuration.timeout.COS_BOOT, "plug": False})
                self.runStep(Check_scripts.Check.CosCheckAfterGracefulShutdownStep)

    class BatteryChargingScript(TestCaseModule.TestCaseModule.TC_structure):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_structure.__init__(self, conf)
            if not self.name:
                self.name = "BATTERY CHARGING - "
        def runAllSteps(self):
            self.runStep(COS.BatteryChargingStep)

    class PkpInCosStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "Power key Press in COS"
            self.description = "press power key press and wait for OS change"
            self.defaultParameter = {"pkpDuration": self.configuration.timeout.PKPON}
        def step(self, parameter):
            pkpDuration = parameter["pkpDuration"]
            self.description = "press power key press for {}s".format(pkpDuration)
            pkpDuration = int(pkpDuration)
            self.relayCard.powerButtonPress(pkpDuration)
            # If pkp_duration >= 7 (hard shutdown), check if board goes OFF
            if pkpDuration >= 7:
                self.description += " and expect offline because of hard shutdown"
                if self.osManager.getOs() not in ("offline", "dnx"):
                    self.verdict = False
            # If pkp_duration >= PKPON (user power key press), check if board reboots in MOS
            elif pkpDuration >= self.configuration.timeout.PKPON:
                self.description += "and expect MOS"
                if not self.osManager.waitOs("main", blind=True):
                    self.verdict = False
            # power key not long enough to boot the board, it should stay in COS
            else:
                self.description += "expect COS because power key not pressed long enough"
                if self.osManager.getOs() != "charger":
                    self.verdict = False

    class BatteryChargingStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "Battery Charging"
            self.description = "put the device in COS and wait enough time for the battery to charge up"
        def step(self, parameter):
            battery_level = "100"
            if not self.osManager.getOs() == "main":
                if not self.osManager.getOs() == "charger":
                    if not self.osManager.getOs() == "offline":
                        self.osManager.forceShutdown(plug=False)
                    else:
                        self.relayCard.usbConnection(False)
                    self.relayCard.usbConnection(True)
                    self.osManager.waitOs("charger")

                if self.osManager.getOs() in ("charger","main"):
                    exec_status, capacity = self.host.commandExecAdb("shell cat /sys/class/power_supply/*_battery/capacity")
                    if exec_status != 0 or "No such file or directory" in capacity:
                        if not self.osManager.gotoMos():
                            self.logger.printLog("WARNING","go to mos failed")
                        else:
                            status, battery_present, battery_level = self.device.getDumpsysBattery()
                            if not status:
                                self.logger.printLog("WARNING","get battery level failed")
                    else:
                        battery_level = int(capacity)
            else:
                status, battery_present, battery_level = self.device.getDumpsysBattery()
                if not status:
                    self.logger.printLog("WARNING","get battery level failed")

            battery_level = int(battery_level)
            if battery_level <= 10:
                times_charging = 0
                while battery_level <= 40 and times_charging < 6:
                    if self.osManager.getOs() == "main":
                        if any(element in self.configuration.board.lower() for element in ["sf3g", "s3gr", "slt"]):
                            if not self.osManager.gracefulShutdown():
                                self.logger.printLog("WARNING","graceful shutdown cannot be done")
                        else:
                            if not self.osManager.adbRebootCharging():
                                self.logger.printLog("WARNING","reboot in charging mode cannot be done")
                    self.relayCard.usbConnection(False)
                    self.relayCard.usbConnection(True)
                    time.sleep(10)
                    if self.osManager.getOs() == "charger":
                        self.misc.waitDelay("charging board", 1800)
                        times_charging += 1
                        self.logger.printLog("INFO", "the device has booted in charging: {0} times".format(times_charging))
                        exec_status, capacity = self.host.commandExecAdb("shell cat /sys/class/power_supply/*_battery/capacity")
                        if exec_status != 0 or "No such file or directory" in capacity:
                            if not self.osManager.gotoMos():
                                self.logger.printLog("WARNING","go to mos failed")
                                break
                            battery_level = self.device.getDumpsysBattery()[2]
                            battery_level = int(battery_level)
                        else:
                            battery_level = int(capacity)
                    else:
                        self.logger.printLog("WARNING", "reboot in charging mode failed")
                if not self.osManager.gotoMos():
                    self.logger.printLog("WARNING","go to mos failed")
            else:
                self.logger.printLog("INFO","battery charging is not needed because level is more than 10%")
