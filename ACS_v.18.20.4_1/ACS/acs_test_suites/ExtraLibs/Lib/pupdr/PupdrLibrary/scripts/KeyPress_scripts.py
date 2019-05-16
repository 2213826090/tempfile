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
@summary: Pupdr Library - KeyPress scripts
@since: 15/01/2015
@author: travenex
"""

from .. import TestCaseModule
from .. import ConfigurationModule
import Check_scripts
import Usb_scripts
import MOS_scripts
import COS_scripts

class KeyPress(object):

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

    def Pkp2sWithoutUsbTestCase(self):
        TestCase = KeyPress.PowerKeyPressScript(self.__globalConf)
        TestCase(pkpDuration=2, plug=False)
        return TestCase.verdict, TestCase.output_dict

    def Pkp4sWithUsbTestCase(self):
        TestCase = KeyPress.PowerKeyPressScript(self.__globalConf)
        TestCase(pkpDuration=4, plug=True)
        return TestCase.verdict, TestCase.output_dict

    def PkpOffWithoutUsbTestCase(self):
        TestCase = KeyPress.PowerKeyPressOffScript(self.__globalConf)
        TestCase(plug=False)
        return TestCase.verdict, TestCase.output_dict

    def PkpOnWithUsbTestCase(self):
        TestCase = KeyPress.PowerKeyPressOnScript(self.__globalConf)
        TestCase(plug=True)
        return TestCase.verdict, TestCase.output_dict

    def PkpOnWithoutUsbTestCase(self):
        TestCase = KeyPress.PowerKeyPressOnScript(self.__globalConf)
        TestCase(plug=False)
        return TestCase.verdict, TestCase.output_dict

    def ColdBootTestCase(self):
        TestCase = KeyPress.coldBootScript(self.__globalConf)
        TestCase(plug=False)
        return TestCase.verdict, TestCase.output_dict

    class coldBootScript(TestCaseModule.TestCaseModule.TC_structure):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_structure.__init__(self, conf)
            if not self.name:
                self.name = "PKP ON - "
        def runAllSteps(self, plug=True):
            pkpDuration = self.configuration.timeout.PKPON
            if not self.name:
                self.name = "{0}S {1}PLUG PKP - ".format(pkpDuration, "" if plug else "UN")
            key_list = ["SwitchOnOff"]
            if not plug:
                key_list.append("UsbHostPcConnect")
            if "cht" in self.configuration.board:
                timeout = self.device.getOsExtentedTimeout("main") + 200
            else:
                timeout = None
            self.runStep(COS_scripts.COS.BatteryChargingStep)
            self.runStep(KeyPress.CheckRelayStep,
                             parameter={"key_list": key_list})
            self.runStep(MOS_scripts.MOS.CheckBootTimeStep)
            current_boot_time = self.globalConf["CURRENT_BOOT_TIME"]
            self.runStep(KeyPress.ForceShutdownStep, parameter={"plug": plug})
            self.runStep(KeyPress.PowerKeyPressStep, parameter={"delay": pkpDuration})
            self.runStep(Usb_scripts.Usb.PlugAndWaitOsStep,
                         parameter={
                             "expectedOs": "main",
                             "plug": not plug,
                             "timeout": timeout})
            if self.configuration.boot.BOOT_COS_ON_FORCED_SHUTDOWN_PLUG and plug:
                self.runStep(Check_scripts.Check.MosCheckAfterRebootStep)
            else:
                self.runStep(Check_scripts.Check.MosCheckAfterBootStep)
            self.runStep(MOS_scripts.MOS.CheckBootTimeStep,
                         parameter={"compare_with": current_boot_time, "match": False})
            self.runStep(Check_scripts.Check.CheckColdBootDoneStep)

    class PowerKeyPressOnScript(TestCaseModule.TestCaseModule.TC_structure):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_structure.__init__(self, conf)
            if not self.name:
                self.name = "PKP ON - "
        def runAllSteps(self, plug=True):
            pkpDuration = self.configuration.timeout.PKPON
            if not self.name:
                self.name = "{0}S {1}PLUG PKP - ".format(pkpDuration, "" if plug else "UN")
            key_list = ["SwitchOnOff"]
            if not plug:
                key_list.append("UsbHostPcConnect")
            if "cht" in self.configuration.board:
                timeout = self.device.getOsExtentedTimeout("main") + 200
            else:
                timeout = None
            self.runStep(KeyPress.CheckRelayStep,
                             parameter={"key_list": key_list})
            self.runStep(MOS_scripts.MOS.CheckBootTimeStep)
            current_boot_time = self.globalConf["CURRENT_BOOT_TIME"]
            self.runStep(KeyPress.ForceShutdownStep, parameter={"plug": plug})
            self.runStep(KeyPress.PowerKeyPressStep, parameter={"delay": pkpDuration})
            self.runStep(Usb_scripts.Usb.PlugAndWaitOsStep,
                         parameter={
                             "expectedOs": "main",
                             "plug": not plug,
                             "timeout": timeout})
            if self.configuration.boot.BOOT_COS_ON_FORCED_SHUTDOWN_PLUG and plug:
                self.runStep(Check_scripts.Check.MosCheckAfterRebootStep)
            else:
                self.runStep(Check_scripts.Check.MosCheckAfterBootStep)
            self.runStep(MOS_scripts.MOS.CheckBootTimeStep,
                         parameter={"compare_with": current_boot_time, "match": False})

    class PowerKeyPressOffScript(TestCaseModule.TestCaseModule.TC_structure):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_structure.__init__(self, conf)
            if not self.name:
                self.name = "PKP OFF - "
        def runAllSteps(self, plug=False):
            pkpDuration = self.configuration.timeout.PKPOFF
            if not self.name:
                self.name = "{0}S UNPLUG PKP - ".format(pkpDuration)
            self.runStep(KeyPress.ForceShutdownStep, parameter={"plug": plug})
            self.runStep(KeyPress.PowerKeyPressStep, parameter={"delay": pkpDuration})
            if not self.configuration.boot.BOOT_ON_FORCED_SHUTDOWN:
                # if board not unplugged, do not toggle USB
                self.runStep(Usb_scripts.Usb.PlugAndGetOsStep, parameter={"expectedOs": ("offline", "dnx"), "replug": not plug})
                self.runStep(KeyPress.ForceShutdownStep, parameter={"plug": True})
                self.runStep(KeyPress.PowerKeyPressStep, parameter={"delay": self.configuration.timeout.PKPON})
                if self.configuration.board in ["cht_cr_rvp"]:
                    timeout = self.device.getOsExtentedTimeout("main") + 200
                else:
                    timeout = None
                self.runStep(Usb_scripts.Usb.PlugAndWaitOsStep, parameter={"expectedOs":"main", "plug":False, "timeout":timeout})
                if self.configuration.boot.BOOT_COS_ON_FORCED_SHUTDOWN_PLUG:
                    self.runStep(Check_scripts.Check.MosCheckAfterRebootStep)
                else:
                    self.runStep(Check_scripts.Check.MosCheckAfterBootStep)
            else:
                self.runStep(Usb_scripts.Usb.UsbUnplugReplugStep, parameter={"unplug": False, "duration": self.device.getOsExtentedTimeout("main"), "replug": False})
                self.runStep(Usb_scripts.Usb.PlugAndGetOsStep, parameter={"expectedOs": ("offline", "main"), "replug": not plug})
                if self.osManager.getOs() != "main":
                    self.runStep(KeyPress.ForceShutdownStep, parameter={"plug": False})
                    self.runStep(KeyPress.PowerKeyPressStep, parameter={"delay": self.configuration.timeout.PKPON})
                    self.runStep(Usb_scripts.Usb.UsbUnplugReplugStep,
                                 parameter={"duration": self.configuration.timeout.ROS_EXPIRATION+self.configuration.timeout.MOS_BOOT, "replug": False})
                    self.runStep(Usb_scripts.Usb.PlugAndWaitOsStep, parameter={"expectedOs": "main", "plug": True})
                self.runStep(Check_scripts.Check.MosCheckAfterBootStep)

    class PowerKeyPressScript(TestCaseModule.TestCaseModule.TC_structure):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_structure.__init__(self, conf)
            if not self.name:
                self.name = "PKP - "
        def runAllSteps(self, pkpDuration=3, plug=True):
            if plug:
                if "_" not in self.name:
                    self.name = "{0}S PLUG PKP - ".format(pkpDuration)
            else:
                if "_" not in self.name:
                    self.name = "{0}S UNPLUG PKP - ".format(pkpDuration)
            self.runStep(KeyPress.ForceShutdownStep, parameter={"plug": plug})
            self.runStep(Usb_scripts.Usb.UsbUnplugReplugStep,
                         parameter={"duration": 60, "unplug": False, "replug": False})
            self.runStep(KeyPress.PowerKeyPressStep, parameter={"delay": pkpDuration})
            if not self.configuration.boot.BOOT_ON_FORCED_SHUTDOWN and pkpDuration >= self.configuration.timeout.PKPOFF:
                # if board not unplugged, do not toggle USB
                self.runStep(Usb_scripts.Usb.PlugAndGetOsStep, parameter={"expectedOs": ("offline", "dnx"), "replug": not plug})
                self.runStep(KeyPress.ForceShutdownStep, parameter={"plug": True})
                self.runStep(KeyPress.PowerKeyPressStep, parameter={"delay": self.configuration.timeout.PKPON})
                # TODO: remove WA (due to IMINAN-35862)
                if self.configuration.board in ["cht_cr_rvp"]:
                    timeout = self.device.getOsExtentedTimeout("main") + 200
                else:
                    timeout = None
                self.runStep(Usb_scripts.Usb.PlugAndWaitOsStep, parameter={"expectedOs":"main", "plug":False,
                                                                              "timeout":timeout})
            else:
                self.runStep(Usb_scripts.Usb.UsbUnplugReplugStep, parameter={"unplug": False, "duration": self.device.getOsExtentedTimeout("main"), "replug": False})
                if pkpDuration >= self.configuration.timeout.PKPON:
                    self.runStep(Usb_scripts.Usb.PlugAndWaitOsStep, parameter={"expectedOs": "main", "plug": True, "timeout": 20})
                else:
                    self.runStep(Usb_scripts.Usb.PlugAndGetOsStep, parameter={"expectedOs": ("offline", "main"), "replug": not plug})
                    if self.osManager.getOs() != "main":
                        self.runStep(KeyPress.ForceShutdownStep, parameter={"plug": False})
                        self.runStep(KeyPress.PowerKeyPressStep, parameter={"delay": self.configuration.timeout.PKPON})
                        self.runStep(Usb_scripts.Usb.UsbUnplugReplugStep, parameter={"duration": self.configuration.timeout.ROS_EXPIRATION+self.configuration.timeout.MOS_BOOT, "replug": False})
                        self.runStep(Usb_scripts.Usb.PlugAndWaitOsStep, parameter={"expectedOs": "main", "plug": True})
            if self.configuration.boot.BOOT_COS_ON_FORCED_SHUTDOWN_PLUG and plug:
                self.runStep(Check_scripts.Check.MosCheckAfterRebootStep)
            else:
                self.runStep(Check_scripts.Check.MosCheckAfterBootStep)

    class ForceShutdownStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "Force Shutdown"
            self.description = "use long power key press to shutdown the board"
            self.defaultParameter = {"delay": None, "plug": True}
        def step(self, parameter):
            self.description = "press power key for {}s while USB charger is {}".format(
                parameter["delay"] if parameter["delay"] is not None else self.configuration.timeout.PKPOFF,
                "plugged in" if parameter["plug"] else "unplugged"
            )
            self.verdict = self.osManager.forceShutdown(delay=parameter["delay"], plug=parameter["plug"])
            if self.configuration.boot.BOOT_COS_ON_FORCED_SHUTDOWN_PLUG and parameter["plug"]:
                self.description += " and wait for charger boot"
                self.osManager.waitOs("charger")

    class PowerKeyPressStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "Power Key Press"
            self.description = "use power key press to change board state"
            self.defaultParameter = {"delay": self.configuration.timeout.PKPOFF}
        def step(self, parameter):
            self.description = "press power key for {0}s".format(parameter["delay"])
            # additional PKP 1s for BYT_T_CRV2_64 PMIC XPOWER
            if any (element in self.configuration.board for element in ["byt_t_cr", "byt_cr"])\
                and parameter["delay"] >= self.configuration.timeout.PKPOFF:
                self.relayCard.powerButtonPress(self.configuration.timeout.PKPON)
            self.relayCard.powerButtonPress(parameter["delay"])

    class CheckRelayStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "Check Relay Card"
            self.description = "checking relay card setup"
            self.defaultParameter = {"key_list": []}
        def step(self, parameter):
            key_list = parameter["key_list"]
            if not key_list:
                self.logger.printLog("INFO", "nothing to check")
                return
            self.description += ": {}".format(", ".join(key_list))
            for key in key_list:
                if not self.relayCard.relayConfiguration or not self.relayCard.relayConfiguration.get(key):
                    self.verdict = False
                    self.output = "missing '{}' entry relay card configuration".format(key)
                    self.logger.printLog("DEBUG", "current relay card configuration = {}".format())
                    break