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
@summary: Pupdr Library - Usb scripts
@since: 07/01/2015
@author: travenex
"""

import time
from .. import TestCaseModule

class Usb(object):

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

    class UsbUnplugReplugStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "Wait"
            self.description = "wait while USB is connected or disconnected"
            self.defaultParameter = {"duration": 60, "unplug": True, "replug": True}
        def step(self, parameter):
            if parameter["replug"] or parameter["unplug"]:
                if "_" not in self.name:
                    self.name += " And Toggle USB"
            self.description = "wait for {0}s{1}".format(
                parameter["duration"],
                " and plug USB charger{}".format(" back" if parameter["unplug"] else "") if parameter["replug"] else "")
            if parameter["unplug"]:
                self.description = "unplug USB charger then " + self.description
            # unplug USB if required:
            if parameter["unplug"]:
                self.relayCard.usbConnection(False)
            # wait during given duration
            self.misc.waitDelay("wait for defined duration", parameter["duration"])
            # plug back usb if required
            if parameter["replug"]:
                self.relayCard.usbConnection(True)

    class UsbInsertionFromOffStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "Plug USB when OFF"
            self.description = "plug USB charger and wait for COS boot"
        def step(self, parameter):
            # plug USB
            self.relayCard.usbConnection(True)
            # check that board is OFF
            currentOs = self.osManager.getOs()
            if currentOs not in ("offline", "dnx"):
                self.output = "Board not OFF when checking OS state (expected: offline, actual: {0})".format(currentOs)
                self.verdict = False
            # wait for COS boot
            self.misc.waitDelay("wait before checking board is in COS", self.configuration.timeout.COS_BOOT)
            if not self.osManager.waitOs("charger", timeout=self.configuration.timeout.COS_BOOT):
                self.output = "failure to boot COS on charger insertion"
                self.verdict = False

    class PlugAndWaitOsStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "Wait OS Change"
            self.description = "plug USB charger and wait for OS boot"
            self.defaultParameter = {"expectedOs": "main", "timeout": None, "plug": True, "blind": False}
        def step(self, parameter):
            self.description = "wait for '{}' OS detection".format(parameter["expectedOs"])
            if parameter["timeout"]:
                self.description += " for {}s".format(parameter["timeout"])
            if parameter["plug"]:
                self.relayCard.usbConnection(True)
                self.description = "plug USB charger and " + self.description
            if not self.osManager.waitOs(parameter["expectedOs"], timeout=parameter["timeout"], blind=parameter["blind"]):
                self.verdict = False

    class PlugAndGetOsStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "Get Current OS"
            self.description = "plug USB charger and get board state"
            self.defaultParameter = {"expectedOs": "offline", "replug": True, "wait_charger": True}
        def step(self, parameter):
            self.description = "get board state (expected: {0})".format(parameter["expectedOs"])
            if parameter["replug"]:
                self.relayCard.usbConnection(True)
                time.sleep(5)
                self.description = "plug USB charger and " + self.description
            else:
                self.specific = "check board state (expected: {0})".format(parameter["expectedOs"])
            currentOs = self.osManager.getOs()
            if currentOs not in parameter["expectedOs"]:
                self.output = "failure checking board state (actual: {0}, " \
                              "expected: {1})".format(currentOs, parameter["expectedOs"])
                self.verdict = False
            elif parameter["wait_charger"] and not self.configuration.flash.MERRIFIELD and currentOs == "offline":
                self.osManager.waitOs("charger", blind=True, waitWdExp=False)
                self.description += " and wait COS boot"
                currentOs = self.osManager.getOs()
                if not self.configuration.boot.NOCOS and currentOs not in ["offline", "charger"]:
                    self.verdict = False
                    self.output = "failure after USB insertion, unexpected '{}' boot".format(currentOs)