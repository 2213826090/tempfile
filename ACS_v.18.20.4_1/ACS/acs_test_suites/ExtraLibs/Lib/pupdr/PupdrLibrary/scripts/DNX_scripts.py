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
@summary: Pupdr Library - DNX scripts
@since: 11/03/2015
@author: travenex
"""

from .. import TestCaseModule
from .. import ConfigurationModule
import Reboot_scripts
import Check_scripts
import KeyPress_scripts
import Usb_scripts

class DNX(object):

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

    def AdbRebootDnxTestCase(self):
        TestCase = DNX.AdbRebootDnxScript(self.__globalConf)
        TestCase()
        return TestCase.verdict, TestCase.output_dict

    class AdbRebootDnxScript(TestCaseModule.TestCaseModule.TC_structure):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_structure.__init__(self, conf)
            if not self.name:
                self.name = "DNX BOOT - "
        def runAllSteps(self):
            if not self.configuration.flash.DEDIPROG:
                output = "board has no fastboot-dnx OS"
                self.logger.printLog("INFO", output)
                self.skipping = True
                self.toggleVerdict(self.VALID)
                self.output_dict["valid_log"] = output
            self.runStep(Reboot_scripts.Reboot.AdbRebootDnxStep)
            if "bxtp" in self.configuration.board:
                self.runStep(Reboot_scripts.Reboot.RebootBxtpMrbStep)
            else:
                self.runStep(KeyPress_scripts.KeyPress.ForceShutdownStep,
                             parameter={"plug": False})
                self.runStep(KeyPress_scripts.KeyPress.PowerKeyPressStep,
                             parameter={"delay": self.configuration.timeout.PKPON})
                self.runStep(Usb_scripts.Usb.PlugAndWaitOsStep)
            self.runStep(Check_scripts.Check.MosCheckAfterBootStep)