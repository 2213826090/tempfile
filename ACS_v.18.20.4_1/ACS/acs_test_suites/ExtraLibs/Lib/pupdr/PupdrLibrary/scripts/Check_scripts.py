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

class Check(object):

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

    class MosCheckAfterRebootStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "Check MOS Reboot Variables"
            self.description = "check boot vars after reboot in MOS"
        def step(self, parameter):
            self.verdict, self.output = self.bootData.bootDataCheck(self.configuration.boot_vars.MOS_CHECK_AFTER_REBOOT)
            if not self.verdict:
                self.output = ""

    class RosCheckAfterRebootStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "Check ROS Reboot Variables"
            self.description = "check boot vars after reboot in MOS"
        def step(self, parameter):
            self.verdict, self.output = self.bootData.bootDataCheck(self.configuration.boot_vars.ROS_CHECK_AFTER_REBOOT)
            if not self.verdict:
                self.output = ""

    class MosCheckAfterBootStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "Check MOS Boot Variables"
            self.description = "check boot vars after boot on power key in MOS"
        def step(self, parameter):
            self.verdict, self.output = self.bootData.bootDataCheck(self.configuration.boot_vars.MOS_CHECK_AFTER_BOOT)
            if not self.verdict:
                self.output = ""

    class CosCheckAfterRebootStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "Check COS Reboot Variables"
            self.description = "check boot vars after reboot in COS"
        def step(self, parameter):
            self.verdict, self.output = self.bootData.bootDataCheck(self.configuration.boot_vars.COS_CHECK_AFTER_REBOOT)
            if not self.verdict:
                self.output = ""

    class CosCheckAfterHardShutdownStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "Check COS Boot After Hard Shutdown Variables"
            self.description = "check boot vars in COS after hard shutdown"
        def step(self, parameter):
            self.verdict, self.output = self.bootData.bootDataCheck(self.configuration.boot_vars.COS_CHECK_AFTER_HARD_SHUTDOWN)
            if not self.verdict:
                self.output = ""

    class CosCheckAfterGracefulShutdownStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "Check COS Boot After Graceful Shutdown Variables"
            self.description = "check boot vars in COS after graceful shutdown"
        def step(self, parameter):
            self.verdict, self.output = self.bootData.bootDataCheck(self.configuration.boot_vars.COS_CHECK_AFTER_GRACEFUL_SHUTDOWN)
            if not self.verdict:
                self.output = ""

    class MosCheckAfterKernelWatchdogStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "Check MOS Boot After Kernel WD Variables"
            self.description = "check boot vars in MOS after kernel watchdog"
        def step(self, parameter):
            self.verdict, self.output = self.bootData.bootDataCheck(self.configuration.boot_vars.MOS_CHECK_AFTER_KERNEL_WATCHDOG)
            if not self.verdict:
                self.output = ""

    class MosCheckAfterSecurityWatchdogStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "Check MOS Boot After Security WD Variables"
            self.description = "check boot vars in MOS after security watchdog"
        def step(self, parameter):
            self.verdict, self.output = self.bootData.bootDataCheck(self.configuration.boot_vars.MOS_CHECK_AFTER_SECURITY_WATCHDOG)
            if not self.verdict:
                self.output = ""

    class RosCheckAfterWatchdogStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "Check ROS Boot After WD Variables"
            self.description = "check boot vars in ROS after watchdog expiration"
        def step(self, parameter):
            self.verdict, self.output = self.bootData.bootDataCheck(self.configuration.boot_vars.ROS_CHECK_AFTER_WATCHDOG)
            if not self.verdict:
                self.output = ""

    class MosCheckAfterWatchdogFallbackStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "Check MOS Boot After WD Expiration Variables"
            self.description = "check boot vars in ROS after watchdog expiration"
        def step(self, parameter):
            self.verdict, self.output = self.bootData.bootDataCheck(self.configuration.boot_vars.MOS_CHECK_AFTER_WATCHDOG_FALLBACK)
            if not self.verdict:
                self.output = ""

    class MosCheckAfterRebootFromPosStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "Check MOS Reboot From POS Variables"
            self.description = "check boot vars in MOS after reboot from POS"
        def step(self, parameter):
            self.verdict, self.output = self.bootData.bootDataCheck(self.configuration.boot_vars.MOS_CHECK_AFTER_REBOOT_FROM_POS)
            if not self.verdict:
                self.output = ""

    class PosCheckAfterRebootStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "Check POS Boot Variables"
            self.description = "check boot vars in POS after 'adb reboot bootloader' command"
        def step(self, parameter):
            if self.configuration.boot_vars.POS_CHECK_AFTER_REBOOT:
                if not self.osManager.fastboot2adb()[0]:
                    self.verdict = False
                else:
                    self.verdict, self.output = self.bootData.bootDataCheck(self.configuration.boot_vars.POS_CHECK_AFTER_REBOOT)
                    if self.verdict:
                        self.verdict = self.osManager.adb2fastboot()[0]
                    if not self.verdict:
                        self.output = ""

    class CheckColdBootDoneStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "Check cold boot done"
            self.description = "check cold boot done and cold reset time"
        def step(self, parameter):
            self.verdict = True
            self.output = self.device.coldBootDone()
