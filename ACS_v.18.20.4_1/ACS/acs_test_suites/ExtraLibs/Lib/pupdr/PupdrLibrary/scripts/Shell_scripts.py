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
@summary: Pupdr Library - Shell scripts
@since: 10/04/2015
@author: travenex
"""

import os
import re
import time
import tempfile
from .. import TestCaseModule

class Shell(object):

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

    def QCTVWarmRebootTestCase(self):
        TestCase = Shell.QCTVWarmRebootScript(self.__globalConf)
        TestCase()
        return TestCase.verdict, TestCase.output_dict

    def QCTVColdBootTestCase(self):
        TestCase = Shell.QCTVColdBootScript(self.__globalConf)
        TestCase()
        return TestCase.verdict, TestCase.output_dict

    class QCTVWarmRebootScript(TestCaseModule.TestCaseModule.TC_structure):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_structure.__init__(self, conf)
            if not self.name:
                self.name = "QCTV WARM REBOOT - "
        def runAllSteps(self):
            self.runStep(Shell.LaunchQCTVWarmRebootShellScriptStep)

    class QCTVColdBootScript(TestCaseModule.TestCaseModule.TC_structure):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_structure.__init__(self, conf)
            if not self.name:
                self.name = "QCTV COLD BOOT - "
        def runAllSteps(self):
            self.runStep(Shell.LaunchQCTVColdBootShellScriptStep)

    class LaunchQCTVColdBootShellScriptStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "QCTV Cold Boot"
            self.description = "launch QCTV cold boot script"
        def step(self, parameter):
            path_to_script = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "Extras", "QCTV_cold_boot_script.sh"))
            serial_number = None
            index = 0
            while not serial_number and index < 30:
                device = self.host.commandExecAdb("devices", 20)[1]
                search = re.search("([A-Z0-9][A-Z0-9][A-Z0-9][A-Z0-9]+).*device", device)
                if search:
                    serial_number = search.group(1)
                index += 1
                time.sleep(5)
            if not serial_number:
                self.output = "impossible to get serial number"
                self.verdict = False
            else:
                if self.logs.tc_report_path:
                    log_file = os.path.join(self.logs.tc_report_path, "script_output.log")
                else:
                    log_file = os.path.join(tempfile.gettempdir(), "script_output.log")
                relay_card_tty = self.relayCard.cardConfiguration.get("ComPort", "/dev/ttyRelayCard")
                power_key_index = self.relayCard.relayConfiguration.get("SwitchOnOff", 0)
                if not relay_card_tty or power_key_index == 0:
                    self.output = "incorrect relay card configuration (tty='{0}', power_key={1})".format(relay_card_tty, power_key_index)
                    self.verdict = False
                elif not os.path.exists(relay_card_tty):
                    self.output = "incorrect relay card configuration (unknown tty={0})".format(relay_card_tty)
                    self.verdict = False
                else:
                    cmdline = "{0} {1} 1000 {2} {3} {4}".format(path_to_script, serial_number, log_file, relay_card_tty, power_key_index)
                    # command has 16 hours to be executed and will be killed if inactive for 15 minutes
                    exec_status, output = self.host.commandExecActivityScan(cmdline, timeout=3600*32, scan_inactivity_timeout=60*15)
                    if exec_status != 0:
                        self.verdict = False

    class LaunchQCTVWarmRebootShellScriptStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "QCTV Warm Reboot"
            self.description = "launch QCTV warm reboot script"
        def step(self, parameter):
            path_to_script = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "Extras", "QCTV_warm_reboot_script.sh"))
            serial_number = None
            index = 0
            while not serial_number and index < 30:
                device = self.host.commandExecAdb("devices", 20)[1]
                search = re.search("([A-Z0-9][A-Z0-9][A-Z0-9][A-Z0-9]+).*device", device)
                if search:
                    serial_number = search.group(1)
                index += 1
                time.sleep(5)
            if not serial_number:
                self.output = "impossible to get serial number"
                self.verdict = False
            else:
                if self.logs.tc_report_path:
                    log_file = os.path.join(self.logs.tc_report_path, "script_output.log")
                else:
                    log_file = os.path.join(tempfile.gettempdir(), "script_output.log")
                cmdline = "{0} {1} 1000 {2}".format(path_to_script, serial_number, log_file)
                # command has 16 hours to be executed and will be killed if inactive for 15 minutes
                exec_status, output = self.host.commandExecActivityScan(cmdline, timeout=3600*32, scan_inactivity_timeout=60*15)
                if exec_status != 0:
                    self.verdict = False