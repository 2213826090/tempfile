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
@summary: Pupdr Library - Time scripts
@since: 11/03/2015
@author: travenex
"""

from .. import TestCaseModule

class Time(object):

    __instance = None
    __testCase = None
    __globalConf = None

    def __new__(cls):
        """
        Constructor that always returns the same instance
        """
        if not cls.__instance:
            cls.__instance = object.__new__(cls)
        return cls.__instance

    def init(self, globalConf=None):
        self.__globalConf = globalConf
        self.__testCase = TestCaseModule.TestCaseModule()

    def TimeMosS3TestCase(self):
        TestCase = Time.TimeMosScript(self.__globalConf)
        TestCase(S3=True)
        return TestCase.verdict, TestCase.output_dict

    def TimeOffTestCase(self):
        TestCase = Time.TimeOffScript(self.__globalConf)
        TestCase()
        return TestCase.verdict, TestCase.output_dict

    class TimeMosScript(TestCaseModule.TestCaseModule.TC_structure):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_structure.__init__(self, conf)
            if not self.name:
                self.name = "TIME MOS - "
        def runAllSteps(self, S3=False):
            if S3:
                if "_" not in self.name:
                    self.name = "TIME MOS S3 - "
            self.runStep(Time.TimeMosStep, parameter={"S3":S3})

    class TimeOffScript(TestCaseModule.TestCaseModule.TC_structure):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_structure.__init__(self, conf)
            if not self.name:
                self.name = "TIME OFF - "
        def runAllSteps(self, S3=False):
            self.runStep(Time.TimeOffStep)

    class TimeMosStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "Time Diff In MOS"
            self.description = "wait for specific duration and check time drift"
            self.defaultParameter = {"S3": False, "duration": 120}
        def step(self, parameter):
            S3 = parameter["S3"]
            duration = parameter["duration"]
            self.description = "wait for {}s{} and check time drift".format(duration, " in S3" if S3 else "")

            # Set a date and time
            dateTime = self.device.DateTimeClass()
            dateTime.setDateTime(deltaDays=30)

            # if S3_MODE go into sleep mode during DURATION
            if S3:
                self.logger.printLog("INFO", "Go into S3 mode")
                self.osManager.gotoS3(duration, 5)
            else:
                self.misc.waitDelay("wait in MOS", duration)

            # Read date and time
            dateTime.readDateTime()
            if dateTime.uptimeShift == 0:
                self.logger.printLog("INFO", "no time shift (in {0}s)".format(dateTime.timePast))
            else:
                self.output = "device has shifted of {0}s in {1}s (host='{2}', device='{3}')"\
                    .format(dateTime.uptimeShift, dateTime.timePast, dateTime.hostTime, dateTime.deviceTime)
                self.verdict = False

    class TimeOffStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "Time Diff In OFF"
            self.description = "wait for specific duration and check time drift"
            self.defaultParameter = {"duration": 120}
        def step(self, parameter):
            duration = parameter["duration"]
            self.description = "wait for {}s and check time drift".format(duration)

            dateTime = self.device.DateTimeClass()
            dateTime.setDateTime(deltaDays=30)

            #> Force shutdown
            if not self.osManager.forceShutdown(plug=True):
                self.output = "failure to force shutdown to set device in OFF state"
                self.verdict = False
            else:
                self.misc.waitDelay("wait in OFF", duration)
                # boot the board
                self.relayCard.powerButtonPress(self.configuration.timeout.PKPON)
                if not self.osManager.waitOs("main"):
                    self.output = "failure to reboot the board in MOS"
                    self.verdict = False
                else:
                    # Read date and time
                    dateTime.readDateTime()
                    if dateTime.offtimeShift == 0:
                        self.logger.printLog("INFO", "no time shift (in {0}s)".format(dateTime.timePast))
                    else:
                        self.output = "device has shifted of {0}s in {1}s (host='{2}', device='{3}')"\
                            .format(dateTime.offtimeShift, dateTime.timePast, dateTime.hostTime, dateTime.deviceTime)
                        self.verdict = False