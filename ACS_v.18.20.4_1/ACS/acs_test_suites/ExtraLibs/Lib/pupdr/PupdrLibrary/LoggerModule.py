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
@summary: Pupdr Library - LoggerModule
@since: 11/17/2014
@author: travenex
"""

import time

class LoggerModule(object):

    __instance = None
    __globalConf = None
    __isExternal = None
    testStep = None

    def __new__(cls):
        """
        Constructor that always returns the same instance
        """
        if not cls.__instance:
            cls.__instance = object.__new__(cls)
        return cls.__instance

    def init(self, globalConf=None, externalLogger=False):
        self.__globalConf = globalConf
        if externalLogger:
            self.__isExternal = True
        else:
            self.__isExternal = False
        self.testStep = ""

    @staticmethod
    def __internalLogger(logtype, content):
        print "{0:19} BOOTOTA   {1:7} {2}".format(time.strftime("%m/%d %H:%M:%S.") + str(int(time.time()*1000))[-3:], logtype, content)

    # function called to print logs
    def printLog(self, logType, content, frontContent="", raw=False):
        # check logType input
        logTypeList = ("ERROR", "INFO", "DEBUG", "WARNING")
        if logType not in logTypeList:
            print "ERROR - LoggerModule.printLog: invalid logtype '%s' not in '%s'" \
                  % (logType, ", ".join(logTypeList))
        else:
            if frontContent != "":
                fullContent = " ".join((frontContent, content))
            else:
                fullContent = content
            if self.testStep:
                testStep = self.testStep
            elif self.__globalConf.get("DEFAULT_INIT", False):
                testStep = "FWK INIT - "
            else:
                testStep = ""
            if testStep and not raw:
                if testStep.endswith(" "):
                    testStep = testStep[:-1]
                fullContent = " ".join((testStep, fullContent))
            if self.__isExternal:
                self.__globalConf["EXTERNAL_LOGGER"](logType, fullContent)
            else:
                self.__internalLogger(logType, fullContent)