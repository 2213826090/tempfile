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
@summary: Pupdr Library - RelayCardModule
@since: 11/17/2014
@author: travenex
"""

import time
import MiscModule
import LoggerModule
import HostModule

class OutputModule(object):

    __instance = None
    __outputList = list()
    __formattedOutput = None
    __failuresCallStackList = None
    __stack = None
    __globalConf = None
    __logger = None
    __host = None
    __misc = None

    def __new__(cls):
        """
        Constructor that always returns the same instance
        """
        if not cls.__instance:
            cls.__instance = object.__new__(cls)
        return cls.__instance

    def init(self, globalConf=None):
        self.__globalConf = globalConf
        self.__logger = LoggerModule.LoggerModule()
        self.__host = HostModule.HostModule()
        self.__outputList = list()
        self.__misc = MiscModule.MiscModule()
        self.__formattedOutput = ""
        self.__failuresCallStackList = list()
        self.__stack = list()

    def __innerPrintFunction(self, index, indent):
        output = ""
        for _ in range(indent):
            output += ".  "
        output += self.__outputList[index]["method"]
        if self.__outputList[index]["argument"]:
            output += "(" + self.__outputList[index]["argument"] + ")"
        if self.__outputList[index]["output"]:
            output += " : " + self.__outputList[index]["output"]
        if self.__outputList[index]["verdict"] is not None:
            if self.__outputList[index]["verdict"]:
                output += " (verdict: PASS)"
            else:
                output += " (verdict: FAIL)"
        if indent == 0:
            output += " (called by: {0}".format(self.__outputList[index]["caller"]) + ")"
        self.__formattedOutput += output + "\n"
        if self.__outputList[index].get("hasCalled"):
            for subIndex in self.__outputList[index]["hasCalled"]:
                self.__innerPrintFunction(subIndex, indent+1)

    def printFormattedOutput(self):
        # remove elements from init and finalStep (not critical to TC verdict)
        doNotDisplay = ["init", "setup", "finalStep"]
        self.__formattedOutput = "TC execution stack\n"
        for index in [index for (index, element) in enumerate(self.__outputList) if element["caller_index"] == -1 and element["method"] not in doNotDisplay]:
            self.__innerPrintFunction(index, 0)
        self.__logger.printLog("INFO", self.__formattedOutput)

    def __getFullErrorStack(self, index):
        self.__stack.append(index)
        if self.__outputList[index].get("hasCalled"):
            for subIndex in sorted(self.__outputList[index]["hasCalled"], reverse=True):
                if self.__outputList[subIndex]["verdict"] is False:
                    self.__getFullErrorStack(subIndex)

    def __stack2OutputString(self, stack):
        output = ""
        for index in sorted(stack, reverse=False):
            singleError = self.__outputList[index]["method"]
            if self.__outputList[index]["argument"]:
                singleError += "(" + self.__outputList[index]["argument"] + ")"
            singleError += ": " + self.__outputList[index]["output"]
            if output:
                output = " | ".join((output, singleError))
            else:
                output = singleError
        return output

    def __stack2OutputList(self, stack):
        outputList = []
        for index in sorted(stack, reverse=False):
            singleError = self.__outputList[index]["method"]
            if self.__outputList[index]["argument"]:
                singleError += "(" + self.__outputList[index]["argument"] + ")"
            singleError += ": " + self.__outputList[index]["output"]
            outputList.append(singleError)
        return outputList

    def getErrorCallStacks(self):
        self.__failuresCallStackList = list()
        for index in [index for (index, element) in enumerate(self.__outputList) if (element["caller_index"] == -1 and element["verdict"] is False)]:
            self.__stack = list()
            self.__getFullErrorStack(index)
            self.__failuresCallStackList.append(self.__stack)
        if not self.__failuresCallStackList:
            self.__logger.printLog("INFO", "no stack list found")
        else:
            for index, stack in enumerate(self.__failuresCallStackList):
                self.__logger.printLog("INFO", "stack: " + str(stack))
                output = self.__stack2OutputString(stack)
                self.__logger.printLog("WARNING", "failure stack {0}:\n".format(index) + output)
        if self.__failuresCallStackList:
            returnOutput = "ERROR AT {0} | ".format(self.__outputList[self.__failuresCallStackList[-1][-1]]["timestamp"]) + self.__stack2OutputString(self.__failuresCallStackList[-1])
            returnOutputList = ["ERROR AT {0}".format(self.__outputList[self.__failuresCallStackList[-1][-1]]["timestamp"])]
            returnOutputList.extend(self.__stack2OutputList(self.__failuresCallStackList[-1]))
        else:
            returnOutput = ""
            returnOutputList = []
        return returnOutput, returnOutputList

    def printRawOutput(self):
        print str(self.__outputList)
        for element in self.__outputList:
            self.__logger.printLog("INFO", str(element))

    def appendOutput(self, output, verdict, argument=""):
        # check verdict in True, False
        if verdict not in (True, False, None):
            self.__logger.printLog("WARNING", "appendOutput(): non boolean verdict")
        else:
            # create new element
            currentElement = {"verdict": verdict,
                              "output": output.replace("\n", " - "),
                              "method": self.__misc.getCaller(2).replace("__", ""),
                              "caller": self.__misc.getCaller(3).replace("__", ""),
                              "argument": argument,
                              "caller_index": -1,
                              "hasCalled": list(),
                              "timestamp": time.strftime("%m/%d %H:%M:%S.") + str(int(time.time()*1000))[-3:]}
            # look for links between called methods
            if len(self.__outputList) >= 1:
                for index, element in enumerate(self.__outputList):
                    if currentElement["method"] == element["caller"] and\
                        element["caller_index"] == -1:
                        element["caller_index"] = len(self.__outputList)
                        currentElement["hasCalled"].append(index)
            # save new output
            self.__outputList.append(currentElement)
            # print it
            if self.__outputList[-1]["output"]:
                printOutput = self.__outputList[-1]["method"] + "(" + self.__outputList[-1]["argument"] + "): " + self.__outputList[-1]["output"]
            else:
                printOutput = self.__outputList[-1]["method"] + "(" + self.__outputList[-1]["argument"] + ")"
            if verdict:
                self.__logger.printLog("INFO", printOutput + " --> SUCCESS")
            elif verdict is None:
                self.__logger.printLog("INFO", printOutput + " --> DONE")
            else:
                self.__logger.printLog("WARNING", printOutput + " --> FAIL (could not impact Test Case verdict)")