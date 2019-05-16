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
@summary: Pupdr Library - HostModule
@since: 11/17/2014
@author: travenex
"""

import re
import LoggerModule
import MiscModule
import ConfigurationModule

class WorkaroundModule(object):

    __instance = None
    output = None
    __final_output = None
    __content = None
    __dictionary = None
    __globalConf = None
    TC = "TC"
    Function = "Function"
    TestCase = "TestCase"
    Step = "Step"
    __logger = None
    __misc = None
    __configuration = None

    def __new__(cls):
        """
        Constructor that always returns the same instance
        """
        if not cls.__instance:
            cls.__instance = object.__new__(cls)
        return cls.__instance

    def init(self, globalConf=None):
        self.__globalConf = globalConf
        self.output         = ""
        self.__final_output = ""
        self.__content      = dict()
        self.__dictionary   = dict()
        self.__logger = LoggerModule.LoggerModule()
        self.__misc = MiscModule.MiscModule()
        self.__configuration = ConfigurationModule.ConfigurationModule()

    def isWorkaround(self, data_type, name="", output="", check_skip_only=False):
        log = "Workaround.isWorkaround() "
        is_workaround = False
        workaround_log = ""
        allowed_data_type = (self.TC, self.Step, self.TestCase, self.Function)
        if data_type not in allowed_data_type:
            self.__logger.printLog("WARNING", log + "invalid data type '{0}' not in: {1}".format(data_type, ", ".join(allowed_data_type)))
            return False, "invalid data type '{0}'".format(data_type)
        if not name:
            name = self.__misc.getCaller(2)
        self.__logger.printLog("INFO", log + "start with name: {0}{1}".format(name, " (output=\"{0}\")".format(output) if output else ""))
        if data_type in self.__configuration.workaround.DATA:
            for element in self.__configuration.workaround.DATA[data_type]:
                if element["Name"] == name:
                    if element["Skip"]:
                        is_workaround = True
                        workaround_log = "skipping '{0}' element because of issue: \"{1}\" (from {2} configuration)".format(name, element["Issue"], element["Origin"])
                    elif element["Pattern"] and not check_skip_only:
                        is_workaround = re.search(element["Pattern"], output)
                        if is_workaround:
                            workaround_log = "pattern \"{0}\" for issue \"{1}\" found in: \"{2}\" (from {3} configuration)".format(element["Pattern"], element["Issue"], output, element["Origin"])
        if is_workaround:
            self.__logger.printLog("INFO", log + "!!! WORKAROUND FOUND !!! " + workaround_log)

        return is_workaround, workaround_log