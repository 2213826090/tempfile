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
@summary: Pupdr Library - DeviceModule
@since: 11/17/2014
@author: travenex
"""

from random import getrandbits
import LoggerModule
import HostModule

class AcsApk():
    """ Class that cover interactions with ACS agent APk
    """

    def __init__(self):
        self.__logger = LoggerModule.LoggerModule()
        self.__host = HostModule.HostModule()

    def isRunning(self):
        exec_status, output = self.__host.commandExecAdb("shell 'ps | grep com.intel.acs.agentv2.service'")
        if not exec_status and not output:
            # Try to launch the service
            exec_status, output = self.__host.commandExecAdb("shell am start -n com.intel.acs.agentv2/.common.framework.ServiceStarterActivity")
        return not exec_status and output

    def gshtnIntent(self, delay=5):
        opcode = str(getrandbits(32))
        classe = "acscmd.utils.RebootCmd"
        method = "GracefulShutdown"
        param  = "delay"

        # Clear the logcat
        self.__host.commandExecAdb("logcat -c")

        # Sending the intent
        exec_status, output = self.__host.commandExecAdb("shell am broadcast -a intel.intent.action.acs.cmd \
            -e opcode %s -e class %s -e method %s --ei %s %s" % (opcode, classe, method, param, delay))

        if exec_status != 0:
            self.__logger.printLog("WARNING", "Fail to send intent")
            return False

        # Check the shutdown procedure is started
        exec_status, output = self.__host.commandExecAdb("logcat -d", 3)
        if not exec_status and "opcode::%s - status::SUCCESS - output::RECEIVED - function::GracefulShutdown" % opcode in output:
            if "opcode::%s - status::SUCCESS - output::Delayed graceful shutdown recieved" % opcode in output:
                self.__logger.printLog("INFO", "Gracefull shutdown request sent")
            else:
                self.__logger.printLog("WARNING", "Fail to sent the Graceful shutdown request")
                return False
            if "opcode::%s - status::FAILURE - exception_message::" in output:
                self.__logger.printLog("WARNING", "Your ACS agent is out to date")
                return False
        else:
            self.__logger.printLog("WARNING", "Fail to initiate graceful shutdown through acsagent: %s\n%s" % (str(exec_status), output))
            return False

        return True