#!/usr/bin/env python
# -*- coding: utf-8 -*-
# pylint: disable=missing-docstring, invalid-name, unused-argument
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
@summary: Dumps soc debug info on test failure using PythonSV scripts
@since: 3/5/14
@author: ahkhowaj
"""

import os
import sys
from Device.Module.DeviceModuleBase import DeviceModuleBase
from acs_test_scripts.Device.Module.Common.Debug.IDebugModule import IDebugModule
from UtilitiesFWK.Utilities import Verdict
from Device.DeviceManager import DeviceManager

class SocDebugModule(IDebugModule, DeviceModuleBase):
    """
    Implement a module to handle soc debug
    """

    def __init__(self):
        super(SocDebugModule, self).__init__()

    def init(self, device_serial_number):
        return

    def dump(self, verdict, dump_location):
        """
        Dump all soc debug information
        :type verdict: str
        :param verdict: test status
        :type dump_location: str
        :param dump_location: directory where data will be stored
        """

        if verdict != Verdict.PASS:
            dm = DeviceManager()
            dumpDebugLogs = False
            # read dumpDebugLogs from module config file
            try:
                dumpDebugLogs = eval(str(self.configuration.dumpDebugLogs))
            except AttributeError:
                self.logger.error("missing dumpDebugLogs param from module config." )
                return -1
            self.logger.info("dumpDebugLogs [%s]." % dumpDebugLogs)
            if dumpDebugLogs:
                if sys.platform == "win32":
                    Soc = ""
                    # read Soc from module config file
                    try:
                        Soc = str(self.configuration.Soc)
                    except AttributeError:
                        self.logger.error("missing Soc param from module config." )
                        return -1
                    self.logger.info("Soc [%s]." % Soc)
                    pythonSVHome = ""
                    # read pythonSVHome from module config file
                    try:
                        pythonSVHome = str(self.configuration.pythonSVHome)
                    except AttributeError:
                        self.logger.error("missing pythonSVHome param from module config." )
                        return -1
                    self.logger.info("dump_location [%s]." % dump_location)
                    if not os.path.exists(dump_location):
                        self.logger.error("invalid destination folder" )
                        return -2
                    self.logger.info("pythonSVHome [%s]." % pythonSVHome)
                    sys.path.append(pythonSVHome)
                    dbgScript = pythonSVHome + r"\misc\OSV\debug_dump.py"
                    self.logger.info("dbgScript [%s]." % dbgScript)
                    if os.path.exists(dbgScript):
                        retVal = 1
                        try:
                            import misc.OSV.debug_dump as dump
                            retVal = dump.main(Soc, pythonSVHome, dump_location)
                        except Exception:
                            import traceback
                            exception_str = traceback.format_exc()
                            self.logger.error("Exception occurred while executing %s: %s" % (dbgScript, exception_str))

                        self.logger.info("debug_dump returned [%d]" % retVal)
                        # Power cycle the platform irrespective of dump script outcome
                        self.logger.info("Rebooting using relay.")
                        conn = (0, 0, 0, 0, 0)
                        try:
                            self.logger.info("Rebooting device [%s] " % self.device.config.device_name)
                            conn = self.device.init_device_connection(False, False, 1)
                        except Exception:
                            self.logger.error("Error rebooting platform: conn_status [%s]" % str(conn))
                            import traceback
                            exception_str = traceback.format_exc()
                            self.logger.info("Exception occurred while rebooting platform: %s" % (exception_str))
                        return retVal
                    else:
                        self.logger.error("Script [%s] not found" % dbgScript)
                        return -3
                else:
                    self.logger.info("Not a windows host, debug logs features not supported")
                    return -4
            else:
                self.logger.info("SocDebugModule disabled")
                return 0
        else:
            self.logger.info("Test verdict [%s], not collecting debug info" % verdict)
            return 0

    def cleanup(self):
        return