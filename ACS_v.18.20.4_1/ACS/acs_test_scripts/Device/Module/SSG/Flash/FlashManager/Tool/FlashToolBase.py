"""

:copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
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

:organization: INTEL MCG PSI
:summary: This file implements the FlashToolBase class which is the generic class used to implement a specific flash tool
:since: 29/07/2013
:author: lbavois
"""
from acs_test_scripts.Device.Module.SSG.Flash.FlashManager.Tool.CmdExecManager import CmdExecManager
from ErrorHandling.AcsConfigException import AcsConfigException
from UtilitiesFWK.Utilities import Global
from Core.Report.ACSLogging import LOGGER_TEST_SCRIPT


class FlashToolBase(CmdExecManager):

    """
    This class is the generic class used by all specific flash tool for their implementation
    """

    def __init__(self, flash_tool_name, logger=None):
        """
        Constructor for Flash tool base

        :type flash_tool_name: str
        :param flash_tool_name: Name of the flash tool used
        :type logger: object
        :param logger: logger name to log info
        """
        self._flash_tool_name = flash_tool_name
        self._logger = logger if logger else LOGGER_TEST_SCRIPT
        self._flash_file = None
        CmdExecManager.__init__(self, self._flash_tool_name, self._logger)

        # Check flash tool availability
        self._flash_tool_cmd = self._check_flash_tool_availability()

        # Output datas for the flash procedure
        self.exec_result = False
        self.error_message = ""

    def _check_flash_tool_availability(self):
        """
        Check if the Flash Tool is installed over current ACS bench
        """
        raise AcsConfigException(AcsConfigException.FEATURE_NOT_IMPLEMENTED, "FlashToolBase: " +
                                                                             self._flash_tool_name + " flash tool is not installed over ACS bench")

    def _run(self):
        """
        Run the flash command
        """
        return CmdExecManager._run(self)

    def _stop(self):
        """
        Stop the flash command
        """
        CmdExecManager._stop(self)

    def _flash(self, timeout):
        """
        Start the flash execution for the flash tool

        :type timeout: int
        :param timeout: max time in seconds to do flash procedure
        :rtype: int
        :return: result of the flash procedure(Global.SUCCESS, Global.FAILURE, Global.BLOCKED)
        """

        # Start the flash tool thread
        CmdExecManager._start(self, timeout, execute_in_thread=False)

        # Check result
        if self.exec_result:
            self._logger.info("%s: FLASH SUCCESS (file= %s)" % (self._flash_tool_name, self._flash_file))
            flash_operation = Global.SUCCESS
        else:
            self._logger.error("%s: FLASH FAILURE (file= %s, error=%s)" %
                               (self._flash_tool_name, self._flash_file, self.error_message))
            flash_operation = Global.FAILURE

        return flash_operation
