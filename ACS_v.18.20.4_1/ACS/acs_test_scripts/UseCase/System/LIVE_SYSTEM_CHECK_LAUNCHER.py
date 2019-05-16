"""
:copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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

:organization: INTEL NDG SW
:summary: This file is the implementation of
            CHECK_LAUNCHER use case
:since: 03/04/2014
:author: jreynaux
"""

import time
from ErrorHandling.AcsConfigException import AcsConfigException
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from UtilitiesFWK.Utilities import Global, str_to_bool
from acs_test_scripts.Device.UECmd.UECmdTypes import PACKAGE_STATE


class LiveSystemCheckLauncher(UseCaseBase):

    """
    This UC will allows user to check launcher installed.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call UseCase base Init function
        UseCaseBase.__init__(self, tc_name, global_config)

        # Get TC Parameters

        self._launcher_name = self._tc_parameters.get_param_value("LAUNCHER_NAME")

        self._open_chooser = str_to_bool(self._tc_parameters.get_param_value("OPEN_CHOOSER", "False"))

        # Get UECmdLayer
        self._system_api = self._device.get_uecmd("System")

        self._initial_package_state = None

#------------------------------------------------------------------------------
    def set_up(self):
        """
        Initialize the test
        """
        UseCaseBase.set_up(self)

        if self._launcher_name in [None, ""]:
            message = "Need a launcher name as package format (eg. com.android.launcher)"
            raise AcsConfigException(AcsConfigException.INVALID_TEST_CASE_FILE, message)

        self._initial_package_state = self._system_api.get_package_state(self._launcher_name)

        time.sleep(self._wait_btwn_cmd)

        if self._initial_package_state == PACKAGE_STATE.DISABLED:
            self._system_api.set_package_state(self._launcher_name, PACKAGE_STATE.ENABLED)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------
    def run_test(self):
        """
        Execute the test
        """
        status = Global.SUCCESS
        output = "Launcher %s found" % str(self._launcher_name)

        # Call UseCase base Run function
        UseCaseBase.run_test(self)

        time.sleep(self._wait_btwn_cmd)

        installed_launchers = self._system_api.get_installed_launchers(open_chooser=self._open_chooser)

        if self._launcher_name not in installed_launchers:
            status = Global.FAILURE
            output = "Unable to find launcher %s (or launcher disabled)" % str(self._launcher_name)
            return status, output
        else:
            self._logger.debug(output)

        if self._open_chooser:
            self._logger.info("Showing launcher chooser ...")
            time.sleep(10)

        return status, output

#------------------------------------------------------------------------------
    def tear_down(self):
        """
        End and dispose the test
        """
        UseCaseBase.tear_down(self)

        time.sleep(self._wait_btwn_cmd)

        # Re-check in case of unwanted change
        last_state = self._system_api.get_package_state(self._launcher_name)

        time.sleep(self._wait_btwn_cmd)

        # Restore initial package state
        if self._initial_package_state == PACKAGE_STATE.DISABLED and last_state == PACKAGE_STATE.ENABLED:
            self._system_api.set_package_state(self._launcher_name, PACKAGE_STATE.DISABLED)

        return Global.SUCCESS, "No errors"
