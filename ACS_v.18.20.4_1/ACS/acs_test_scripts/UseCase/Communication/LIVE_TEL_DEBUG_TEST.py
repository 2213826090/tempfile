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
self._modified, published, uploaded, posted, transmitted, distributed, or
disclosed in any way without Intel's prior express written permission.

No license under any patent, copyright, trade secret or other intellectual
property right is granted to or conferred upon you by disclosure or delivery
of the Materials, either expressly, by implication, inducement, estoppel or
otherwise. Any license under such intellectual property rights must be express
and approved by Intel in writing.

:organization: INTEL MCG PSI
:summary: TEL - this UseCase test all UECmd used by all TEL UseCases
:author: lvacheyx
:since: 19/03/2013
"""

from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.Communication.LIVE_TEL_DEBUG_BASE import LiveTelDebugBase
from acs_test_scripts.UseCase.Misc.UECMD_TEST_TOOL import UECmdTestTool
import time


class LiveTelDebugTest(LiveTelDebugBase, UECmdTestTool):

    """
    Lab Telephony Debug Test class.
    """

    # Constant Android values for WIFI_SLEEP_POLICY
    WIFI_SLEEP_POLICY = {"DEFAULT": 0x0, "WHEN_SCREEN_OFF": 0x0,
                         "NEVER": 0x2, "NEVER_WHILE_PLUGGED": 0x1}

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call LiveTelDebugBase base Init function
        LiveTelDebugBase.__init__(self, tc_name, global_config)

        UECmdTestTool.__init__(self, self.get_name(),
                               self._logger, self._device)

        self._test_type = self._tc_parameters.\
            get_param_value("UECMD_TYPE_LIST",
                            "NETWORKING").upper()

        # If TC parameter not defined, then run all tests
        if self._test_type == "":
            self._test_type = "NETWORKING"

        self._fcts2test_vc = list()
        self._fcts2test_mod = list()
        self._fcts2test_mess = list()
        self._fcts2test_net = list()

#------------------------------------------------------------------------------
    def set_up(self):
        """
        Set up the test configuration
        """
        # Call Tel Debug Base Setup function
        LiveTelDebugBase.set_up(self)
        UECmdTestTool.set_up(self, self.tc_order)

        """
        List of tests to perform. Each test is described as followed:
        {Label to print in the secondary report,
         UECmd to test,
         parameter(s) for the UECmd to test,
         Depends on the test names in the list}
        """
        net = self._networking_api

        if "NETWORKING" in self._test_type:
            self._fcts2test_net = [
                {self._FCT: net.set_flight_mode, self._PARMS: ["0"]},
                {self._FCT: net.get_flight_mode},
                {self._FCT: net.activate_pdp_context, self._PARMS: [None, False]},
                {self._FCT: net.deactivate_pdp_context, self._PARMS: [None, False]},
                {self._FCT: net._get_pdp_context_status},
                {self._FCT: net.set_roaming_mode, self._PARMS: ["off"]},
                {self._FCT: net.get_roaming_mode},
                {self._FCT: net.set_apn},
                {self._FCT: net.usb_tether, self._PARMS: [1, 0, 0]}]

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        # Call the Tel Debug Base Run_test function
        LiveTelDebugBase.run_test(self)

        # Check every EUCmd
        for fct2test in self._fcts2test_net:
            self._check_uecmd(fct2test)
            time.sleep(self._wait_btwn_cmd)

        # Raise an Exception in case of all tests do not pass
        self._compute_general_verdict()

        return Global.SUCCESS, "All UECmds OK"

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """
        # Call the Tel Debug Base tear_down function
        LiveTelDebugBase.tear_down(self)

        return Global.SUCCESS, "No errors"
