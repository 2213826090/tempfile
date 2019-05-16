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
:summary: This file implements the LAB_CWS_SYSTEM_DEBUG_TEST: tests system UE commands used in CWS tests (Wifi/BT/NFC)
:author: fchagnea
:since:06/06/2013
"""
import time
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.Misc.UECMD_TEST_TOOL import UECmdTestTool


class LabCwsSystemDebugTest(UseCaseBase, UECmdTestTool):

    """
    Lab CWS System TEST UECmd UseCase: tests system UE commands used in CWS tests (Wifi/BT/NFC)
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call UseCaseBase Init function
        UseCaseBase.__init__(self, tc_name, global_config)

        UECmdTestTool.__init__(self, self.get_name(),
                               self._logger, self._device)

        # Read UECMD_TYPE_LIST from TC parameters
        self.__uecmd_type = str(self._tc_parameters.get_param_value("UECMD_TYPE_LIST")).upper()

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        # Call UseCaseBase set_up function
        UseCaseBase.set_up(self)

        UECmdTestTool.set_up(self, self.tc_order)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """

        if "DISPLAY" in self.__uecmd_type:
            # test DISPLAY
            self.__display_test()

        # Raise an Exception in case of all tests do not pass
        self._compute_general_verdict()

        return Global.SUCCESS, "All UECmds OK"

#---------------------------------------------------------------------------
    def __display_test(self):
        """
        test all uecmd linked to display feature

        :return: None
        """
        tag = "DISPLAY_TEST"
        phonesystem_api = self._device.get_uecmd("PhoneSystem")

        fct_list = [
            # test the set screen timeout validity
            {self._FCT: phonesystem_api.set_screen_timeout,
             self._PARMS: [3600]},
            # test the get screen timeout validity
            {self._FCT: phonesystem_api.get_screen_timeout},
            # test the get screen status
            {self._FCT: phonesystem_api.get_screen_status},
            # test the set_phone_lock
            {self._FCT: phonesystem_api.set_phone_lock,
             self._PARMS: [0]}]

        self.__evaluate_fct_list(fct_list, tag)

#---------------------------------------------------------------------------

    def __evaluate_fct_list(self, fct_list, tag):
        """
        sub function to simplify redundant call

        :type fct: list
        :param fct: a list of dict describing functions to test

        :type tag: str
        :param tag: tag associated to these tests. Will be used for EM reports.
        """

        for fct2test in fct_list:
            # do a logical operation to catch fail
            self._check_uecmd(fct2test)
            time.sleep(self._wait_btwn_cmd)
