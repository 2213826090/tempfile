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
:summary: This file implements the LIVE_LAB_NFC_DEBUG_TEST
:author: apairex
:since:27/03/2013
"""
import time
from LIVE_NFC_BASE import LiveNfcBase
from UtilitiesFWK.Utilities import Global, str_to_bool
from acs_test_scripts.UseCase.Misc.UECMD_TEST_TOOL import UECmdTestTool


class LiveLabNfcDebugTest(LiveNfcBase, UECmdTestTool):

    """
    Live Lab NFC TEST UECmd UseCase
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call LiveBTBase init function
        LiveNfcBase.__init__(self, tc_name, global_config)
        UECmdTestTool.__init__(self, self.get_name(),
                               self._logger, self._device)

        self.__fcts2test = list()
        self.__robot_fcts2test = list()
        self.__secure_element_fcts2test = list()

        self.__use_robot = str(self._tc_parameters.get_param_value("USE_ROBOT"))
        if self.__use_robot.lower() in ["1", "true", "yes", "ok"]:
            self.__use_robot = True
        else:
            self.__use_robot = False

        self.__use_secure_element = str(self._tc_parameters.get_param_value("USE_SECURE_ELEMENT"))
        if self.__use_secure_element.lower() in ["1", "true", "yes", "ok"]:
            self._secure_element = str(self._tc_parameters.get_param_value("SECURE_ELEMENT"))
            self.__use_secure_element = True
        else:
            self.__use_secure_element = False

        self.__start_time = time.time()

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        LiveNfcBase.set_up(self)
        UECmdTestTool.set_up(self, self.tc_order)

        self.__use_robot = str_to_bool(str(self.__use_robot))

        """
        List of tests to perform. Each test is described as followed:
        [Label to print in the secondary report,
         UECmd to test,
         parameter(s) for the UECmd to test,
         Depends on the test names in the list]
        """
        nfc_api = self._nfc_api
        self.__fcts2test = [

            {self._FCT: nfc_api.exchange_apdu_using_scapi, self._PARMS: ["BASIC", "SMX", "4", 255, 1]},
            {self._FCT: nfc_api.get_nfc_beam_status},
            {self._FCT: nfc_api.enable_nfc_beam},
            {self._FCT: nfc_api.disable_nfc_beam},
            {self._FCT: nfc_api.force_nfc_state, self._PARMS: ["1"]},
            {self._FCT: nfc_api.nfc_touch_to_beam, self._PARMS: ["1024x768", "10"]},
            {self._FCT: nfc_api.get_nfc_status},
            {self._FCT: nfc_api.nfc_disable},
            {self._FCT: nfc_api.nfc_enable},
            {self._FCT: nfc_api.check_nfc_crash, self._PARMS: [self.__start_time]},
            {self._FCT: nfc_api.set_default_nfc_p2p_configuration, self._DEP: ["nfc_enable"]},
            {self._FCT: nfc_api.set_nfc_p2p_configuration, self._PARMS: ["passive", "target", "106"],
             self._DEP: ["nfc_enable"]}
        ]

        if self.__use_robot is True:
            self.__robot_fcts2test = [
                {self._FCT: nfc_api.read_nfc_tag},
                {self._FCT: nfc_api.write_nfc_tag, self._PARMS: ["RTD_TEXT", "RANDOM"]}
            ]

        if self.__use_secure_element is True:
            self.__secure_element_fcts2test = [
                {self._FCT: nfc_api.select_secure_element, self._PARMS: [self._secure_element]}
            ]

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        # Call LiveNfcBase base run_test function
        LiveNfcBase.run_test(self)

        # Check every EUCmd
        for fct2test in self.__fcts2test + self.__robot_fcts2test + self.__secure_element_fcts2test:
            self._check_uecmd(fct2test)
            time.sleep(self._wait_btwn_cmd)

        # Raise an Exception in case of all tests do not pass
        self._compute_general_verdict()

        return Global.SUCCESS, "All UECmds OK"
