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
:summary: This file implements the LIVE VC MOMR UC
:since: 15/11/2011
:author: cbresoli
"""

import time

from acs_test_scripts.UseCase.Networking.LIVE_CELLULAR_BASE import LiveCellularBase
from UtilitiesFWK.Utilities import Global, str_to_bool


class LiveVcMoMr(LiveCellularBase):

    """
    Live Voice Call MO/MR.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        LiveCellularBase.__init__(self, tc_name, global_config)

        # Read callSetupTimeout from Phone_Catalog.xml
        self._call_setup_time = \
            int(self._dut_config.get("callSetupTimeout"))

        # Get Test Cases Parameters
        self._numtocall = self._tc_parameters.get_param_value("PHONE_NUMBER")

        self._callduration = \
            int(self._tc_parameters.get_param_value("CALL_DURATION"))

        # Read the parameter for call on hold/resume procedure
        self._check_on_hold_resume_procedure = str_to_bool(
            self._tc_parameters.get_param_value(
                "CALL_HOLD_RESUME",
                "False"))

        # Get UECmdLayer
        self._voicecall_api = self._device.get_uecmd("VoiceCall")

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        LiveCellularBase.run_test(self)

        # Release any previous call (Robustness)
        self._voicecall_api.release()

        self._voicecall_api.dial(self._numtocall)

        self._logger.info("Wait for call duration: " + str(self._callduration) + "s...")
        time.sleep(self._callduration)

        self._voicecall_api.check_state(self._uecmd_types.VOICE_CALL_STATE.ACTIVE)  # pylint: disable=E1101

        # Perform call hold and resume procedure if requested
        if self._check_on_hold_resume_procedure:
            self._logger.info("Call hold resume procedure will be checked")
            # Put the call on hold
            self._voicecall_api.switch_holding_and_active()
            current_state = self._voicecall_api.get_background_call_state()
            if str(current_state)  is "ON_HOLD":
                self._logger.info("Call State: " + str(current_state))
            else:
                self._logger.info("Call State: " + str(current_state))
                self._voicecall_api.release()
                return Global.FAILURE, "Call is not ON_HOLD"
            self._logger.info("Wait for call duration: " + str(self._callduration) + "s...")
            time.sleep(self._callduration)

            # Resume the call
            self._voicecall_api.switch_holding_and_active()
            self._voicecall_api.check_state(self._uecmd_types.VOICE_CALL_STATE.ACTIVE)  # pylint: disable=E1101

        # Release the call
        self._voicecall_api.release()

        return Global.SUCCESS, "No errors"

    def tear_down(self):
        """
        Disposes this test.
        """
        # Release the call
        self._voicecall_api.release()
        LiveCellularBase.tear_down(self)

        return Global.SUCCESS, "No errors"