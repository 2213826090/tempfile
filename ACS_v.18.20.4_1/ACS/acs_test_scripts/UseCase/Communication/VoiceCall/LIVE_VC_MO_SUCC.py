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
:summary: This file implements the LIVE VC MO SUCC use case
:since: 19/12/2014
:author: amitrofx
"""

import time
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from UtilitiesFWK.Utilities import Global

class LiveVcMoSucc(UseCaseBase):

    """
    Live Voice Call MO/MR.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        UseCaseBase.__init__(self, tc_name, global_config)

        # Read callSetupTimeout from Phone_Catalog.xml
        self._call_setup_time = \
            int(self._dut_config.get("callSetupTimeout"))

        # Get Test Cases Parameters
        self._numtocall = self._tc_parameters.get_param_value("PHONE_NUMBER")

        self._callduration = \
            int(self._tc_parameters.get_param_value("CALL_DURATION"))

        self._number_of_calls = \
            int(self._tc_parameters.get_param_value("ITERATION_NO"))

        # Get UECmdLayer
        self._voicecall_api = self._device.get_uecmd("VoiceCall")
        self._networking_api = self._device.get_uecmd("Networking")

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        UseCaseBase.run_test(self)

        # Disable flight mode
        self._networking_api.set_flight_mode("off")

        # Release any previous call (Robustness)
        self._voicecall_api.release()

        # Make the call; wait for state ACTIVE to be reached for 2 seconds;
        # release the call and after that check that the DUT is in idle state
        # Repeat this several times (20)
        for i in range(1, self._number_of_calls):
            self._voicecall_api.dial(self._numtocall)
            self._voicecall_api.wait_for_state(self._uecmd_types.VOICE_CALL_STATE.ACTIVE, 2)
            self._voicecall_api.release()
            self._voicecall_api.check_state(self._uecmd_types.VOICE_CALL_STATE.NOCALL)

        # Make the call; and wait for 10 seconds
        self._voicecall_api.dial(self._numtocall)
        self._logger.info("Wait for call duration: " + str(self._callduration) + "s...")
        time.sleep(self._callduration)

        #Check that the phone is in ACTIVE state
        self._voicecall_api.check_state(self._uecmd_types.VOICE_CALL_STATE.ACTIVE)

        # Release the call and check that the phone is in idle stat
        self._voicecall_api.release()
        self._voicecall_api.check_state(self._uecmd_types.VOICE_CALL_STATE.NOCALL)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """
        UseCaseBase.tear_down(self)
        # Release any previous call (Robustness)
        self._voicecall_api.release()
        return Global.SUCCESS, "No errors"
