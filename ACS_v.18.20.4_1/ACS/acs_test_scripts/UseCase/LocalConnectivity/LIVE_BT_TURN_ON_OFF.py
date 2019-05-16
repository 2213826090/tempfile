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
:summary: This file implements the LIVE BT TURN ON OFF IN SPEC MODE
:author: npan2
:since:09/09/2011
"""

import time

from acs_test_scripts.UseCase.LocalConnectivity.LIVE_BT_BASE import LiveBTBase
from UtilitiesFWK.Utilities import Global


class LiveBTTurnOnOff(LiveBTBase):

    """
    Live BT Turn on off test.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        # Call LiveBTBase init function
        LiveBTBase.__init__(self, tc_name, global_config)

        # Read TURN_BLUETOOTH_SEQUENCE from test case xml file
        self._turn_bluetooth_sequence = \
            str(self._tc_parameters.get_param_value("TURN_BLUETOOTH_SEQUENCE"))

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """

        # Call UseCase base run_test function
        LiveBTBase.run_test(self)

        time.sleep(self._wait_btwn_cmd)

        # begin bluetooth turn on off sequence
        self._logger.info("bluetooth turn on off sequence is :"
                          + self._turn_bluetooth_sequence)
        time.sleep(self._wait_btwn_cmd)

        seqlist = self._turn_bluetooth_sequence.strip().split()
        for switch in seqlist:
            if switch.lower() in ("on", "1", 1):
                self._logger.info("try to turn on bluetooth")
                self._bt_api.set_bt_power("1")
                time.sleep(self._wait_btwn_cmd)
            elif switch.lower() in ("off", "0", 0):
                self._logger.info("try to turn off bluetooth ")
                self._bt_api.set_bt_power("0")
                time.sleep(self._wait_btwn_cmd)
            else:
                self._error.Code = Global.FAILURE
                self._error.Msg = \
                    "input wrong sequence , failed ." \
                    + "alter your sequence in test case xml file"

        return self._error.Code, self._error.Msg
