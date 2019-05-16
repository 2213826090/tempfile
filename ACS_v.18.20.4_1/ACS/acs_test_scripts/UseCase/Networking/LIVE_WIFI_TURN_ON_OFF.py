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

:organization: INTEL NDG SW
:summary: This file implements the LIVE WIFI TURN ON OFF
:author: jreynaux
:since: 04/29/2014
"""

import time

from acs_test_scripts.UseCase.Networking.LIVE_WIFI_BASE import LiveWifiBase
from UtilitiesFWK.Utilities import Global


class LiveWifiTurnOnOff(LiveWifiBase):

    """
    Live BT Turn on off test.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        # Call LiveBTBase init function
        LiveWifiBase.__init__(self, tc_name, global_config)

        # Read TURN_WIFI_SEQUENCE from test case xml file
        self._turn_wifi_sequence = \
            str(self._tc_parameters.get_param_value("TURN_WIFI_SEQUENCE"))

        self._wifi_initial_state = None

#------------------------------------------------------------------------------

    def set_up(self):

        # Does not perform LiveWifiBase.set_up(self) because to much yet
        self._logger.debug("Storing initial wifi power status")
        self._wifi_initial_state = self._networking_api.get_wifi_power_status()

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """

        # Call UseCase base run_test function
        LiveWifiBase.run_test(self)

        time.sleep(self._wait_btwn_cmd)

        # begin wifi turn on off sequence
        self._logger.info("Wifi turn on off sequence is :"
                          + self._turn_wifi_sequence)
        time.sleep(self._wait_btwn_cmd)

        seqlist = self._turn_wifi_sequence.strip().split()
        for switch in seqlist:
            if switch.lower() in ("on", "1", 1):
                self._logger.info("try to turn on wifi")
                self._networking_api.set_wifi_power("1")
                time.sleep(self._wait_btwn_cmd)
            elif switch.lower() in ("off", "0", 0):
                self._logger.info("try to turn off wifi ")
                self._networking_api.set_wifi_power("0")
                time.sleep(self._wait_btwn_cmd)
            else:
                self._error.Code = Global.FAILURE
                self._error.Msg = \
                    "input wrong sequence , failed ." \
                    + "alter your sequence in test case xml file"

        return self._error.Code, self._error.Msg

#------------------------------------------------------------------------------

    def tear_down(self):

        # Does not perform LiveWifiBase.tear_down(self) because to much yet
        if self._wifi_initial_state is not None:
            self._logger.info("Restoring wifi initial state to %d" % self._wifi_initial_state)
            self._networking_api.set_wifi_power(self._wifi_initial_state)

        return Global.SUCCESS, "No errors"
