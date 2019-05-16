"""
:copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
The source code contained or described herein and all documents related
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
:summary: This file implements the RAT check after flight mode
:since: 20/05/2015
:author: amurarux
"""

import time

from UseCase.UseCaseBase import UseCaseBase
from UtilitiesFWK.Utilities import Global


class LiveFlightModeCheckRat(UseCaseBase):

    """
    RAT check after flight mode, test steps :
    - Camp on a specific RAT
    - set flight mode ON
    - set flight mode OFF
    - check device is camping on the same RAT after flight mode
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call UseCaseBase init function
        UseCaseBase.__init__(self, tc_name, global_config)

        self._network_pref = \
            self._tc_parameters.get_param_value("PREFERRED_NETWORK_TYPE",
                                                None)

        # Get TC parameters
        self._registration_timeout = \
            int(self._dut_config.get("registrationTimeout"))

        # Arbitrary value for timeout in seconds
        self._flight_mode_timeout = 30

        if self._network_pref:
            self._network_pref = self._network_pref.upper()

        self._initial_pref_network = None

        # Instantiate UE Command categories
        self._networking_api = self._device.get_uecmd("Networking")
        self._modem_api = self._device.get_uecmd("Modem")

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Test setup
        """

        UseCaseBase.set_up(self)

        # Disable flight mode, if set
        if (self._networking_api.get_flight_mode() is "1"):
            self._networking_api.set_flight_mode("off")

        if self._network_pref is None:
            # If there is no Network preference in the testcase.
            self._network_pref = self._dut_config.get("defaultPreferredNetwork")
            self._logger.warning("No preferred network set in the testcase,"
                                 " will use the one currently set on the"
                                 " device catalog: %s" % str(self._network_pref))
        else:
            self._initial_pref_network = self._dut_config.get("defaultPreferredNetwork")
            time.sleep(self._wait_btwn_cmd)

        # Setting the DUT preferred network type
        self._networking_api.\
            set_preferred_network_mode(self._network_pref)
        # Check the DUT is camped on a compatible network with the selected
        # preferred network.
        self._modem_api.\
            check_rat_with_pref_network(self._network_pref,
                                            self._registration_timeout)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """

        UseCaseBase.run_test(self)

        # Enable flight mode on the device
        self._networking_api.set_flight_mode("on")

        self._logger.info("Sleeping %d seconds to ensure modem OFF state"
                        % self._flight_mode_timeout)
        time.sleep(self._flight_mode_timeout)

        # Disable flight mode on the device
        self._networking_api.set_flight_mode("off")

        self._logger.info("Sleeping %d seconds to ensure modem ON state"
                        % self._flight_mode_timeout)
        time.sleep(self._flight_mode_timeout)

        # Check the DUT is camped on a compatible network with the expected
        # preferred network.
        self._modem_api.\
            check_rat_with_pref_network(self._network_pref,
                                            self._registration_timeout)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        Tear down function
        """

        UseCaseBase.tear_down(self)

        # Set initial network state
        if self._initial_pref_network is not None:
            self._networking_api.\
                set_preferred_network_mode(self._initial_pref_network)
            time.sleep(self._wait_btwn_cmd)

        return Global.SUCCESS, "No errors"
